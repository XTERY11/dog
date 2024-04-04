/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: compute the force for stance controller. 
* Author: Zang Yaohua
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#include "mpc_controller/qp_torque_optimizer.h"
#include "QuadProg++.hh"
#include "Array.hh"
namespace Quadruped {
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                              Eigen::Matrix<float, 3, 3> robotInertia,
                                              Eigen::Matrix<float, 4, 3> footPositions) {
    Eigen::Matrix<float, 3, 3> rotZ = Eigen::Matrix<float, 3, 3>::Identity(3, 3);
    Eigen::Matrix<float, 3, 3> invMass;
    Eigen::Matrix<float, 3, 3> invInertia;
    Eigen::Matrix<float, 6, 12> massMat = Eigen::Matrix<float, 6, 12>::Zero();
    Eigen::Matrix<float, 1, 3> x;
    Eigen::Matrix<float, 3, 3> footPositionSkew;

    invMass = rotZ / robotMass;
    invInertia = robotInertia.inverse();

    for (int legId = 0; legId < 4; ++legId) {
        massMat.block<3, 3>(0, legId * 3) = invMass;
        x = footPositions.row(legId);
        footPositionSkew << 0., -x[2], x[1],
                x[2], 0., -x[0],
                -x[1], x[0], 0.;
        massMat.block<3, 3>(3, legId * 3) = rotZ.transpose() * invInertia * footPositionSkew;
    }
    return massMat;
}

std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
        float mpcBodyMass,
        Eigen::Matrix<int, 4, 1> contacts,
        float frictionCoef,
        float fMinRatio,
        float fMaxRatio) {
    float fMin;
    float fMax;
    fMin = fMinRatio * mpcBodyMass * 9.8;
    fMax = fMaxRatio * mpcBodyMass * 9.8;
    Eigen::Matrix<float, 24, 12> A = Eigen::Matrix<float, 24, 12>::Zero();
    Eigen::Matrix<float, 24, 1> lb = Eigen::Matrix<float, 24, 1>::Zero();

    for (int legId = 0; legId < 4; legId++) {
        A(legId * 2, legId * 3 + 2) = 1;
        A(legId * 2 + 1, legId * 3 + 2) = -1;
        if (contacts(legId, 0) > 0) {
            lb(legId * 2, 0) = fMin;
            lb(legId * 2 + 1, 0) = -fMax;
        } else {
            lb(legId * 2, 0) = 1e-7;
            lb(legId * 2 + 1, 0) = 1e-7;
        }
    }
    // Friction constraints
    int rowId;
    int colId;
    for (int legId = 0; legId < 4; ++legId) {
        rowId = 8 + legId * 4;
        colId = legId * 3;
        lb.block<4, 1>(rowId, 0) << 0., 0., 0., 0.;
        A.block<1, 3>(rowId, colId) << 1, 0, frictionCoef;
        A.block<1, 3>(rowId + 1, colId) << -1, 0, frictionCoef;
        A.block<1, 3>(rowId + 2, colId) << 0, 1, frictionCoef;
        A.block<1, 3>(rowId + 3, colId) << 0, -1, frictionCoef;
    }
    std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> Alb(A.transpose(), lb);
    return Alb;
}

std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> ComputeObjectiveMatrix(
        Eigen::Matrix<float, 6, 12> massMatrix,
        Eigen::Matrix<float, 6, 1> desiredAcc,
        Eigen::Matrix<float, 6, 1> accWeight,
        float regWeight) {
    Eigen::Matrix<float, 6, 1> g = Eigen::Matrix<float, 6, 1>::Zero();
    g(2, 0) = 9.8;
    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Zero();
    for (int i = 0; i < 6; ++i) {
        Q(i, i) = accWeight(i, 0);
    }
    Eigen::Matrix<float, 12, 12> R = Eigen::Matrix<float, 12, 12>::Ones();
    R = R * regWeight;
    Eigen::Matrix<float, 12, 12> quadTerm;
    Eigen::Matrix<float, 12, 1> linearTerm;
    quadTerm = massMatrix.transpose() * Q * massMatrix + R;
    linearTerm = (g + desiredAcc).transpose() * Q * massMatrix;
    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> quadLinear(quadTerm, linearTerm);
    return quadLinear;
}

Eigen::Matrix<float, 3, 4> ComputeContactForce(Robot *quadruped,
                                               Eigen::Matrix<float, 6, 1> desiredAcc,
                                               Eigen::Matrix<int, 4, 1> contacts,
                                               Eigen::Matrix<float, 6, 1> accWeight,
                                               float regWeight,
                                               float frictionCoef,
                                               float fMinRatio,
                                               float fMaxRatio) {
    Eigen::Matrix<float, 6, 12> massMatrix;
    massMatrix = ComputeMassMatrix(quadruped->bodyMass,
                                   quadruped->bodyInertia,
                                   quadruped->GetFootPositionsInBaseFrame().transpose());
    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> Ga;
    Eigen::Matrix<float, 12, 12> G;
    Eigen::Matrix<float, 12, 1> a;
    Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight);
    G = std::get<0>(Ga);
    Eigen::Matrix<float, 12, 12> temp = Eigen::Matrix<float, 12, 12>::Identity();
    G = G + temp * 1e-4;
    a = std::get<1>(Ga);
    std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> CI;
    Eigen::Matrix<float, 12, 24> Ci;
    Eigen::Matrix<float, 24, 1> b;
    CI = ComputeConstraintMatrix(quadruped->bodyMass, contacts, frictionCoef, fMinRatio, fMaxRatio);
    Ci = std::get<0>(CI);
    b = std::get<1>(CI);

    quadprogpp::Matrix<double> GG(12, 12);

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            GG[i][j] = double(G(j, i));
        }
    }
    quadprogpp::Vector<double> aa(12);
    for (int i = 0; i < 12; i++) {
        aa[i] = double(-a(i, 0));
    }
    quadprogpp::Matrix<double> CICI(12, 24);
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 24; j++) {
            CICI[i][j] = double(Ci(i, j));
        }
    }
    quadprogpp::Vector<double> bb(24);
    for (int i = 0; i < 24; i++) {
        bb[i] = double(-b(i, 0));
    }
    quadprogpp::Matrix<double> CECE(12, 0);
    quadprogpp::Vector<double> ee(0);
    quadprogpp::Vector<double> x(12);
    quadprogpp::solve_quadprog(GG, aa, CECE, ee, CICI, bb, x);
    //reshape x from (12,) to (4,3)
    Eigen::Matrix<float, 4, 3> X;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            X(i, j) = -float(x[3 * i + j]);
        }
    }
    return X.transpose();
}

Eigen::Matrix<float, 3, 4> ComputeContactForceTest(float bodyMass,
                                                   Eigen::Matrix<float, 3, 3> bodyInertia,
                                                   Eigen::Matrix<float, 4, 3> footPositions,
                                                   Eigen::Matrix<float, 6, 1> desiredAcc,
                                                   Eigen::Matrix<int, 4, 1> contacts,
                                                   Eigen::Matrix<float, 6, 1> accWeight,
                                                   float regWeight,
                                                   float frictionCoef,
                                                   float fMinRatio,
                                                   float fMaxRatio) {
    Eigen::Matrix<float, 6, 12> massMatrix;
    massMatrix = ComputeMassMatrix(bodyMass,
                                   bodyInertia,
                                   footPositions);
    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> Ga;
    Eigen::Matrix<float, 12, 12> G;
    Eigen::Matrix<float, 12, 1> a;
    Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight);
    G = std::get<0>(Ga);
    Eigen::Matrix<float, 12, 12> temp = Eigen::Matrix<float, 12, 12>::Identity();
    G = G + temp * 1e-4;
    a = std::get<1>(Ga);
    std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> CI;
    Eigen::Matrix<float, 12, 24> Ci;
    Eigen::Matrix<float, 24, 1> b = Eigen::Matrix<float, 24, 1>::Zero();
    CI = ComputeConstraintMatrix(bodyMass, contacts, frictionCoef, fMinRatio, fMaxRatio);
    Ci = std::get<0>(CI);
    b = std::get<1>(CI);

    quadprogpp::Matrix<double> GG(12, 12);
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            GG[i][j] = double(G(j, i));
        }
    }
    quadprogpp::Vector<double> aa(12);
    for (int i = 0; i < 12; i++) {
        aa[i] = double(-a(i, 0));
    }
    quadprogpp::Matrix<double> CICI(12, 24);
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 24; j++) {
            CICI[i][j] = double(Ci(i, j));
        }
    }
    quadprogpp::Vector<double> bb(24);
    for (int i = 0; i < 24; i++) {
        bb[i] = double(-b(i, 0));
    }
    quadprogpp::Matrix<double> CECE(12, 0);
    quadprogpp::Vector<double> ee(0);
    quadprogpp::Vector<double> x(12);
    quadprogpp::solve_quadprog(GG, aa, CECE, ee, CICI, bb, x);
    //reshape x from (12,) to (4,3)
    Eigen::Matrix<float, 4, 3> X;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            X(i, j) = -float(x[3 * i + j]);
        }
    }
    return X.transpose();
}
} // namespace Quadruped