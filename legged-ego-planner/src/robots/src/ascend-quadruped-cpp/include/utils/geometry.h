#ifndef ASCEND_MATH_GEOMETRY_H
#define ASCEND_MATH_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <type_traits>
#include "utils/algebra.h"
#include "utils/cppTypes.h"

namespace robotics {
    namespace math {
        /**
         * @brief Spline class defines an abstract class for different spline interpolations
         */
        class Spline {
        public:
            /**
             * @brief record the spatial information, including the time dervative.
             */
            struct Point {
                Point() : x(0.0), xd(0.0), xdd(0.0) {}
                
                Point(float p, float v=0.0, float a=0.0) : x(p), xd(v), xdd(a) {}
                
                void setZero()
                {
                    x = 0.0;
                    xd = 0.0;
                    xdd = 0.0;
                }
                float x;
                float xd;
                float xdd;
            };
            
            Spline::Point operator=(const Spline::Point &rhs)
            {
                Spline::Point out;
                out.x = rhs.x;
                out.xd = rhs.xd;
                out.xdd = rhs.xdd;
                return out;
            }

            /** @brief Constructor function */
            Spline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            Spline(const float &initial_time,
                   const float &duration,
                   const Point &start,
                   const Point &end);

            /** @ Destructor function */
            virtual ~Spline() = 0;

            /**
             * @brief Sets the boundary of the spline
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            void setBoundary(const float &initial_time,
                             const float &duration,
                             const Point &start_p,
                             const Point &end_p);

            /**
             * @brief Sets the boundary of the spline
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const float& Start point
             * @param const float& End point
             */
            void setBoundary(const float &initial_time,
                             const float &duration,
                             const float &start_p,
                             const float &end_p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            virtual bool getPoint(const float &current_time,
                                  Point &p) = 0;

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            virtual bool getPoint(const float &current_time,
                                  float &p) = 0;

            bool isTimeElapsed(float &time);

        protected:
            /** @brief Initial time of the spline */
            float initial_time_;

            /** @brief Duration of the spline */
            float duration_;

            /** Start point of the spline */
            Point start_;

            /** @brief End point of the spline */
            Point end_;
        };

        inline bool Spline::isTimeElapsed(float &t)
        {
            //this makes sense only without the time interval
            if ((t - initial_time_) > duration_)
                return true;
            else
                return false;
        }

        /**
         * @brief CubicSpline class defines a cubic spline interpolation
         */
        class CubicSpline : public Spline {
        public:
            /** @brief Constructor function */
            CubicSpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            CubicSpline(const float &initial_time,
                        const float &duration,
                        const Point &start,
                        const Point &end);

            /** @ Destructor function */
            ~CubicSpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            bool getPoint(const float &current_time,
                          Point &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            bool getPoint(const float &current_time,
                          float &p);
        };

        /**
         * @brief FifthOrderPolySpline class defines a 5-order spline interpolation
         */
        class FifthOrderPolySpline : public Spline {
        public:
            /** @brief Constructor function */
            FifthOrderPolySpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            FifthOrderPolySpline(const float &initial_time,
                                 const float &duration,
                                 const Point &start,
                                 const Point &end);

            /** @ Destructor function */
            ~FifthOrderPolySpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            bool getPoint(const float &current_time,
                          Point &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            bool getPoint(const float &current_time,
                          float &p);
        };

        /**
         * @brief LinearSpline class defines a linear spline interpolation
         */
        class LinearSpline : public Spline {
        public:
            /** @brief Constructor function */
            LinearSpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            LinearSpline(const float &initial_time,
                         const float &duration,
                         const Point &start,
                         const Point &end);

            /** @ Destructor function */
            ~LinearSpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            bool getPoint(const float &current_time, Point &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            bool getPoint(const float &current_time, float &p);
        };
    } //  namespace math
} // namespace robotics

#endif//ASCEND_MATH_GEOMETRY_H