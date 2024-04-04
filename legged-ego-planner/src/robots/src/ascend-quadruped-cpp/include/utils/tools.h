/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: motor object on robots.
* Author: Zhu Yijie
* Create: 2021-11-22
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*/

#ifndef ASCEND_UTILS_TOOLS_H
#define ASCEND_UTILS_TOOLS_H

#include <iostream>
#include <string>
#include <limits.h>
#include <unistd.h>

namespace robotics
{
    namespace utils
    {
        /** @brief query the path of the executable file. */
        std::string GetExePath()
        {
            char result[PATH_MAX];
            std::size_t count = readlink("/proc/self/exe", result, PATH_MAX);
            return std::string(result, (count > 0) ? count : 0);
        }

    } // utils
} // robotics

#endif // ASCEND_UTILS_TOOLS_H