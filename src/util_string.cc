// @file  util_string.cc
#include "util_string.h"

#include <algorithm>
#include <limits.h>
#include <unistd.h>
#include <stdio.h>

namespace utils {

    std::string to_uppercase(const std::string& str)
    {
        std::string s = str;
        std::transform(s.begin(), s.end(), s.begin(), std::ptr_fun<int, int>(std::toupper));
        return s;
    }

    std::string hostname_string()
    {
        char hostname[HOST_NAME_MAX];
        gethostname(hostname, HOST_NAME_MAX);
        std::string hostname_str(hostname);
        return hostname_str;
    }

}   // namespace utils
