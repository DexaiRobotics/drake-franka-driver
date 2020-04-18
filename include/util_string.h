/// @file util_string.h
/// Namespaced utilities for strings
#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace utils {

    /// return uppercase version of string
    std::string to_uppercase(const std::string& str);

    /// return hostname of computer this is being run on
    std::string hostname_string();

}   //  namespace utils
