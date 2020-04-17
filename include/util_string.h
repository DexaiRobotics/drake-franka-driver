/// @file util_string.h
/// Namespaced utilities for strings
#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace utils {

/// Join a vector of strings into one string separated by a const char or string,
/// as for CSV output.
template <typename T>
std::string join_strings(const std::vector<std::string>& str_vec, const T& sep)
{
    const int len = str_vec.size();
    if (len) {
        std::ostringstream oss;
        oss << str_vec[0];
        for (int j = 1; j < len; j++) {
            oss << sep << str_vec[j];
        }
        return oss.str();
    }
    return "";
}

/// Overload with default argument of a single comma character.
/// Default argument declaration uses type const char (there can be only one).
std::string join_strings(const std::vector<std::string>& str_vec, const char sep = ',');

/// split a string on a single character into tokens (substrings),
/// and return them in a vector.  Whitespace is NOT trimmed.
std::vector<std::string> split_string(const std::string& str, const char sep = ',');

///  string_starts_with
///  From the C++20 future: https://en.cppreference.com/w/cpp/string/basic_string/starts_with
bool string_starts_with(std::string const& str, std::string const& prefix);

///  string_ends_with
///  From the C++20 future: https://en.cppreference.com/w/cpp/string/basic_string/ends_with
bool string_ends_with(std::string const& str, std::string const& suffix);

///  Return true IFF str contains sub_str
///  TODO @sprax: replace calls with std::string::contains in C++17.
bool string_contains(std::string const& str, std::string const& sub_str);

///  string_contains_char
///  check if a string contains specified character
bool string_contains_char(const std::string& name, const char char_to_find = '_');

///  get first substring before the first occurrence of a character.
///  Note: If the string does not contain this character, the method will return false
bool get_string_before_char( const std::string& name, std::string& string_before_char, const char& sep = '_');

///  replace a substring in a string with a new substring
///  return by referencce, return false if substring_to_replace not found
bool replace_substring( const std::string& orig_string
                      , std::string& new_string
                      , const std::string& substring_to_replace
                      , const std::string& replacement_substring
);

/// Removes a single trailing slash character ('/') if present.
/// Returns a new string.
std::string remove_trailing_slash(const std::string& path_input);

/// Removes one or more trailing instances of the chars in end_chars, if any are present.
/// Returns a new string.
std::string trim_trailing_chars(const std::string& _string, const char *end_chars = " ");

/// trim
/// Removes leading and trailing whitespace (or other chars) from a std::string.
/// From: http://www.cplusplus.com/forum/beginner/50209
/// @param _string The string to be modified.
/// @param end_chars The set of characters to erase from each end of the string.
/// @return The same string passed in as a parameter reference.
std::string& trim(std::string& _string, const char *end_chars = " \t\n\r\f\v");

/// Removes a single character from the front and or back of the given string.
/// Returns a new string.
std::string strip_front_and_back(const std::string& ins, const char end_char = '"');

/// Same as above, but:
/// @deprecated: bad name.  Use strip_front_and_back instead.
inline std::string strip_front_and_or_back(const std::string& ins, const char end_char = '"') {
    return  strip_front_and_back(ins, end_char);
}

/// Removes a double-quote char ('"') from the front and/or back of the given string.
/// Returns a new string.
std::string strip_double_quotes(const std::string& instring);

/// return uppercase version of string
std::string to_uppercase(const std::string& str);

/// return hostname of computer this is being run on
std::string hostname_string();

}   //  namespace utils
