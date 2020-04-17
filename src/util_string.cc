// @file  util_string.cc
#include "util_string.h"

#include <algorithm>
#include <limits.h>
#include <unistd.h>
#include <stdio.h>

namespace utils {

    // Default implementation uses type const char
    std::string join_strings(const std::vector<std::string>& str_vec, const char sep)
    {
        return join_strings<char>(str_vec, sep);
    }

    // only implemented for char; string implementation would differ.
    std::vector<std::string> split_string(const std::string& str, const char sep)
    {
        std::vector<std::string> str_vec;
        std::string token;
        std::stringstream ss(str);
        while (std::getline(ss, token, sep)) {
            str_vec.push_back(token);
        }
        return str_vec;
    }

    //   From the C++20 future: https://en.cppreference.com/w/cpp/string/basic_string/starts_with
    bool string_starts_with(std::string const& str, std::string const& prefix)
    {
        if (prefix.size() > str.size()) {
            return false;
        }
        return equal(prefix.begin(), prefix.end(), str.begin());
    }

    //   From the C++20 future: https://en.cppreference.com/w/cpp/string/basic_string/ends_with
    bool string_ends_with(std::string const& str, std::string const& suffix)
    {
        if (suffix.size() > str.size()) {
            return false;
        }
        return equal(suffix.rbegin(), suffix.rend(), str.rbegin());
    }

    //   Return true IFF str contains sub_str
    bool string_contains(std::string const& str, std::string const& sub_str)
    {
        return str.find(sub_str) != std::string::npos;
    }

    // check if a string contains specified character
    bool string_contains_char(const std::string& name, const char char_to_find)
    {
        std::vector<std::string> str_vec = split_string(name, char_to_find);
        if( str_vec.size() == 1 ) {
            return false;   // does not contain char_to_find
        }
        return true;        // contains char_to_find
    }

    // get first substring before the first occurrence of a character.
    // Note: If the string does not contain this character, the method will return false
    bool get_string_before_char( const std::string& name
                               , std::string& string_before_char
                               , const char& sep
    ) {
        std::vector<std::string> str_vec = split_string(name, sep);
        string_before_char = str_vec[0];
        if (str_vec.size() < 2) {
            return false;
        }
        return true;
    }

    // replace a substring in a string with a new substring
    // return by reference, return false if substring_to_replace not found
    bool replace_substring( const std::string& orig_string
                          , std::string& new_string
                          , const std::string& substring_to_replace
                          , const std::string& replacement_substring
    ) {
        new_string = orig_string;
        if (new_string.find(substring_to_replace) != std::string::npos) {
            new_string = new_string.replace( new_string.find(substring_to_replace)
                                           , substring_to_replace.length()
                                           , replacement_substring
            );
            return true;
        }
        return false;
    }

    // Removes a single trailing slash character ('/') if present.
    // Returns a new string.
    std::string remove_trailing_slash(const std::string& path_input)
    {
        std::string path = path_input;
        if (path.length() > 0) {
            std::string::iterator it = path.end() - 1;
            if (*it == '/') {
                path.erase(it);
            }
        }
        return path;
    }

    // Removes one or more trailing instances of one character, if present.
    // Returns a new string.
    std::string trim_trailing_chars(const std::string& _string, const char *end_chars)
    {
        std::string result(_string);
        result.erase(result.find_last_not_of(end_chars) + 1);
        return result;
    }

    /// trim
    /// Removes leading and trailing whitespace (or other chars) from a std::string.
    /// From: http://www.cplusplus.com/forum/beginner/50209
    std::string& trim(std::string& _string, const char *end_chars)
    {
        _string.erase(0, _string.find_first_not_of(end_chars));
        _string.erase(_string.find_last_not_of(end_chars) + 1);
        return _string;
    }

    // Removes a single character from the front and/or back of the given string.
    // Returns a new string.
    std::string strip_front_and_back(const std::string& ins, const char end_char)
    {
        size_t length = ins.length();
        if (length < 2) {
            return ins;
        }
        if (ins.front() == end_char) {
            if (ins.back() == end_char) {
                return ins.substr(1, length - 2);
            } else {
                return ins.substr(1, length - 1);
            }
        } else if (ins.back() == end_char) {
            return ins.substr(0, length - 1);
        }
        return ins;
    }

    // Removes a double-quote char ('"') from the front and/or back of the given string.
    // Returns a new string.
    std::string strip_double_quotes(const std::string& instring)
    {
        return strip_front_and_or_back(instring, '"');
    }

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
