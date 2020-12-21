// @file  util_string.cc
#include "util_string.h"

#include <limits.h>
#include <stdio.h>
#include <unistd.h>

#include <algorithm>

namespace utils {

std::string to_uppercase(const std::string& str) {
  std::string s = str;
  std::transform(s.begin(), s.end(), s.begin(),
                 std::ptr_fun<int, int>(std::toupper));
  return s;
}

std::string hostname_string() {
  char hostname[HOST_NAME_MAX];
  gethostname(hostname, HOST_NAME_MAX);
  std::string hostname_str(hostname);
  return hostname_str;
}

}  // namespace utils
