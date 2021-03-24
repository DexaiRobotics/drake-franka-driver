/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Dexai Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/// @file: util_io.h
/// Namespaced utilities for file I/O and time (FIOT)

#ifndef INCLUDE_UTIL_IO_H_
#define INCLUDE_UTIL_IO_H_

#include <sys/stat.h>
#include <sys/time.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "utils/dexai_log.h"

namespace fs = std::filesystem;

namespace utils {

extern const char* TIMESTAMP_FORMAT;

/// Timestamp used in log_dexai (dexai::log) directory names,
/// in SG:ActionIndex/IndexedAction result maps, and other places.
/// Gets the current localtime and
/// returns it as a datetime string with YYYYMMDDThhmmss format.
/// If the epoch_seconds argument non-zero, it is expected to ba
/// a count of seconds since 1970 (Unix time).
std::string date_time_string(time_t seconds = 0);

//$ return today's date string with YYYYMMDD format
std::string get_date_string();

/// Is the calling process running in a CircleCI environment?
bool IsEnvCircleCi();

///  prompt user for input.
///  This routine shall be defined as a no-op in non-interactive runtime
///  environments. NOTE: We use std::cout instead of std::cerr for prompting
///  because we expect logging output to go to cerr by default, so cout output
///  may show up on the command line even if logging is redirected or silenced.
///  If you redirect both cerr and cout, you do not expect to get any prompts
///  anyway, right?
template <typename STR = std::string>
std::string PromptUserInput(bool do_prompt = true, STR prompt_str = "") {
  std::string answer;  // empty
  if (!do_prompt || IsEnvCircleCi()) {
    return answer;
  }
  // TODO(@dyt): SL.con.1: Prefer using STL array or vector instead of a C array
  std::string prompt_string {prompt_str};
  if (prompt_string.empty()) {
    std::cout << "PromptUserInput at " << __FILE__ << ":" << __LINE__
              << " - Press ENTER to continue...  " << std::endl;
  } else {
    std::cout << prompt_string << " hit RETURN... " << std::endl;
  }
  std::cin >> answer;
  return answer;
}

bool yes_no_prompt(const std::string& beg_prompt);

/// Finds or attempts to create a subdirectory under sub_dir_name_in.
/// On success, returns true and places the output path in sub_dir_path_out.
/// On failure, returns false, and the value of sub_dir_path_out may be invalid.
bool sub_dir_path(std::string& sub_dir_path_out,
                  const std::string& base_path_in,
                  const std::string& sub_dir_name_in);

std::string getenv_var(const std::string& var);

/// Get the current user's home directory path; result does not include any
/// trailing slash.
std::string get_home_dir();

std::string get_cwd();

// makes dirs recursively using path_str; returns 0 on success, negative int on
// error.
/// If path_str idenfies an existing directory or file, this function does not
/// change it.
int make_dirs(const std::string& path_str);

bool file_stat(const std::string& path, struct stat& result);

/// Return the file name, stripped of any directory-like prefix.
/// Uses filesystem::path::filename.
fs::path file_name(const std::string& full_path);

time_t file_mod_time(const std::string& path);

template <typename TimeT>
TimeT expect_file_mod(const std::string& temp_file_path, TimeT max_seconds) {
  if (!fs::exists(temp_file_path)) {
    throw std::runtime_error("expect_file_mod: path does not exist: "
                             + temp_file_path);
  }
  auto time_mod {file_mod_time(temp_file_path)};
  auto time_dif {static_cast<TimeT>(std::time(nullptr) - time_mod)};
  if (time_dif > max_seconds) {
    std::cerr << "    Checked file: " << temp_file_path
              << "    Seconds since file mod exceeds limit: " << time_dif
              << " > " << max_seconds << std::endl;
  }
  return time_dif;
}

int64_t get_current_utime();

/// Returns true if process with specified PID is currrently running.
bool process_is_alive(int pid);

/// Attempts to kill process with specified PID. Returns false if no such
/// process is found, or if killing the process fails.
bool kill_process(int pid);

/// Try to lock a file and write the current process ID to it.
/// Used to enforce singleton robot application, so no two instances of
/// the same application (or a generic robot application) can be running
/// at once.
bool lock_pid_file(const std::string& pid_file_path,
                   bool kill_existing_process = true,
                   bool prompt_before_kill = true);

}  //  namespace utils

#endif  // INCLUDE_UTIL_IO_H_
