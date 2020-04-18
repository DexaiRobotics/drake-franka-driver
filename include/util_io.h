/// @file: util_io.h
/// Namespaced utilities for file I/O and time (FIOT)
#pragma once

#include "momap_log.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h>               // mkdir(2), stat
#include <time.h>
#include <vector>
#include <sys/time.h>
#include <thread>

// #include <experimental/filesystem>
// namespace fs = std::experimental::filesystem;
#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;

namespace utils {

    using momap::log;

    /// Three-valued answer logic.
    namespace yes_no_maybe {    // Is this the most idiomatic or "natural" order?
        typedef enum {
            No      =  -1,      // negative.  Note, as an unsigned, this may be UINT_MAX.  TODO: find out.
            Maybe   =   0,      // neutral
            Yes     =   1,      // positive
        }   YesNoMaybe;         // Examples: Responses to user prompts or system queries
    }

    extern const char *TIMESTAMP_FORMAT;

    /// Timestamp used in log_momap (momap::log) directory names,
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
    ///  This routine shall be defined as a no-op in non-interactive runtime environments.
    ///  NOTE: We use std::cout instead of std::cerr for prompting because we expect
    ///  logging output to go to cerr by default, so cout output may show up on the
    ///  command line even if logging is redirected or silenced.
    ///  If you redirect both cerr and cout, you do not expect to get any prompts anyway,
    ///  right?
    template<typename STR = std::string>
    std::string PromptUserInput( bool do_prompt = true
                               , STR prompt_str = ""
    ) {
        std::string answer;         // empty
        if ( do_prompt && ! IsEnvCircleCi()) {
            const size_t BUFSIZE = 128;
            char line[BUFSIZE];
            std::string prompt_string(prompt_str);
            if (prompt_string.empty()) {
                std::cout << "PromptUserInput at " << __FILE__ << ":" << __LINE__
                          << " - Press ENTER to continue...  " << std::endl;
            } else {
                std::cout << prompt_string << " hit RETURN... " << std::endl;
            }
            if (fgets(line, BUFSIZE, stdin) != 0) {
                line[strcspn(line, "\n")] = '\0';
                answer = line;
                //  std::cout << "Got <" << answer << ">, continuing..." << std::endl;
            }
        }
        return answer;
    }

    bool yes_no_prompt(const std::string& beg_prompt);

    /// Finds or attempts to create a subdirectory under sub_dir_name_in.
    /// On success, returns true and places the output path in sub_dir_path_out.
    /// On failure, returns false, and the value of sub_dir_path_out may be invalid.
    bool sub_dir_path( std::string& sub_dir_path_out
                     , const std::string& base_path_in
                     , const std::string& sub_dir_name_in
    );

    std::string getenv_var(const std::string& var);

    /// Get the current user's home directory path; result does not include any trailing slash.
    std::string get_home_dir();

    // makes dirs recursively using path_str; returns 0 on success, negative int on error.
    /// If path_str idenfies an existing directory or file, this function does not change it.
    int make_dirs(const std::string& path_str);

    bool file_stat(const std::string& path, struct stat& result);

    inline bool file_exists(const std::string& path) {
        struct stat result;
        return stat(path.c_str(), &result) == 0;
    }

    /// Return the file name, stripped of any directory-like prefix.
    /// Uses filesystem::path::filename.
    fs::path file_name(const std::string& full_path);

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
    bool lock_pid_file( const std::string& pid_file_path
                      , bool kill_existing_process = true
                      , bool prompt_before_kill = true
    );

}   //  namespace utils
