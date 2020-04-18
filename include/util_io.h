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

    yes_no_maybe::YesNoMaybe IsDrakeVisualizerRunning(uint verbose = 0);
    // bool UseDrakeVisualizer() { return false; } // STUB!
    // void UseDrakeVisualizer(bool use_it) { static bool do_use_ = false; do_use = use_it } // STUB!

    extern const char *TIMESTAMP_FORMAT;
    extern const int   CIRCLE_CI_NUM_THREADS;

    /// Set the logging level of Drake's console logger, if found.
    /// This is separate from setting the logging level of the momap logger.
    bool set_console_log_level( spdlog::level::level_enum level = spdlog::level::warn
                              , int verbose = 1 // 0: silent, 1: one message, 2: all messages
    );

    /// Convert a timestamp string to epoch seconds.
    /// NOTE: We assume time_t is an integral type, possibly unsigned.
    time_t seconds_from_ts_string(const std::string& ts);

    /// Timestamp used in log_momap (momap::log) directory names,
    /// in SG:ActionIndex/IndexedAction result maps, and other places.
    /// Gets the current localtime and
    /// returns it as a datetime string with YYYYMMDDThhmmss format.
    /// If the epoch_seconds argument non-zero, it is expected to ba
    /// a count of seconds since 1970 (Unix time).
    std::string date_time_string(time_t seconds = 0);

    inline size_t date_time_length() { return 15; }

    inline bool is_date_time_valid(const std::string& ts) {
        return ts.length() == date_time_length();
    }

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


    bool write_binary_file(const std::string& full_path, void *data, unsigned max_bytes);
    bool read_binary_file(const std::string& full_path, void *result, unsigned max_bytes);

    /// Finds or attempts to create a subdirectory under the caller's HOME
    /// directory, as for for robot logging.
    /// On success, returns true and places the path in log_path_out.
    /// On failure, returns false, and the value of log_path_out may be invalid.
    bool home_sub_dir(std::string& log_path_out, const std::string sub_dir_name_in);

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

    std::string get_cwd();

    /// determines if the input path identifies a directory (vs. file or ...?)
    bool is_dir(const std::string& path);

    // makes dirs recursively using path_str; returns 0 on success, negative int on error.
    /// If path_str idenfies an existing directory or file, this function does not change it.
    int make_dirs(const std::string& path_str);

    int rename_file(const std::string& old_name, const std::string& new_name);

    const std::string temp_dir();

    bool file_stat(const std::string& path, struct stat& result);

    inline bool file_exists(const std::string& path) {
        struct stat result;
        return stat(path.c_str(), &result) == 0;
    }

    inline bool file_exists_in_folder( const std::string folder_path
                                     , const std::string file_name
    ) {
        std::string full_path = folder_path;
        std::string last_char = std::to_string(folder_path.back());
        if (last_char != "/") {
            full_path = full_path + "/";
        }
        full_path = full_path + file_name;
        struct stat result;
        bool exists = file_stat(full_path, result);
        if ( ! exists) {
            momap::log()->debug("File {} does not exist in {}", file_name, folder_path);
        }
        else {
            momap::log()->debug("File {} exists in {}", file_name, folder_path);
        }
        return exists;
    }

    time_t file_mod_time(const std::string& path);

    /// Return the file name, stripped of any directory-like prefix.
    /// Uses filesystem::path::filename.
    fs::path file_name(const std::string& full_path);

    /// Returns the filename part of the given full_path as a std::string.
    /// @deprecated: Use file_name directly; fs::path autoconverts to std::string.
    inline std::string get_file_name(const std::string& full_path) {
        return file_name(full_path);
    }

    /// Parse out the base file name (no directory prefix and no file extension suffix)
    /// from a full file specification, as in: /home/$USER/.nose2.cfg -> nose2
    std::string file_base_name(const std::string& full_path);

    /// @deprecated (naming conventions): use file_base_name instead.
    inline std::string get_file_base_name(const std::string& full_path) {
        return file_base_name(full_path);
    }

    /// splith the path into two parts, on the last "." by default.
    bool split_file_path( const std::string& file_path
                        , std::string& beg
                        , std::string& end
                        , const std::string& sep = "."
    );

    /// get all the parts
    bool parse_file_spec( const std::string& full_path
                        , std::string& base_dir
                        , std::string& file_name
                        , std::string& file_base
                        , std::string& file_extn
    );

    char *get_program_full_path();

    std::string get_program_short_name();

    std::vector<std::string> glob_v(const std::string& pat);

    bool glob_b(const std::string& pat, std::vector<std::string>& result);

    template<typename TimeT>
    TimeT expect_file_mod(const std::string& temp_file_path, TimeT max_seconds)
    {
        TimeT time_mod = file_mod_time(temp_file_path);
        TimeT time_dif = time(NULL) - time_mod;
        #if defined(ASSERT_TRUE) || defined(assertTrue) // inside a test?
        if (max_seconds > 0) {
            EXPECT_LE(time_dif, max_seconds);
        }
        #endif
        if (time_dif > max_seconds) {
            std::cerr << "    Checked file: " << temp_file_path
                      << "    Seconds since file mod exceeds limit: "
                      << time_dif << " > " << max_seconds << std::endl;
        }
        return time_dif;
    }

    /// Returns just the file extension (to determine file type from file name)
    /// @deprecated (naming conventions)
    std::string GetFileExtension(const std::string& file_name);

    /// Returns the file extension without the ".", or "" if there is none.
    std::string get_file_extension(const std::string& file_spec);

    /// Renames an existing file named like file.ext into a series numbered back ups,
    /// as follows:
    /// file.<N-1>.ext => file.<N>.ext for N = max_numbered_backup_files down to 1;
    /// file.ext => file.1.ext
    /// If path does not include a file extension, then path => path.1, etc.
    /// Returns the number of numbered backup files left behind,
    /// which is 0 if path identifies a directory or anything but a rename-able file.
    int backup_to_num(const char *path, int max_numbered_backup_files = 9);

    /// remove directory's contents recursively, like system("rm -rf")
    int remove_dir_rec(const char *dir_spec);   /// returns 0 on success

    /// Copy one file to another using fstreams.  Efficient and portable.
    bool copy_file_stream(const std::string& src_path, const std::string& dst_path);

    /// Delete a file by calling unlink the path, if it is an ordinary file
    int delete_file(const std::string& path);

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
