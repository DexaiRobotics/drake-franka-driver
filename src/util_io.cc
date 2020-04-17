// @file: util_io.cc
#include "util_io.h"
#include "momap_log.h"

#include <cmath>
#include <cstdint>

#include <ftw.h>
#include <glob.h>

#include <errno.h>
#include <limits.h>                 // PATH_MAX
#include <pwd.h>
#include <stdlib.h>
#include <stdio.h>                  // defines FILENAME_MAX
#include <string.h>
#include <sys/stat.h>               // mkdir(2)
#include <sys/types.h>
#include <unistd.h>                 // rmdir & unlink
#include <signal.h>                 // kill

#include <semaphore.h>
#include <fcntl.h>

#include <iomanip>

#ifndef OPEN_MAX
#define OPEN_MAX 0                  // Guess 256 if it must be determinate
#endif


namespace utils {

    const char *TIMESTAMP_FORMAT = "%Y%m%dT%H%M%S";
    const int   CIRCLE_CI_NUM_THREADS = 2;

    bool IsEnvCircleCi() { return getenv_var("CIRCLECI") == "true"; }


    // TODO @sprax: implement
    yes_no_maybe::YesNoMaybe IsDrakeVisualizerRunning(uint verbose)
    {
        if (verbose) {
            log()->warn("DRU:IsDrakeVisualizerRunning: STUB, always returns Maybe.");
        }
        return yes_no_maybe::Maybe; // #stub STUB
    }

    // NOTE: PromptUserInput<> is defined the header file (util_io.h)
    bool yes_no_prompt(const std::string& beg_prompt)
    {
        // NOTE 1: On CircleCI, PromptUserInput returns "", which is taken as a 'no'.
        // NOTE 2: Beware that command line I/O buffering differs between Darwin and Ubuntu
        std::string prompt_str =  beg_prompt + " Enter (Y/n) then";
        auto answer = PromptUserInput(true, prompt_str);
        char firstc = answer.empty() ? '\0' : answer.front();
        if ( firstc == 'Y' || firstc == 'y' ) {
            return true;
        }
        std::cerr << "Taking '" << answer << "' as 'no'." << std::endl;
        return false;
    }

    bool set_console_log_level( spdlog::level::level_enum level
                              , int verbose     // 0: silent, 1: one message, 2: all messages
    ) {
        // Try to log first, so logger gets created if it does not already exist.
        // FIXME @sprax: namespace issue may cause first call to go to momap::log.
        if (verbose > 1) {
            drake::log()->warn("DRU: Trying to set console log level to {} ...", static_cast<unsigned>(level));
        } else {
            drake::log()->debug("DRU: Trying to set console log level to {} ...", static_cast<unsigned>(level));
        }
        std::shared_ptr<drake::logging::logger> conslog(spdlog::get("console"));
        if ( ! conslog) {
            if (verbose > 0) {
                log()->info("DRU:set_console_log_level: LOGGER NOT FOUND");
            }
            return false;
        }
        // Can this fail?   Throw?
        if (verbose > 1) {
            conslog->warn("DRU: Got console; will set log level to {}", (unsigned)level);
        } else {
            conslog->debug("DRU: Got console; will set log level to {}", (unsigned)level);
        }
        conslog->set_level(level);
        return true;
    }


    time_t seconds_from_ts_string(const std::string& ts)
    {
        time_t ts_seconds = 0;
        std::tm stm = {};
        std::istringstream iss(ts);
        iss >> std::get_time(&stm, TIMESTAMP_FORMAT);
        if (iss.fail() || -1 == (ts_seconds = std::mktime(&stm))) {
            throw std::invalid_argument("bad timestamp string: " + ts); // risky
        }
        return ts_seconds;
    }

    // Timestamp used in log_momap (momap::log) directory names,
    // in SG:ActionIndex/IndexedAction result maps, and other places.
    // The epoch_seconds argument is expected to ba a count of seconds since 1970 (Unix time).
    std::string date_time_string(time_t epoch_seconds)
    {
        time_t time = epoch_seconds ? epoch_seconds : std::time(nullptr);
        std::tm* stm = std::localtime(&time);        // TODO #time: standardize on localtime or UTC
        std::ostringstream iss;
        iss << std::put_time(stm, TIMESTAMP_FORMAT);
        return iss.str();
    }

    std::string get_date_string()
    {
        time_t time = std::time(nullptr);
        std::tm* stm = std::localtime(&time);
        std::ostringstream iss;
        iss << std::put_time(stm, "%Y%m%d");
        return iss.str();
    }

    bool write_binary_file(const std::string& full_path, void *data, unsigned max_bytes)
    {
        try {
            std::ofstream ofs(full_path, std::fstream::binary|std::fstream::out);
            ofs.write((char *)data, max_bytes);
            ofs.close();                            // TODO: needed?
            return true;
        } catch (const std::exception& ex) {
            log()->warn("Error in write_binary_file: {}", ex.what());
        }
        return false;
    }

    bool read_binary_file(const std::string& full_path, void *result, unsigned max_bytes)
    {
        try {   // Don't check specifics, just catch any error.
            std::ifstream ifs(full_path, std::fstream::binary|std::fstream::in);
            if (! ifs.good()) {
                log()->warn("Failed to open file ({}) for binary read", full_path);
                return false;
            }
            if (!ifs.read((char *)result, max_bytes)) {
                log()->warn("Error reading binary file ({})", full_path);
                return false;
            }
            return true;
        } catch (const std::exception& ex) {
            log()->warn("Error in read_binary_file: {}", ex.what());
        }
        return false;
    }


    // Finds or attempts to create the base directory for robot logging.
    // On success, returns true and places the path in log_path_out.
    // On failure, returns false, and the value of log_path_out may be invalid.
    bool home_sub_dir(std::string& log_path_out, const std::string sub_dir_path_in)
    {
        try {
            std::string home_dir = get_home_dir();
            return sub_dir_path(log_path_out, home_dir, sub_dir_path_in);
        } catch (const std::exception& ex) {
            log()->error( "Error in home_sub_dir (log_path_out={}): {}", log_path_out, ex.what());
        }
        return false;
    }

    // Finds or attempts to create a subdirectory under sub_dir_path_in.
    // On success, returns true and places the output path in sub_dir_path_out.
    // On failure, returns false, and the value of sub_dir_path_out may be invalid.
    bool sub_dir_path( std::string& sub_dir_path_out
                     , const std::string& base_path_in
                     , const std::string& sub_dir_path_in
    ) {
        try {
            sub_dir_path_out = base_path_in + "/" + sub_dir_path_in;
            int status = make_dirs(sub_dir_path_out);
            return 0 == status;
        } catch (const std::exception& ex) {
            log()->error( "Error in sub_dir_path (sub_dir_path_out={}): {}", sub_dir_path_out, ex.what());
        }
        return false;
    }

    std::string getenv_var(const std::string& var)
    {
         const char *val = std::getenv(var.c_str());
         if (val == 0) {
             return "";
         } else {
             return val;    // auto-converted
         }
    }

    /** Get the current user's home directory one way or another */
    std::string get_home_dir()
    {
        const char *val = std::getenv("HOME");
        return *val ? val : getpwuid(getuid())->pw_dir;
    }

    std::string get_cwd()
    {
        char buff[FILENAME_MAX];
        char *cwd = getcwd( buff, FILENAME_MAX );
        if (! cwd) {
            char *error = strerror(errno);
            log()->error( "Error in get_cwd from getcwd(): {}", error);
            return "";
        }
        return buff;    // auto-converted
    }

    // determines if the input path identifies a directory (vs. file or ...?)
    bool is_dir(const std::string& path)
    {
        struct stat result;
        if (stat(path.c_str(), &result) == 0) {
            if ( result.st_mode & S_IFDIR ) {
                return true;
            }
        }
        return false;
    }

    // makes dirs recursively using path_str; returns 0 on success, negative int on error.
    // If path_str identifies an existing directory or file, this function does not change it.
    int make_dirs(const std::string& path_str)
    {
        const char *path = path_str.c_str();
        /* Adapted from http://stackoverflow.com/a/2336245/119527 */
        const size_t len = strlen(path);
        char _path[PATH_MAX];
        char *p;

        errno = 0;

        /* Copy string so its mutable */
        if (len > sizeof(_path)-1) {
            errno = ENAMETOOLONG;
            return -1;
        }
        strcpy(_path, path);

        /* Iterate on the string */
        for (p = _path + 1; *p; p++) {
            if (*p == '/') {
                /* Temporarily truncate */
                *p = '\0';
                if (mkdir(_path, S_IRWXU) != 0) {
                    if (errno != EEXIST) {
                        return errno;
                    }
                }
                *p = '/';
            }
        }
        if (mkdir(_path, S_IRWXU) != 0) {
            if (errno != EEXIST) {
                char *error = strerror(errno);
                log()->error( "Error in make_dirs from mkdir({}): {}", path_str, error);
                return -2;
            }
        }
        return 0;
    }

    int rename_file(const std::string& old_name, const std::string& new_name)
    {
        int result = rename(old_name.c_str(), new_name.c_str());
        if (result != 0) {
            char* error = errno ? strerror(errno) : strerror(result);
            log()->error( "Error from rename_file({}, {}): {}", old_name, new_name, error);
            return result;
        }
        return 0;
    }

    const std::string temp_dir()
    {
        // return std::filesystem::temp_directory_path(); // (C++17)
        return "/tmp";
    }

    bool file_stat(const std::string& path, struct stat& result)
    {
        if (stat(path.c_str(), &result) == 0) {
            return true;
        }
        return false;
    }

    time_t file_mod_time(const std::string& path)
    {
        time_t file_mod_time = 0;
        struct stat result;
        if (stat(path.c_str(), &result) == 0) {
            file_mod_time = result.st_mtime;
        }
        return file_mod_time;
    }

    // Return the file name, stripped of any directory-like prefix.
    fs::path file_name(const std::string& full_path)
    {
        fs::path fs_path(full_path);
        return fs_path.filename();
    }

    // Return the file name, stripped of any directory-like prefix and any extension.
    std::string file_base_name(const std::string& full_path)
    {
        //  size_t dir_pos = full_path.find_last_of("/");
        std::string file_name = get_file_name(full_path);
        size_t dot_pos = file_name.find_last_of(".");
        return dot_pos == std::string::npos ? file_name : file_name.substr(0, dot_pos);
    }

    bool split_file_path( const std::string& file_path
                        , std::string& beg
                        , std::string& end
                        , const std::string& sep
    ) {
        size_t pos = file_path.find_last_of(sep);
        if (pos != std::string::npos) {
            beg = file_path.substr(0, pos);
            end = file_path.substr(pos);    // includes sep
            return true;
        }
        beg = file_path;
        end = "";
        return false;
    }

    bool parse_file_spec( const std::string& full_path
                        , std::string& base_dir
                        , std::string& file_name
                        , std::string& file_base
                        , std::string& file_extn
    ) {
        size_t dir_pos = full_path.find_last_of("/");
        if (dir_pos == std::string::npos) {
            base_dir  = "";
            file_name = full_path;
        } else {
            base_dir  = full_path.substr(0, dir_pos + 1);   // include the trailing '/'
            file_name = full_path.substr(dir_pos + 1);
        }
        size_t dot_pos = file_name.find_last_of(".");
        if (dot_pos == std::string::npos) {
            file_base = file_name;
            file_extn = "";
        } else {
            file_base = file_name.substr(0, dot_pos);
            file_extn = file_name.substr(dot_pos);      // include the leading '.'
        }
        return true;
    }


    // old-school way of getting the executable's full path in a *nix-like system.
    char *get_program_full_path()
    {
        char *path = (char *)malloc(PATH_MAX);
        if (path != NULL) {
            if (readlink("/proc/self/exe", path, PATH_MAX) == -1) {
                free(path);
                path = NULL;
            }
        }
        return path;
    }

    std::string get_program_short_name()
    {
        std::string full_path = get_program_full_path();
        return file_base_name(full_path);
    }

    std::vector<std::string> glob_v(const std::string& pat)
    {
        using namespace std;
        glob_t glob_result;
        glob(pat.c_str(), GLOB_TILDE, NULL, &glob_result);
        vector<string> ret;
        for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
            ret.push_back(string(glob_result.gl_pathv[i]));
        }
        globfree(&glob_result);
        return ret;
    }

    bool glob_b(const std::string& pat, std::vector<std::string>& result)
    {
        using namespace std;
        glob_t glob_result;
        glob(pat.c_str(), GLOB_TILDE, NULL, &glob_result);
        result = vector<string>();
        for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
            result.push_back(string(glob_result.gl_pathv[i]));
        }
        globfree(&glob_result);
        return result.size() > 0;
    }


    // @deprecated (naming conventions)
    std::string GetFileExtension(const std::string& file_name)
    {
        return get_file_extension(file_name);
    }

    // return the file extension without the ".", or "" if there is none.
    std::string get_file_extension(const std::string& file_spec)
    {
        size_t pos = file_spec.find_last_of(".");
        if (pos != std::string::npos) {
            return file_spec.substr(pos + 1);
        }
        return "";
    }


    // call rmdir or unlink on the path, depending on the flag
    static int rm_dir_or_file( const char *path
                             , const struct stat *pStat // unused, but required in signature for nftw
                             , int flag
                             , struct FTW *pFtw         // unused, but required in signature for nftw
    ) {
        int status = 0;
        (void) pStat;
        (void) pFtw;
        if (flag == FTW_DP) {
            status = rmdir(path);
            // cerr << "status = rmdir(" << path << "); => " << status << endl;
        } else {
            status = unlink(path);
            // cerr << "status = unlink(" << path << "); => " << status << endl;
        }
        if (status != 0) {
            perror(path);
        }
        return status;
    }


    // remove directory's contents recursively, like system("rm -rf")
    int remove_dir_rec(const char *dir_spec)
    {
        if (nftw(dir_spec, rm_dir_or_file, OPEN_MAX, FTW_DEPTH)) {
            perror(dir_spec);
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    }


    // Renames an existing file named like file.ext into a series numbered back ups,
    // as follows:
    // file.<N-1>.ext => file.<N>.ext for N = max_numbered_backup_files down to 1;
    // file.ext => file.1.ext
    // If path does not include a file extension, then path => path.1, etc.
    // Returns the number of numbered backup files left behind,
    // which is 0 if path identifies a directory or anything but a rename-able file.
    int backup_to_num(const char *path, int max_numbered_backup_files)
    {
        int err, num_renamed = 0;
        if (nullptr == path || 0 == strlen(path)) {
            log()->error("backup_to_num: empty file name");
            return -1;
        }
        struct stat result;
        if (0 != (err = stat(path, &result))) {
            perror(path);
            return -2;
        }
        if (result.st_mode & S_IFDIR) {
            log()->error("backup_to_num: path ({}) is a directory");
            return -3;
        }
        std::string base, extn, backup_file;
        split_file_path(path, base, extn);
        std::string backup_plus_1 = base + "." + std::to_string(max_numbered_backup_files)
                                  + extn;
        for (int j = max_numbered_backup_files; --j > 0; ) {
            backup_file = base + "." + std::to_string(j) + extn;
            if (0 == stat(backup_file.c_str(), &result)) {
                log()->debug("backup_to_num: moving {} to {}", backup_file, backup_plus_1);
                if (0 != rename_file(backup_file, backup_plus_1)) {
                    return -4 - j;
                }
                num_renamed++;
            }
            backup_plus_1 = backup_file;
        }
        log()->debug("backup_to_num: moving {} to {}", path, backup_file);
        if (0 != rename_file(path, backup_file)) {
            return -5 - max_numbered_backup_files;
        }
        return 1 + num_renamed;
    }


    // Copy one file to another using fstreams.  Efficient and portable.
    bool copy_file_stream(const std::string& src_path, const std::string& dst_path)
    {
        try {
            std::ifstream src_stream(src_path, std::fstream::binary);
            std::ofstream dst_stream(dst_path, std::fstream::trunc|std::fstream::binary);
            dst_stream << src_stream.rdbuf();
            return true;
        } catch (const std::exception& ex) {
            log()->warn("Error in copy_file_stream: {}", ex.what());
        }
        return false;
    }

    // Delete a file by calling unlink the path, if it is an ordinary file
    int delete_file(const std::string& path)
    {
        int status = unlink(path.c_str());
        if (status != 0) {
            perror(path.c_str());
        }
        return status;
    }

    int64_t get_current_utime() {
        struct timeval  tv;
        gettimeofday(&tv, NULL);
        int64_t current_utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
        return current_utime;
    }

    bool process_is_alive(int pid) {
        int ret = kill(pid, 0);
        if (ret == 0) {
            std::cout << "Proceess with PID " << pid << " is alive" << std::endl;
            return true;
        }
        else {
            char* error = strerror(errno);
            std::cerr << "Checking status of process with PID " << pid;
            std::cerr << ": " << error << std::endl;
            errno = 0;
        }
        return false;
    }

    bool kill_process(int pid) {
        if (!process_is_alive(pid)) {
            std::cout << "Process with PID " << pid << " is already terminated" << std::endl;
            return true;
        }
        else {
            // polite request to terminate
            int ret = kill(pid, SIGTERM);
            if (ret) {
                char* error = strerror(errno);
                std::cout << "Error attempting to kill PID " << pid;
                std::cout << " with SIGTERM:" << error << std::endl;
                errno = 0;
            }
            usleep(1e6);
            if (!process_is_alive(pid)) {
                std::cout << "Successfully killed PID " << pid << " with SIGTERM" << std::endl;
                return true;
            }
            else {
                // no really you should just die immediately
                ret = kill(pid, SIGKILL);
                if (ret) {
                    char* error = strerror(errno);
                    std::cout << "Error attetmpting to kill PID " << pid;
                    std::cout << " with SIGKILL:" << error << std::endl;
                    errno = 0;
                }
                usleep(1e6);
                if (!process_is_alive(pid)) {
                    std::cout << "Successfully killed PID " << pid << " with SIGKILL" << std::endl;
                    return true;
                }
            }
        }
        std::cerr << "dru:kill_process: failed to kill PID " << pid << "?" << std::endl;
        return false;
    }

    bool lock_pid_file( const std::string& pid_file_path
                      , bool kill_existing_process
                      , bool prompt_before_kill
    ) {

        int pid_file_handle = open(pid_file_path.c_str(), O_CREAT | O_RDWR, 0666);

        if (pid_file_handle == -1)  {
            // couldn't open lock file
            char* error = strerror(errno);
            std::cerr << "Could not open PID lock file " << pid_file_path;
            std::cerr << ", error " << error << std::endl;
            errno = 0;
            return false;
        }

        // try to lock file
        while (lockf(pid_file_handle, F_TLOCK, 0) == -1)
        {
            // couldn't get lock on lock file
            std::cerr << "Could not lock PID lock file " << pid_file_path << std::endl;
            FILE *f = fdopen(pid_file_handle, "r+");
            if (f == NULL) {
                char* error = strerror(errno);
                std::cerr << "Error opening " << pid_file_path;
                std::cerr << ", error: " << error << std::endl;
                errno = 0;
                return false;
            }
            int other_pid;
            int scanf_result = fscanf(f, "%d", &other_pid);
            if (scanf_result != 1) {
                std::cerr << "Error reading PID from file " << pid_file_path << std::endl;
                errno = 0;
                return false;
            }
            fclose(f);

            std::cerr << "Lock is held by PID " << other_pid << std::endl;
            if (!kill_existing_process) {
                return false;
            }
            else {
                if (prompt_before_kill) {
                    bool confirm_kill = yes_no_prompt("Kill existing app?");
                    if (!confirm_kill) {
                        return false;
                    }
                }
                if (!kill_process(other_pid)) {
                    return false;
                }
                return lock_pid_file(pid_file_path, kill_existing_process);
            }
        }

        // get and format PID
        int pid = getpid();
        char pid_str[10];
        int printf_result = snprintf(pid_str, sizeof(pid_str), "%d\n", pid);
        if (printf_result < 0 || size_t(printf_result) >= sizeof(pid_str)) {
            std::cerr << "Error formatting PID " << pid;
            if (errno) {
                char* error = strerror(errno);
                std::cerr << ": " << error << std::endl;
                errno = 0;
            }
            else {
                std::cerr << std::endl;
            }
            return false;
        }

        // truncate the file to prevent partial overwriting if
        // new PID is shorter than old
        int truncate_status = ftruncate(pid_file_handle, 0);
        if (truncate_status < 0) {
            char* error = strerror(errno);
            std::cerr << "Error truncating " << pid_file_path;
            std::cerr << ": " << error << std::endl;
            errno = 0;
            return false;
        }

        // write pid to lockfile
        ssize_t write_status = write(pid_file_handle, pid_str, strlen(pid_str));
        if (write_status < 0 || size_t(write_status) >= sizeof(pid_str)) {
            std::cerr << "Error writing to file " << pid_file_path;
            if (errno) {
                char* error = strerror(errno);
                std::cerr << ": " << error << std::endl;
                errno = 0;
            }
            else {
                std::cerr << std::endl;
            }
            return false;
        }

        return true;
    }

}   // namespace utils
