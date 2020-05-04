// @file: util_io.cc
#include "util_io.h"
#include "dexai_log.h"

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

    bool IsEnvCircleCi() { return getenv_var("CIRCLECI") == "true"; }

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

    // Timestamp used in log_dexai (dexai::log) directory names,
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
