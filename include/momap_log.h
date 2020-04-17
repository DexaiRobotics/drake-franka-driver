/// @file: momap_log.h
#pragma once
/// NEW GOALS:
/// In general, do not shadow drake::log(), because that makes the original
/// drake logger inaccessible wherever it is shadowed.  In that case Drake
/// libraries will themselves still call drake::log() with the [console] label
/// writing to stderr, and our code will not be able to change its settings.
/// So define OVERRIDE_DRAKE_LOG 0

#include "drake/common/text_logging.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

#define OVERRIDE_DRAKE_LOG 1    // TODO: Move this definition to the build system environment

#ifdef HAVE_SPDLOG
using slogger = drake::logging::logger;
#else
using slogger = spdlog::logger;
#endif

/// Declare a logger based on spdlog (or nothing).  We can use it regardless.
class MomapLogger : public slogger
{
public:
    template<class It>
    MomapLogger(const std::string& name, const std::string& log_dir, const It &begin, const It &end)
        : slogger(name, begin, end), log_dir_(log_dir)
    { }

    inline const std::string& log_dir() { return log_dir_; }

    /// Returns the length of the log message identifier string (timestamp + prefix + log-level),
    /// like len("[2019-10-28 12:35:00.981] [momap] [error]")
    /// NOTE: defautl argument value is spdlog::level::error (not info) because the
    /// expected use case is for multiline indented error messages.
    unsigned log_id_size(spdlog::level::level_enum ll_enum = spdlog::level::err);

    ///  Support adding a sink at runtime.  Somewhat risky.  remove_sink would be worse.
    inline void add_sink(spdlog::sink_ptr sink)    // non-const
    {
        sinks_.push_back(sink);
    }

private:
    const std::string log_dir_;
    unsigned log_id_error_size_ = 42;   // based on "[2019-10-27 14:14:48.870] [momap] [error] "
};


/// Goals:
/// * Leave drake's "console" stderr logger visible by default, or, optionally,
/// * overload and hide drake::log() by header declaration; replace it by linkage.
/// * If specified, add a file to the Momap (replacement) logger's sinks;
///   - Use a default file if the specified file is not found.
/// * Use this logger for Writing out params in yaml format with timestamp
///     at start and finish of a Parlor + StuffGetter session.
///
/// @deprecated: Use momap::create_log instead.
// void create_momap_log( const std::string& program = "momap"
//                      , const std::string& base_path = ""    // If this is empty, we use the user's HOME/log_robot directory.
//                      , const std::string& prefix = ""  // For use in format: [timestamp] [prefix] [level] message
//                      , bool use_stdout = false
//                      , bool create_once = true
// );


namespace momap {

    /// Returns true IFF the logger object is NULL,
    /// meaning that either is has not yet been created, or it was nullified.
    bool is_log_null();

    /// Expose the singular momap logger, whether drake::log is shadowed or not.
    spdlog::logger* log(int warn_if_null = 1);

    /// Convenience global function to get the "current" global momap logger's log directory.
    /// If the global logger (the one returned by momap::log()) has not yet been created,
    /// calling this will get it created (with warnings about default settings).
    /// If the current logger is not an instance of MomapLogger, thie function will return
    /// an empty string.  We return a raw string, not a const reference, out of concern for
    /// library boundaries on Linux.
    std::string log_dir();

    /// Convenience function to get the "current" logger's log message ID size.  @see log_momap.h
    /// Example usage:
    /// log()->error("This is a test of momap::MomapLogger::log_id_size:\n{:>{}}"
    ///              "This line shall line up with the line up one line.",
    ///              "", momap::log_id_size()
    /// );
    unsigned log_id_size(spdlog::level::level_enum ll_enum = spdlog::level::err);

    /// Goals:
    /// * Leave drake's "console" stderr logger visible by default, or, optionally,
    /// * overload and hide drake::log() by header declaration; replace it by linkage.
    /// * If specified, add a file to the Momap (replacement) logger's sinks;
    ///   - Use a default file if the specified file is not found.
    /// * Use this logger for Writing out params in yaml format with timestamp
    ///     at start and finish of a Parlor + StuffGetter session.
    ///
    bool create_log( const std::string& program = "momap"
                   , const std::string& base_path = ""    // If this is empty, we use the user's HOME/log_robot directory.
                   , const std::string& prefix = ""  // For use in format: [timestamp] [prefix] [level] message
                   , bool use_stdout = false
                   , bool create_once = true
    );

    /// Returns: const pointer to array of 7 string views glossing spdlog level names.
    /// Example: const auto log_level_names = SpdLogLevelNames();
    ///          const std::string& warn_view = log_level_names[pdlog::level::warn];
    // const std::vector<std::string>& SpdLogLevelNames();

}

#if OVERRIDE_DRAKE_LOG
namespace drake {
    // Shadow drake::log() with an overload.  NOTE: s_logger must be initialized elsewhere.
    logging::logger* log();

    // TODO: Get the original drake logger, even if has been replaced by ours?  Or not?
    logging::logger* original_drake_log();
}
#endif  //  OVERRIDE_DRAKE_LOG