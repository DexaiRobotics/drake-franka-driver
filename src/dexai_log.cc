// @file  dexai/log_dexai.cc
#include "dexai_log.h"
#include "utils.h"
#include <iostream>

namespace {
  // static drake::never_destroyed<std::shared_ptr<slogger>> s_logger = nullptr;
  static std::shared_ptr<slogger> s_logger = nullptr;
  static slogger *d_logger = nullptr;
}

unsigned DexaiLogger::log_id_size(spdlog::level::level_enum ll_enum)
{
    switch (ll_enum) {
        case spdlog::level::warn:
            return log_id_error_size_ + 2;
        case spdlog::level::info:
            return log_id_error_size_ - 1;
        // case spdlog::level::debug:
        // case spdlog::level::err:         // NOTE spdlog::level::error does not exist.
        // case spdlog::level::trace:
        default:
            return log_id_error_size_;
    }
}


namespace dexai {

    // Returns true IFF the logger object is NULL,
    // meaning that either is has not yet been created, or it was nullified.
    bool is_log_null() { return s_logger == nullptr; }

    spdlog::logger* log(int warn_if_null)
    {
        if (is_log_null()) {
            if (warn_if_null) {
                std::cerr << "WARNING: Call to uninitialized dexai::logging::logger log()" << std::endl;
                std::cerr << "WARNING: Using all defaults in dexai::logging::logger log()" << std::endl;
            }
            create_log();
        }
        assert(s_logger && "FATAL: s_logger still not set after calling dexai::create_log()");
        return s_logger.get();
    }

    /// Convenience function to get the "current" logger's log directory.  @see log_dexai.h
    std::string log_dir()
    {
        try {
            DexaiLogger *dexai_logger = dynamic_cast<DexaiLogger *>(dexai::log());
            return dexai_logger == nullptr ? "" : dexai_logger->log_dir();
        } catch(const std::exception& ex) {
            dexai::log()->warn("Error in dexai::log_dir: {}", ex.what());
        }
        return "";
    }

    // Convenience function to get the "current" logger's log message ID size.  @see log_dexai.h
    // Example usage:
    // log()->error("This is a test of dexai::DexaiLogger::log_id_size:\n{:>{}}"
    //              "This line shall line up with the line up one line.",
    //              "", dexai::log_id_size()
    // );
    unsigned log_id_size(spdlog::level::level_enum ll_enum)
    {
        try {
            DexaiLogger *dexai_logger = dynamic_cast<DexaiLogger *>(dexai::log(ll_enum));
            return dexai_logger == nullptr ? 42 : dexai_logger->log_id_size();
        } catch(const std::exception& ex) {
            dexai::log()->debug("Error in dexai::log_id_size({}): {}", (unsigned)ll_enum, ex.what());
        }
        return 42;
    }


    bool create_log( const std::string& program_in
                   , const std::string& base_path
                   , const std::string& prefix_in
                   , bool use_stdout
                   , bool create_once
    ) {
        if (s_logger != nullptr && create_once) {
            s_logger->warn("dexai::create_log: s_logger already created!");
            return false;
        }
        std::string program = program_in;
        if (program.empty()) {
            program = "dexai";
        }
        std::string base_dir = base_path;
        if (base_dir.empty()) {
            base_dir = utils::get_home_dir() + "/log_robot";
        }
        std::string prefix = prefix_in;
        if (prefix.empty()) {
            prefix = program;
        }
        std::string log_dir;
        bool got_base = utils::sub_dir_path(log_dir, base_dir, program);
        //$ append extra date and datetime subfolders to log output path
        std::string date_string = utils::get_date_string();
        std::string date_time_string = utils::date_time_string();
        got_base = utils::sub_dir_path(log_dir, log_dir, date_string);
        got_base = utils::sub_dir_path(log_dir, log_dir, date_time_string);
        // NOTE: We cannot call drake::log or dexai::log here; they're uninitialized.
        if (got_base) {
            // NOTE: Caution: Calling log here may stackoverflow.
        } else {
            drake::log()->error("::::::::::: dexai::create_log: fatal error -- no base directory for logs!");
            return false;
        }

        std::vector<spdlog::sink_ptr> sinks;
        if (use_stdout) {
            sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        } else {
            sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());
        }
        std::string log_file_name = utils::date_time_string() + "_" + program + ".log";
        std::string log_file_path = log_dir + "/" + log_file_name;
        sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_path));

        // NOTE: We don't necessarily need a DexaiLogger; we could just use one spd::logger.
        std::shared_ptr<DexaiLogger> logger = std::make_shared<DexaiLogger>(prefix, log_dir, begin(sinks), end(sinks));
        spdlog::register_logger(logger);   //register it if you need to access it globally
        logger->set_level(spdlog::level::info);
        s_logger = logger;
        return true;
    }

}   // namespace dexai


#if OVERRIDE_DRAKE_LOG

namespace drake {
    logging::logger* log()
    {
        #ifdef ASSERT_TRUE  // To be compiled only in a test context.
        std::cerr << "\t<<<< Call to drake::log() got the override log() >>>>" << std::endl << std::endl;
        #endif

        // TODO: Would prefer to assert rather than conpensate for failing
        // to initialize the logger, but then every executable that includes
        // log_dexai.h with OVERRIDE_DRAKE_LOG defined as truthy must either call
        // dexai::create_log or work around it.
        // assert(s_logger && "s_logger not set; call dexai::create_log() first!");
        if (dexai::is_log_null()) {
            std::cerr << "WARNING: Call to uninitialized drake::logging::logger log()" << std::endl;
            dexai::create_log();
            std::cerr << "WARNING: Used all defaults for drake::logging::logger log()" << std::endl;
        }
        assert(s_logger && "FATAL: s_logger still not set after calling dexai::create_log()");
        return s_logger.get();
    }
}   // namespace drake


//  TODO: If possible and worthwhile, provide access to the original result of
//  drake::log() that we overrode.
drake::logging::logger* original_drake_log()
{
    if (d_logger == nullptr) {
        d_logger = drake::log();
    }
    return d_logger;
}

#endif  //  OVERRIDE_DRAKE_LOG
