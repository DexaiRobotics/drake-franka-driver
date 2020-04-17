// @file  momap/log_momap.cc
#include "momap_log.h"
#include "utils.h"
#include <iostream>

namespace {
  // static drake::never_destroyed<std::shared_ptr<slogger>> s_logger = nullptr;
  static std::shared_ptr<slogger> s_logger = nullptr;
  static slogger *d_logger = nullptr;
}

unsigned MomapLogger::log_id_size(spdlog::level::level_enum ll_enum)
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


namespace momap {

    // Returns true IFF the logger object is NULL,
    // meaning that either is has not yet been created, or it was nullified.
    bool is_log_null() { return s_logger == nullptr; }

    spdlog::logger* log(int warn_if_null)
    {
        if (is_log_null()) {
            if (warn_if_null) {
                std::cerr << "WARNING: Call to uninitialized momap::logging::logger log()" << std::endl;
                std::cerr << "WARNING: Using all defaults in momap::logging::logger log()" << std::endl;
            }
            create_log();
        }
        assert(s_logger && "FATAL: s_logger still not set after calling momap::create_log()");
        return s_logger.get();
    }

    /// Convenience function to get the "current" logger's log directory.  @see log_momap.h
    std::string log_dir()
    {
        try {
            MomapLogger *momap_logger = dynamic_cast<MomapLogger *>(momap::log());
            return momap_logger == nullptr ? "" : momap_logger->log_dir();
        } catch(const std::exception& ex) {
            momap::log()->warn("Error in momap::log_dir: {}", ex.what());
        }
        return "";
    }

    // Convenience function to get the "current" logger's log message ID size.  @see log_momap.h
    // Example usage:
    // log()->error("This is a test of momap::MomapLogger::log_id_size:\n{:>{}}"
    //              "This line shall line up with the line up one line.",
    //              "", momap::log_id_size()
    // );
    unsigned log_id_size(spdlog::level::level_enum ll_enum)
    {
        try {
            MomapLogger *momap_logger = dynamic_cast<MomapLogger *>(momap::log(ll_enum));
            return momap_logger == nullptr ? 42 : momap_logger->log_id_size();
        } catch(const std::exception& ex) {
            momap::log()->debug("Error in momap::log_id_size({}): {}", (unsigned)ll_enum, ex.what());
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
            s_logger->warn("momap::create_log: s_logger already created!");
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
        // NOTE: We cannot call drake::log or momap::log here; they're uninitialized.
        // TODO: avoid paths like "here/./now" ??
        // std::cerr << "::::::::::: sub_dir_path(log_dir, " << program << ") gave Log Base Dir (LBD): " << log_dir << std::endl;
        if (got_base) {
            // NOTE: Caution: Calling log here may stackoverflow.
        } else {
            drake::log()->error("::::::::::: momap::create_log: fatal error -- no base directory for logs!");
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

        // NOTE: We don't necessarily need a MomapLogger; we could just use one spd::logger.
        std::shared_ptr<MomapLogger> logger = std::make_shared<MomapLogger>(prefix, log_dir, begin(sinks), end(sinks));
        spdlog::register_logger(logger);   //register it if you need to access it globally
        logger->set_level(spdlog::level::info);
        s_logger = logger;
        return true;
    }

}   // namespace momap


#if OVERRIDE_DRAKE_LOG

namespace drake {
    logging::logger* log()
    {
        #ifdef ASSERT_TRUE  // To be compiled only in a test context.
        std::cerr << "\t<<<< Call to drake::log() got the override log() >>>>" << std::endl << std::endl;
        #endif

        // TODO: Would prefer to assert rather than conpensate for failing
        // to initialize the logger, but then every executable that includes
        // log_momap.h with OVERRIDE_DRAKE_LOG defined as truthy must either call
        // momap::create_log or work around it.
        // assert(s_logger && "s_logger not set; call momap::create_log() first!");
        if (momap::is_log_null()) {
            std::cerr << "WARNING: Call to uninitialized drake::logging::logger log()" << std::endl;
            momap::create_log();
            std::cerr << "WARNING: Used all defaults for drake::logging::logger log()" << std::endl;
        }
        assert(s_logger && "FATAL: s_logger still not set after calling momap::create_log()");
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
