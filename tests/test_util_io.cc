/// @file: test_make_dirs.cc -- part of a googletest suite
#include <gtest/gtest.h>

#include "util_io.h"
#include "util_math.h"
// #include "momap/file_formats.h"

#include <limits.h>

#include <chrono>
#include <thread>

using std::cerr;
using std::cout;
using std::endl;
using std::string;

TEST(MakeDirs, TempDirs) {
  time_t now = time(NULL);
  std::string temp_dirs = "../build/temp/test/dir/" + std::to_string(now);
  std::cerr << "temp_dirs: " << temp_dirs << std::endl;
  int dirs_created = -99;
  EXPECT_NO_THROW(dirs_created = utils::make_dirs(temp_dirs));
  EXPECT_EQ(dirs_created, 0);
  utils::expect_file_mod(temp_dirs, 10);
}

TEST(utime, get_current_utime) {
  auto utime0 = utils::get_current_utime();
  std::this_thread::sleep_for(std::chrono::microseconds(1));
  auto utime1 = utils::get_current_utime();
  EXPECT_TRUE(utime1 >= (utime0 + 1));  // this either takes 1 usec or longer
  std::cout << "utime1: " << utime1 << " vs utime0 " << utime0 << std::endl;
  std::cout << "delta: " << utime1 - utime0 << std::endl;
}
