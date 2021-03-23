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

#include <sys/time.h>

#include <iostream>

#include <robot_msgs/bool_t.hpp>
#include <robot_msgs/pause_cmd.hpp>

#include "lcm/lcm-cpp.hpp"

using namespace std;

int64_t get_current_utime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  int64_t current_utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
  return current_utime;
}

int main(int argc, char** argv) {
  std::string channel_name;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot letter>" << std::endl;
    return -1;
  } else {
    channel_name.append("FRANKA_").append(argv[1]).append("_STOP");
  }

  lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=2");
  if (!lcm.good()) return 1;
  while (1) {
    robot_msgs::pause_cmd cmd;
    string a;
    string b;
    cout << "Enter cmd (s=STOP, c=CONTINUE): ";
    cin >> a;
    cout << "Enter source: ";
    cin >> b;
    cmd.utime = get_current_utime();
    if (a == "s") {
      cmd.data = true;
      cmd.source = b;
      cout << "published STOP\n";
    } else if (a == "c") {
      cmd.data = false;
      cmd.source = b;
      cout << "published CONTINUE\n";
    } else {
      continue;
    }
    lcm.publish(channel_name, &cmd);
  }
  return 0;
}

// #include <iostream>
// #include "lcm/lcm-cpp.hpp"

// using namespace std;

// int main()
// {
//   int a,b;
//   char str[] = "Hello Programmers";

//   /* Single insertion operator */
//   cout << "Enter 2 numbers - ";
//   cin >> a >> b;
//   cout << str;
//   cout << endl;

//   /* Multiple insertion operator */
//   cout << "Value of a is " << a << endl << "Value of b is " << b;

//   return 0;
// }
