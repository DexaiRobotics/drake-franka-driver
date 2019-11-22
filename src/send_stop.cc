#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include <robot_msgs/bool_t.hpp>
#include <robot_msgs/pause_cmd.hpp>
#include <sys/time.h>
#include "drac_util_io.h"            // for get_current_utime

using namespace std;

int main(int argc, char** argv)
{	
	std::string channel_name;
	if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot letter>" << std::endl;
        return -1;
    }
    else {
        channel_name.append("FRANKA_").append(argv[1]).append("_STOP");
    }

	lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=2");
	if(!lcm.good()) return 1;
	while(1){
		robot_msgs::pause_cmd cmd;
		string a;
		string b;
		cout << "Enter cmd (s=STOP, c=CONTINUE): ";
		cin >> a;
		cout << "Enter source: ";
		cin >> b;
		cmd.utime = dru::get_current_utime();
		if(a == "s"){
			cmd.data = true;
			cmd.source = b;
			cout << "published STOP\n";
		} else if(a == "c"){
			cmd.data = false;
			cmd.source = b;
			cout << "published CONTINUE\n";
		}
		else{
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
// 	int a,b;
// 	char str[] = "Hello Programmers";
	
// 	/* Single insertion operator */
// 	cout << "Enter 2 numbers - ";
// 	cin >> a >> b;
// 	cout << str;
// 	cout << endl;
	
// 	/* Multiple insertion operator */
// 	cout << "Value of a is " << a << endl << "Value of b is " << b;
	
// 	return 0;
// }
