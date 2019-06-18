#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include <robot_msgs/bool_t.hpp>
#include <sys/time.h>

using namespace std;

int64_t get_current_utime() {
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    int64_t current_utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
    return current_utime;
}


int main()
{
	lcm::LCM lcm;
	if(!lcm.good()) return 1;
	while(1){
		robot_msgs::bool_t cmd;
		string a;
		cout << "Enter cmd: ";
		cin >> a;
		cmd.utime = get_current_utime();
		cmd.data = true;
		lcm.publish("FRANKA_0_STOP", &cmd);
		cout << "published STOP";
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