#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include "stop_cmd/stop_cmd.hpp"

using namespace std;

int main()
{
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;
	while(1){
		stop_cmd::stop_cmd cmd;
		string a;
		cout << "Enter cmd: ";
		cin >> a;
		cmd.msg = a;
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