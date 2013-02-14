/******************************************************************************************************************* 
 *	Program connects three robots (Easy to implement more robots...just create more objects)
 *	User inputs locate of robot 2 and 3 relative to robot 1 (robot 1 needs to have a (x,y) position of (0,0))
 *	
 *  Created by: Daniel Kulas, Bethune-Cookman University
 *	9/18/12
 *******************************************************************************************************************/ 

#include "Aria.h"
#include <cmath>
#include <iostream>
#include <cstring>
#include <fstream>

//constant
#define KV  .2
#define KH  .09
#define pi  3.14159

using namespace std;

int main(int argc, char** argv)
{
	int ret;
	std::string str;
	
	//User needs to specify the location of each robot relative to robot 1.
	//For robot 1, just use the values (0,0) since it is the robot the others will reference
	//each other off of
	int robot1_location_x, robot1_location_y;
	int robot2_location_x, robot2_location_y;
	int robot3_location_x, robot3_location_y;

	cout << "----------------" << endl;
	cout << "Input x and y coordinates of robot 1" << endl;
	cout << "Robot 1 x: ";
	cin >> robot1_location_x;
	cout << "Robot 1 y: ";
	cin >> robot1_location_y;
	cout << "----------------" << endl;
	cout << "Input x and y coordinates of robot 2" << endl;
	cout << "Robot 2 x: ";
	cin >> robot2_location_x;
	cout << "Robot 2 y: ";
	cin >> robot2_location_y;
	cout << "----------------" << endl;
	cout << "Input x and y coordinates of robot 3" << endl;
	cout << "Robot 3 x: ";
	cin >> robot3_location_x;
	cout << "Robot 3 y: ";
	cin >> robot3_location_y;
	cout << "----------------" << endl;

	//get hostnames and port numbers
	ArArgumentParser argParser(&argc, argv);
	char* host1 = argParser.checkParameterArgument("-rh1");
	if(!host1) host1 = "localhost";
	char* host2 = argParser.checkParameterArgument("-rh2");
	if(!host2) host2 = "localhost";
	char* host3 = argParser.checkParameterArgument("-rh3");
	if(!host3) host3 = "localhost";

	int port1 = 8101;
	int port2 = 8101;
	int port3 = 8101;
	
	//if same host, it must be using two ports
	if(strcmp(host1, host2) == 0)	
		port2++;

	if(strcmp(host1, host3) == 0)
		port3 = port3 + 2;

	bool argSet = false;
	argParser.checkParameterArgumentInteger("-rp1", &port1);
	if(!argSet) argParser.checkParameterArgumentInteger("-rrtp1", &port1);
	argSet = false;
	argParser.checkParameterArgumentInteger("-rp2", &port2);
	if(!argSet) argParser.checkParameterArgumentInteger("-rrtp2", &port2);
	argSet = false;
	argParser.checkParameterArgumentInteger("-rp3", &port3);
	if(!argSet) argParser.checkParameterArgumentInteger("-rrtp3", &port3);

	//add the key handler to aria 
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);

	//First robot variables
	ArTcpConnection con1;
	ArRobot robot1;

	//Second robot variables
	ArTcpConnection con2;
	ArRobot robot2;

	//Third robot variables
	ArTcpConnection con3;
	ArRobot robot3; 

	//Attach the key handler to a robot now
	robot1.attachKeyHandler(&keyHandler);
	robot2.attachKeyHandler(&keyHandler);
	robot3.attachKeyHandler(&keyHandler);

	//start up Aria stuff
	Aria::init();

	/*==================================*/
	//Start up robot 1
	ArLog::log(ArLog::Normal, "Connecting to first robot at %s:%d...", host1, port1);
	if ((ret = con1.open(host1, port1)) != 0)
	{
		str = con1.getOpenMessage(ret);
		printf("Open failed to robot 1: %s\n", str.c_str());
		Aria::exit(1);
		return 1;
	}

	robot1.addRangeDevice(&sonar1);
	robot1.setDeviceConnection(&con1);

	if(!robot1.blockingConnect())
	{
		printf("Could not connect to robot 1...abort\n");
		Aria::exit(1);
		return 1;
	}

	//turn on motors, turn off sounds
	robot1.comInt(ArCommands::ENABLE, 1);
	robot1.comInt(ArCommands::SOUNDTOG, 0);
	

	/*==================================*/
	//Start up robot 2

	ArLog::log(ArLog::Normal, "Connecting to second robot");
	if ((ret = con2.open(host2, port2)) != 0)
	{
		str = con2.getOpenMessage(ret);
		printf("Open failed to robot 2: %s\n", str.c_str());
		Aria::exit(1);
		return 1;
	}

	robot2.addRangeDevice(&sonar2);
	robot2.setDeviceConnection(&con2);

	if(!robot2.blockingConnect())
	{
		printf("Could not connect to robot 2...abort\n");
		Aria::exit(1);
		return 1;
	}
	printf("Turning on motors for robot 2\n");
	//turn on motors, turn off sound
	robot2.comInt(ArCommands::ENABLE, 1);
	robot2.comInt(ArCommands::SOUNDTOG, 0);

/*==================================*/
	//Start up robot 3

	ArLog::log(ArLog::Normal, "Connecting to second robot");
	if ((ret = con3.open(host3, port3)) != 0)
	{
		str = con3.getOpenMessage(ret);
		printf("Open failed to robot 3: %s\n", str.c_str());
		Aria::exit(1);
		return 1;
	}

	robot3.addRangeDevice(&sonar3);
	robot3.setDeviceConnection(&con3);

	if(!robot3.blockingConnect())
	{
		printf("Could not connect to robot 3...abort\n");
		Aria::exit(1);
		return 1;
	}
	printf("Turning on motors for robot 3\n");
	//turn on motors, turn off sound
	robot3.comInt(ArCommands::ENABLE, 1);
	robot3.comInt(ArCommands::SOUNDTOG, 0);

	//setup vars
	double robot1_x, robot1_y;	// x/y position
	double robot2_x, robot2_y;
	double robot3_x, robot3_y;

	double robot1_th, robot2_th, robot3_th; // angular orientation
	double robot1_radius, robot2_radius, robot3_radius;	//robots radius

	double robot1_x_hat, robot1_y_hat;	//parameters
	double robot2_x_hat, robot2_y_hat;
	double robot3_x_hat, robot3_y_hat;
	
	double robot1_control_1, robot1_control_2;
	double robot2_control_1, robot2_control_2;
	double robot3_control_1, robot3_control_2;

	double robot1_velocity, robot1_rotation_velocity;
	double robot2_velocity, robot2_rotation_velocity;
	double robot3_velocity, robot3_rotation_velocity;

	printf("Finished setting up shit\n");
	printf("Getting robots radius\n");

	robot1_radius = robot1.getRobotRadius();
	robot2_radius = robot2.getRobotRadius();
	robot3_radius = robot3.getRobotRadius();

	printf("Aight bruh-man, got the radius\n");

	robot1.runAsync(true);
	robot2.runAsync(true);
	robot3.runAsync(true);
	printf("Running robots\n");
	//start robots
	while(1)
	{
		std::cout << "INSIDE OF LOOP" << std::endl;
		//get x/y coordinates from robot so the other can follow	
		robot1_x = robot1.getX() + robot1_location_x;
		robot1_y = robot1.getY() + robot1_location_y;

		robot2_x = robot2.getX() + robot2_location_x;
		robot2_y = robot2.getY() + robot2_location_y;

		robot3_x = robot3.getX() + robot3_location_x;
		robot3_y = robot3.getY() + robot3_location_y;
	
		robot1_th = robot1.getTh();
		robot2_th = robot2.getTh();
		robot3_th = robot3.getTh();
		
		//control logic for robot1
		robot1_x_hat = robot1_x + robot1_radius*cos(robot1_th);
		robot1_y_hat = robot1_y + robot1_radius*sin(robot1_th);

		robot1_control_1 = KV*(robot2_x - robot1_x);
		robot1_control_2 = KV*(robot2_y - robot1_y);

		robot1_velocity = robot1_control_1*cos(robot1_th) + (robot1_control_2/robot1_radius)*sin(robot1_th);
		robot1_rotation_velocity = -(robot1_control_1/robot1_radius)*sin(robot1_th) + robot1_control_2*cos(robot1_th);

		//control logic for robot2
		robot2_x_hat = robot2_x + robot2_radius*cos(robot2_th);
		robot2_y_hat = robot2_y + robot2_radius*sin(robot2_th);

		robot2_control_1 = KV*(robot3_x - robot2_x);
		robot2_control_2 = KV*(robot3_y - robot2_y);

		robot2_velocity = robot2_control_1*cos(robot2_th) + (robot2_control_2/robot2_radius)*sin(robot2_th);
		robot2_rotation_velocity = -(robot2_control_2/robot2_radius)*sin(robot2_th) + robot2_control_2*cos(robot2_th);

		//control logic for robot3
		robot3_x_hat = robot3_x + robot3_radius*cos(robot3_th);
		robot3_y_hat = robot3_y + robot3_radius*sin(robot3_th);

		robot3_control_1 = KV*(robot1_x - robot3_x);
		robot3_control_2 = KV*(robot1_y - robot3_y);

		robot3_velocity = robot3_control_1*cos(robot3_th) + (robot3_control_2/robot3_radius)*sin(robot3_th);
		robot3_rotation_velocity = -(robot3_control_2/robot3_radius)*sin(robot3_th) + robot3_control_2*cos(robot3_th);		

		//set velocity on robot and angular orientation
		std::cout << "Robot 1 velocity: " << robot1_velocity << std::endl;
		std::cout << "Robot 1 rotation velocity: " << robot1_rotation_velocity << std::endl;
		robot1.setVel(robot1_velocity);
		robot1.setDeltaHeading(robot1_rotation_velocity);

		std::cout << "Robot 2 velocity: " << robot2_velocity << std::endl;
		std::cout << "Robot 2 rotation velocity: " << robot2_rotation_velocity << std::endl;
		robot2.setVel(robot2_velocity);
		robot2.setDeltaHeading(robot2_rotation_velocity);

		std::cout << "Robot 3 velocity: " << robot3_velocity << std::endl;
		std::cout << "Robot 3 rotation velocity: " << robot3_rotation_velocity << std::endl;
		robot3.setVel(robot3_velocity);
		robot3.setDeltaHeading(robot3_rotation_velocity);
	}

	Aria::shutdown();
}
