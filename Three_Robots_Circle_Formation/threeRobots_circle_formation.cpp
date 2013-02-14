/*********************************************************************************** 
 *	Program to connect three robots, and have them follow a lead robot in a circle
 *	formation.
 *
 *	Created By: Daniel Kulas, Bethune-Cookman University
 *	9/22/12
 ***********************************************************************************/

#include "Aria.h"
#include <cmath>
#include <iostream>

//constant
#define KV  1
#define PI  3.14159

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
	
	/*==================================*/

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

	robot1_radius = robot1.getRobotRadius();
	robot2_radius = robot2.getRobotRadius();
	robot3_radius = robot3.getRobotRadius();

	//run robots on background threads
	robot1.runAsync(true);
	robot2.runAsync(true);
	robot3.runAsync(true);
	printf("Running robots\n");

	for(;;)
	{
		robot1_x = robot1.getX() + robot1_location_x;
		robot1_y = robot1.getY() + robot1_location_y;

		robot2_x = robot2.getX() + robot2_location_x;
		robot2_y = robot2.getY() + robot2_location_y;

		robot3_x = robot3.getX() + robot3_location_x;
		robot3_y = robot3.getY() + robot3_location_y;
	
		robot1_th = robot1.getTh() * PI/180;
		robot2_th = robot2.getTh() * PI/180;
		robot3_th = robot3.getTh() * PI/180;
		
		robot1_x_hat = robot1_x + robot1_radius*cos(robot1_th);
		robot1_y_hat = robot1_y + robot1_radius*sin(robot1_th);

		robot2_x_hat = robot2_x + robot2_radius*cos(robot2_th);
		robot2_y_hat = robot2_y + robot2_radius*sin(robot2_th);

		robot3_x_hat = robot3_x + robot3_radius*cos(robot3_th);
		robot3_y_hat = robot3_y + robot3_radius*sin(robot3_th);


		double t = ArUtil::getTime()/(4000*PI);

		//Control Logic for robot 1
		robot1_control_1 = KV*(robot2_x_hat - robot1_x_hat + robot3_x_hat - robot1_x_hat - 1000*sin(t)) - 2000*sin(t);
		robot1_control_2 = KV*(robot2_y_hat - robot1_y_hat + robot3_y_hat - robot1_y_hat + 1000*cos(t)) + 2000*cos(t);

		robot1_velocity = robot1_control_1 * cos(robot1_th) + robot1_control_2 * sin(robot1_th);
		robot1_rotation_velocity = -((robot1_control_1 * sin(robot1_th)) / robot1_radius) + ((robot1_control_2 * cos(robot1_th)) / robot1_radius); 


		//Control logic for robot 2
		robot2_control_1 = KV*(robot3_x_hat - robot2_x_hat + robot1_x_hat - robot2_x_hat - 1500*sin(t) + 500*cos(t)) - 1000*sin(t) + 1000*cos(t);
		robot2_control_2 = KV*(robot3_y_hat - robot2_y_hat + robot1_y_hat - robot2_y_hat + 1500*cos(t) - 500*sin(t)) + 1000*sin(t) + 1000*cos(t);

		robot2_velocity = robot2_control_1 * cos(robot2_th) + robot2_control_2 * sin(robot2_th);
		robot2_rotation_velocity = -((robot2_control_1 * sin(robot2_th)) / robot2_radius) +  ((robot2_control_2 * cos(robot2_th)) / robot2_radius); 

		//Control Logic for robot 3
		robot3_control_1 = KV*(robot1_x_hat - robot3_x_hat + robot2_x_hat - robot3_x_hat - 500*sin(t) - 500*cos(t)) - 3000*sin(t) + 1000*cos(t);
		robot3_control_2 = KV*(robot1_y_hat - robot3_y_hat + robot2_y_hat - robot3_y_hat + 500*cos(t) - 500*sin(t)) + 3000*cos(t) + 1000*sin(t);

		robot3_velocity = robot3_control_1 * cos(robot3_th) + robot3_control_2 * sin(robot3_th);
		robot3_rotation_velocity = -( (robot3_control_1 * sin(robot3_th)) / robot3_radius) + ((robot3_control_2 * cos(robot3_th)) / robot3_radius); 

		//set velocity and rotational velocity on robots
		robot1.setVel(robot1_velocity);
		robot1.setRotVel(robot1_rotation_velocity);

		robot2.setVel(robot2_velocity);
		robot2.setRotVel(robot2_rotation_velocity);

		robot3.setVel(robot3_velocity);
		robot3.setRotVel(robot3_rotation_velocity);
	}

	Aria::shutdown();
}
