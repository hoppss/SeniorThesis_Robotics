/********************************************************
 *	Program creates a map and writes data out to
 *	a .dat file to be used later in MATLAB
 *
 *	Makes use of the Boost libraries
 *	
 *	Created By: Daniel Kulas
 *				10/12/12
 ********************************************************/

#include "Aria.h"
#include <opencv\cv.h>
#include <opencv\highgui.h>

// Refer BOOST's site for more info on the multi_array: http://www.boost.org/doc/libs/1_52_0/libs/multi_array/doc/reference.html
#include <boost/multi_array.hpp>

#include <iostream>
#include <cstring>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <algorithm>
#include <stack>
#include <cmath>

#define PI	3.14159
#define KR	0.1 		//KR = constant

using namespace std;

typedef boost::multi_array<int, 2> array_type;	
typedef array_type::index index;

void msg(char* str)
{
	cout << str << endl;
}

/*	
 *	The fuzzy_map function will handle uncertainties in the sonar.
 *	Function needs the sonar device, the value returned by the sonar, the distance threshold of the users choosing
 *	that will deteremine some offset value due to the sonar, the robots theta orientation (should already be in degrees),
 *	robots x position, and the robots y position.
 *
 *	The "cell_x" variables represent the 9 cells that the function will map values to
 *	_____________
 *	|	|	|	|
 *	|C2_|C5_|C8_|
 *	|	|	|	|
 *	|C1_|C4_|C7_|
 *	|	|	|	|
 *	|C3_|C6_|C9_|
 *
 *	Refer to thesis for more information on this implementation
 */

void fuzzy_map_obstacles( ArRobot* robot, array_type &map, double sonarReading, int sonarIndex, double distanceThreshold, 
						  double robotTh, double robot_x_pos, double robot_y_pos, 
						  std::stack<double>* fuzzy_obstacle_stack_x, std::stack<double>* fuzzy_obstacle_stack_y )
{
	double cell_1x, cell_1y, cell_2x, cell_2y, cell_3x, cell_3y,
		   cell_4x, cell_4y, cell_5x, cell_5y, cell_6x, cell_6y,
		   cell_7x, cell_7y, cell_8x, cell_8y, cell_9x, cell_9y;

	sonarReading = robot->getSonarReading(sonarIndex)->getRange();

	cell_1x = floor( (robot_x_pos + (sonarReading - distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	cell_1y = floor( (robot_y_pos + (sonarReading - distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	//place values onto the stack for faster traversing through the 2D vector
	fuzzy_obstacle_stack_x->push(cell_1x);
	fuzzy_obstacle_stack_y->push(cell_1y);
	//write value at that map index location
	map[cell_1x][cell_1y] = 0.8;

	cell_2x = floor( (robot_x_pos + (sonarReading - distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	cell_2y = floor( (robot_y_pos + (sonarReading - distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	map[cell_2x][cell_2y] = 0.8 * cos( (12/180) * PI);
	fuzzy_obstacle_stack_x->push(cell_2x);
	fuzzy_obstacle_stack_y->push(cell_2y);


	cell_3x = floor( (robot_x_pos + (sonarReading - distanceThreshold) * cos( (	(robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	cell_3y = floor( (robot_y_pos + (sonarReading - distanceThreshold) * sin( (	(robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	fuzzy_obstacle_stack_x->push(cell_3x);
	fuzzy_obstacle_stack_y->push(cell_3y);
	map[cell_3x][cell_3y] = 0.8 * cos( (12/180) * PI);
	
	cell_4x = floor( (robot_x_pos + sonarReading * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	cell_4y = floor( (robot_y_pos + sonarReading * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	fuzzy_obstacle_stack_x->push(cell_4x);
	fuzzy_obstacle_stack_y->push(cell_4y);
	map[cell_4x][cell_4y] = 1;

	cell_5x = floor( (robot_x_pos + sonarReading * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	cell_5y = floor( (robot_y_pos + sonarReading * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	fuzzy_obstacle_stack_x->push(cell_5x);
	fuzzy_obstacle_stack_y->push(cell_5y);
	map[cell_5x][cell_5y] = 0.8 * cos( (12/180) * PI);

	cell_6x = floor( (robot_x_pos + sonarReading * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	cell_6y = floor( (robot_y_pos + sonarReading * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	fuzzy_obstacle_stack_x->push(cell_6x);
	fuzzy_obstacle_stack_y->push(cell_6y);
	map[cell_6x][cell_6y] = 0.8 * cos( (12/180) * PI);

	cell_7x = floor( (robot_x_pos + (sonarReading + distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	cell_7y = floor( (robot_y_pos + (sonarReading + distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	fuzzy_obstacle_stack_x->push(cell_7x);
	fuzzy_obstacle_stack_y->push(cell_7y);
	map[cell_7x][cell_7y] = 0.8;

	cell_8x = floor( (robot_x_pos + (sonarReading + distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	cell_8y = floor( (robot_y_pos + (sonarReading + distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	fuzzy_obstacle_stack_x->push(cell_8x);
	fuzzy_obstacle_stack_y->push(cell_8y);
	map[cell_8x][cell_8y] = 0.8 * cos( (12/180) * PI);

	cell_9x = floor( (robot_x_pos + (sonarReading + distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	cell_9y = floor( (robot_y_pos + (sonarReading + distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	fuzzy_obstacle_stack_x->push(cell_9x);
	fuzzy_obstacle_stack_y->push(cell_9y);
	map[cell_9x][cell_9y] = 0.8 * cos( (12/180) * PI);
}

void fuzzy_map_empty( ArRobot* robot, array_type &map, double sonarReading, int sonarIndex, double distanceThreshold, 
					  double robotTh, double robot_x_pos, double robot_y_pos,
					  std::stack<double>* fuzzy_empty_stack_x, std::stack<double>* fuzzy_empty_stack_y )
{
	double cell_1x, cell_1y, cell_2x, cell_2y, cell_3x, cell_3y,
		   cell_4x, cell_4y, cell_5x, cell_5y, cell_6x, cell_6y,
		   cell_7x, cell_7y, cell_8x, cell_8y, cell_9x, cell_9y;

	sonarReading = robot->getSonarReading(sonarIndex)->getRange();
	
	cell_1x = floor( (robot_x_pos + (sonarReading - distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	cell_1y = floor( (robot_y_pos + (sonarReading - distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_1x);
	fuzzy_empty_stack_y->push(cell_1y);
	map[cell_1x][cell_1y] = 0.5;

	cell_2x = floor( (robot_x_pos + (sonarReading - distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	cell_2y = floor( (robot_y_pos + (sonarReading - distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_2x);
	fuzzy_empty_stack_y->push(cell_2y);
	map[cell_2x][cell_2y] = 0.5 * cos( (12/180) * PI);
	
	cell_3x = floor( (robot_x_pos + (sonarReading - distanceThreshold) * cos( (	(robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	cell_3y = floor( (robot_y_pos + (sonarReading - distanceThreshold) * sin( (	(robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_3x);
	fuzzy_empty_stack_y->push(cell_3y);
	map[cell_3x][cell_3y] = 0.5 * cos( (12/180) * PI);

	cell_4x = floor( (robot_x_pos + sonarReading * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	cell_4y = floor( (robot_y_pos + sonarReading * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_4x);
	fuzzy_empty_stack_y->push(cell_4y);
	map[cell_4x][cell_4y] = 0;

	cell_5x = floor( (robot_x_pos + sonarReading * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	cell_5y = floor( (robot_y_pos + sonarReading * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_5x);
	fuzzy_empty_stack_y->push(cell_5y);
	map[cell_5x][cell_5y] = 0;

	cell_6x = floor( (robot_x_pos + sonarReading * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	cell_6y = floor( (robot_y_pos + sonarReading * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	map[cell_6x][cell_6y] = 0;
	fuzzy_empty_stack_x->push(cell_6x);
	fuzzy_empty_stack_y->push(cell_6y);

	cell_7x = floor( (robot_x_pos + (sonarReading + distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	cell_7y = floor( (robot_y_pos + (sonarReading + distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh()) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_7x);
	fuzzy_empty_stack_y->push(cell_7y);
	map[cell_7x][cell_7y] = 1;

	cell_8x = floor( (robot_x_pos + (sonarReading + distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	cell_8y = floor( (robot_y_pos + (sonarReading + distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() + 12) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_8x);
	fuzzy_empty_stack_y->push(cell_8y);
	map[cell_8x][cell_8y] = 1;

	cell_9x = floor( (robot_x_pos + (sonarReading + distanceThreshold) * cos( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	cell_9y = floor( (robot_y_pos + (sonarReading + distanceThreshold) * sin( ( (robot->getSonarReading(sonarIndex)->getSensorTh() + robot->getTh() - 12) / 180) * PI )) / 100) + 1;
	fuzzy_empty_stack_x->push(cell_9x);
	fuzzy_empty_stack_y->push(cell_9y);
	map[cell_9x][cell_9y] = 1;
}

int main(int argc, char** argv)
{
	//We will be writing data to a file, so create a file
	ofstream output("robot_map.dat");

	/*
	 * stacks are used so when we update our map, we don't have to trace through the entire 2D vector to 
	 * and deal with the slow computation time due to the robots crappy hardware
	 */ 

	std::stack<double> fuzzy_obstacle_stack_x;
	std::stack<double> fuzzy_obstacle_stack_y;
	double update_obstacle_x = NULL;
	double update_obstacle_y = NULL;

	std::stack<double> fuzzy_empty_stack_x;
	std::stack<double> fuzzy_empty_stack_y;
	double update_empty_x = NULL;
	double update_empty_y = NULL;
	
	/* 
	 *	Obstacle map maps obstacles and uses fuzzy logic to do some things
	 *	Empty map maps empty regions and uses fuzzy logic to do some things
	 *	Global map is updated at the very end of the program 
	 */

	array_type robot_map_obstacles(boost::extents[160][160]);
	array_type robot_map_obstacles_new(boost::extents[160][160]);

	array_type robot_map_empty(boost::extents[160][160]);
	array_type robot_map_empty_new(boost::extents[160][160]);

	array_type robot_map_global(boost::extents[160][160]);

	//set initial values of matrix 
	for(index i = 0; i != 160; ++i)
	{
		for(index j = 0; j != 160; ++j)
		{
			robot_map_obstacles[i][j] = 0;
			robot_map_obstacles_new[i][j] = 0;
			
			robot_map_empty[i][j] = 1;		//empty map represents 1 as empty spot. 
			robot_map_empty_new[i][j] = 1;

			robot_map_global[i][j] = 0;
		}
	}

	//Have user specify where the robot is initially located in relation to the map
	int init_x, init_y;

	cout << "x position: ";
	cin >> init_x;

	cout << "\n y position: ";
	cin >> init_y;
	
	//Start up Robot stuff
	Aria::init();

	ArRobot robot;
	ArKeyHandler keyHandler;
	ArSonarDevice sonar;

	ArSimpleConnector connector(&argc, argv);
	connector.parseArgs();

	if(argc > 1)
	{
		connector.logOptions();
		exit(1);
	}

	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	robot.addRangeDevice(&sonar);

	if(!connector.connectRobot(&robot))
	{
		ArLog::log(ArLog::Terse, "Could not connect to robot");
		Aria::shutdown();
		Aria::exit(1);
	}

	ArLog::log(ArLog::Normal, "Connected to robot");

	int robot_x = robot.getX();
	int robot_y = robot.getY();

	robot.comInt(ArCommands::ENABLE, 1);
	robot.comInt(ArCommands::SOUNDTOG, 0);
	
	//init front sonar vars for obstacle avoidance 
	double left_sonar_0;
	double front_left_sonar_1, front_left_sonar_2;
	double front_sonar_1, front_sonar_2;
	double front_right_sonar_1, front_right_sonar_2;
	double right_sonar_7;

	double sonar_range;

	//feedback velocity and rotation velocity
	double vel;
	double robot_th;

	robot.runAsync(true);
	msg("Robot connected successfully...continuing");

	// I'm ashamded on this "counter" that ends the program after a set number of cycles :(
	int x = 300000;
	while(x != 0)
	{
		robot_x  = robot.getX() + init_x;
		robot_y  = robot.getY() + init_y;
		robot_th = robot.getTh();

		//get front sonar values for obstacle avoidance
		left_sonar_0		=	robot.getSonarReading(0)->getRange();
		front_left_sonar_1	=	robot.getSonarReading(1)->getRange();
		front_left_sonar_2	=	robot.getSonarReading(2)->getRange();
		front_sonar_1		=	robot.getSonarReading(3)->getRange();
		front_sonar_2		=	robot.getSonarReading(4)->getRange();
		front_right_sonar_1 =	robot.getSonarReading(5)->getRange();
		front_right_sonar_2 =	robot.getSonarReading(6)->getRange();
		right_sonar_7		=	robot.getSonarReading(7)->getRange();

		//trace through all 16 sonars and update the map if required
		for(int i = 0; i < 16; i++)
		{
			sonar_range = robot.getSonarReading(i)->getRange();
			
			if(sonar_range < 4500 && sonar_range > 200)
			{
				fuzzy_map_obstacles( &robot, robot_map_obstacles, sonar_range, i, 10.0, 
									 robot_th, robot_x, robot_y, 
									 &fuzzy_obstacle_stack_x, &fuzzy_obstacle_stack_y );

				fuzzy_map_empty	   ( &robot, robot_map_empty, sonar_range, i, 10.0, 
									 robot_th, robot_x, robot_y, 
									 &fuzzy_empty_stack_x, &fuzzy_empty_stack_y );
			}
		}

		//update obstacle map by popping off values from the (x,y) stacks for obstacles
		while(!fuzzy_obstacle_stack_x.empty() && !fuzzy_obstacle_stack_x.empty())
		{
			//grab the coordinates at which the map got updated
			update_obstacle_x = fuzzy_obstacle_stack_x.top();
			update_obstacle_y = fuzzy_obstacle_stack_y.top();
			//pop to remove the value...we don't need it anymore after we use it
			fuzzy_obstacle_stack_x.pop();
			fuzzy_obstacle_stack_y.pop();
			//based on those coordinates, update the map at only at THAT location
			robot_map_obstacles_new[ update_obstacle_x ][ update_obstacle_y ] |= robot_map_obstacles[ update_obstacle_x ][ update_obstacle_y ];
		}

		//update empty map by popping off values from the (x,y) stacks for obstacles
		while(!fuzzy_empty_stack_x.empty() && !fuzzy_empty_stack_y.empty())
		{
			update_obstacle_x = fuzzy_empty_stack_x.top();
			update_obstacle_y = fuzzy_empty_stack_y.top();
			fuzzy_empty_stack_x.pop();
			fuzzy_empty_stack_y.pop();

			robot_map_empty_new[ update_obstacle_x ][ update_obstacle_y ] &= robot_map_empty[ update_obstacle_x ][ update_obstacle_y ];
		}

		//now avoid obstacles and things
		if(front_sonar_1 < 800 || front_sonar_2 < 800)
		{
			//cout << "Front blocked. Checking left and rightt" << endl;
			if(front_left_sonar_1 < 800 || front_left_sonar_2 < 800 || left_sonar_0 < 800)
			{
				//cout << "Left blocked. Turning right" << endl;
				//based on how close the robot is to the obstacle, it will gradually speed up if far away, or slow down if closer
				vel = (min(front_left_sonar_1, front_left_sonar_2) - 200) / 6;
				robot.setVel(vel);
				robot.setDeltaHeading(-20);
				//cout << "Robot angle: " << robot.getTh() << endl;
			}
			else if(front_right_sonar_1 < 800 || front_right_sonar_2 < 800 || right_sonar_7 < 800)
			{
			//	cout << "Right blocked. Turning left" << endl;
				vel = (min(front_right_sonar_1, front_right_sonar_2) - 200) / 6;
				robot.setVel(vel);
				robot.setDeltaHeading(20);
				//cout << "Robot angle: " << robot.getTh() << endl;
			}
			else
			{
				robot.setVel(-100);
				robot.setDeltaHeading(20);
				//cout << "Robot angle: " << robot.getTh() << endl;
			}
		}
		else if(front_left_sonar_1 < 800 || front_left_sonar_2 < 800 || left_sonar_0 < 800)
		{
			//cout << "Approaching obstacle on the left. Turning Right" << endl;
			vel = (min(front_left_sonar_1, front_left_sonar_2) - 200)/6;
			robot.setVel(vel);
			robot.setDeltaHeading(-20);
			//cout << "Robot angle: " << robot.getTh() << endl;
		}
		else if(front_right_sonar_1 < 800 || front_right_sonar_2 < 800 || right_sonar_7 < 800)
		{
			//cout << "Approaching obstacle on the right. Turning Left." << endl;
			vel = (min(front_right_sonar_1, front_right_sonar_2) - 200)/6;
			robot.setVel(vel);
			robot.setDeltaHeading(20);
			//cout << "Robot angle: " << robot.getTh() << endl;
		}
		else
		{
			robot.setVel(300);
			robot.setDeltaHeading(0);
		}
		
		x--;
		cout << "X = " << x << endl;
	}

	//stop robot, once it breaks out of the loop, it would keep the velocity it currently had without this
	robot.setVel(0);
	robot.setRotVel(0);

	/*
	 * AND together the obstacle map and the empty map and set it equal to the global map
	 * Nested loop to ensure that the WHOLE map gets updated
	 */
	for(index i = 0; i != 160; ++i)
		for(index j = 0; j != 160; ++j)
			robot_map_global[i][j] = robot_map_obstacles_new[i][j] & ~(robot_map_empty_new[i][j]);
	
	//write global map to file
	cout << "Writing .dat file " << endl;
	for(index i = 0; i != 160; ++i)
	{
		for(index j = 0; j != 160; ++j)
			output << setw(4) << robot_map_global[i][j] << " ";
		output << "\n";
	}	

	cout << "Writing complete. " << endl;

	//close file
	output.close();
	Aria::shutdown();
	Aria::exit(0);


	
	return 0;
}