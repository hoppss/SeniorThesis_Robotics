Before you can even think about running these programs you are going to need some supplementary software and/or hardware.

Hardware required:  
					MobileRobots Amigo robot
		  			MobileRobots P3-AT robot
		  			Point Grey's Bumblebee2 stereo camera

Sofware required:
					OpenCV
					Boost (for the aria_robot_mapping)
					Visual Studio 2010 (should work on 2008 or higher but I never tested it)
					Aria SDK (See: http://robots.mobilerobots.com/wiki/ARIA)
					FlyCapture2 SDK  (See: http://www.ptgrey.com/products/pgrflycapture/flycapture_camera_software.asp)

Optional:			
					MATLAB 
					MobileSim

If you don't have the hardware, you can download MobileSim to simulate the programs that don't require the camera.

--------------------------------------------------

Assuming you have at least all the software up and running, you need to set the includes and libs in visual studio.
Here is what I recommend how to set this all up:


	Move the folders I provided in the repo to C:\Program Files\MobileRobots\Aria\
	You can either put them in the folder "examples" or make your own folder...doesn't matter
	
	In that same directory, open the Aria-vc2010 solution file. This will open up all the example projects. 
	Once visual studio opens, right-click on Solution->Add->Existing Project

	Open up the project you want to open from the folders you just copied over

	Now, just to make things simple, lets say you first opened up "Three_Robots_Converge"
	Right-Click on the project file-> Add -> Existing Item -> "threeRobots_converge.cpp"

	Right-Click on the project file-> Properties
	Click on "Common Properties" -> "Add New Reference..."
	Now click on "AriaDLL" and hit OK

	Click on C/C++ -> Additional Include Directories, put in the file location for the ARIA includes

	Click on Linker -> Additional Library Directories, put in thefile location for the ARIA libs

	If you are working with the line following or circle detection projects, you need to include the opencv stuff also and the Flycapture2 sdk stuff
	I'm going to assuming you can figure that part out yourself. It's pretty much the same thing as above.

	Again, had 3 months to work on all of this and making things user friendly wasn't much of a concern.

	If you have any questions, just shoot me an email at shaboinkin@gmail.com, I'll be happy to help


--------------------------------------------------

What's here?

This was my senior thesis project that I completed at Bethune-Cookman University. It contains 6 programs I wrote in the span of 3 months.
Being said, there is some redundancy across the programs that could have been orgainized better but given my time constraints, I just tried to push it 
and get it working as quickly as I could.

aria_robot_mapping
	Creates a map of a static environment using the sonars on the P3-AT robot

Matlab Map Program
	Simple script that takes the .dat file generated from aria_robot_mapping and creates a map from the data provided. Uses the P3-AT robot.

Line_Following_obstacle_avoidance
	Using one camera from the Bumblebee2 stereo camera, I used opencv for color detection of a red line. I used the Bug2 algorithm to allow the robot
	to move around obstacles that may be present on the line while the robot is in operation. Uses the P3-AT robot.

opencv_circle_detection
	Using both cameras from the Bumblebee2 stereo camera, I used opencv for circle detections that allowed me to track and follow a ball based on the distance
	of the ball to the camera. Uses the P3-AT robot.

Three_Robots_Circle_Formation
	Using the Amigo bot, the program connects three robots to follow a circluar path

Three_Robots_Converge
	Using the Amigo bot, the program connects three robots and drives them to meet at a point. User places the three robots and random locations, inputs the
	distance of the robots relative to the lead robot (robot 1) and the robots drive to that point.

Three_Robots_Triangle_Formation
	Using the Amigobots, the program connects three robots, the robots converge to a triangle formation, then they drive off in that formation.

And my Thesis in PDF form.
Might as well toss in the LATEX file used to write it up...

-------------------------------------------




