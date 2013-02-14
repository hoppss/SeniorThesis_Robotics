/********************************************************************************************************************
 *	Using only 1 camera from the Bumblebee2 camera, apply color detection for an object of interest
 *	and have the robot turn so that the object is always near the center of the image frame.
 *
 *	Robot will move and follow the line. 
 *		If the line is within a specified boundy, it will move forward.
 *		If the line is slightly off to the left or right, it will turn accordnly to get back on the line.
 *
 *		
 *	Camera setup is based on Point-Grey's setup code
 *		Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
 *
 *
 *	Created By: Daniel Kulas, Bethune-Cookman University
 *				9/23/12
 ********************************************************************************************************************/


//				!!CONFLICTING FUNCTIONS!!
//		ARIA.H NEEDS TO BE BEFORE ALL OTHER INCLUDES	
//		

#include "Aria.h"

#include <iostream>
#include <opencv\cv.h>
#include <opencv\highgui.h>

#include "FlyCapture2.h"

#define DISTANCE_THRESHOLD	400

using namespace FlyCapture2;
using namespace std;

//opencv class
IplImage* destImage,
		  pProcessedFrame,
		  tempFrame;
IplImage* imgScribble = NULL;
Error error;

//FlyCapture class
Image colorImage;
bool bInit = false;


//sonar thing
bool foundLine_flag = false;
double centerReading;
double readingAngle;

double leftReading;	//50 -> 30 degrees
double leftSideReading; //90 -> 50 degrees

double rightReading; //-50 -> -30
double rightSideReading; //-90 -> -50

//lazy print
void print(char* str)
{
	cout << str << endl;
}

//print out camera specs if desired
void PrintBuildInfo()
{
	FC2Version fc2Version;
	Utilities::GetLibraryVersion(&fc2Version);
	char version[128];
	sprintf(
        version, 
        "FlyCapture2 library version: %d.%d.%d.%d\n", 
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );
    printf( version );

    char timeStamp[512];
    
	sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );
    printf( timeStamp );
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}

//error messaging 
void PrintError(Error error)
{
	error.PrintErrorTrace();
}

/*
 *	Important function!	[Point Grey code]
 *	Converts the raw image taken from the Bumblebee2 camera into an
 *	IplImage format so opencv can understand the data.	
 */
IplImage* ConvertImageToOpenCV(Image* pImage)
{
	IplImage* cvImage = NULL;
	bool bColor = true;
	CvSize mySize;
	mySize.height = pImage->GetRows();
	mySize.width  = pImage->GetCols();

	//switch used in the event that a different camera is being used
		switch ( pImage->GetPixelFormat() )
	{
		case PIXEL_FORMAT_MONO8:	 cvImage = cvCreateImageHeader(mySize, 8, 1 );
									 cvImage->depth = IPL_DEPTH_8U;
									 cvImage->nChannels = 1;
									 bColor = false;
									 printf("PIXEL_FORMAT_MON08()\n");
									 break;

		case PIXEL_FORMAT_411YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 //printf("PIXEL_FORMAT_411YUV8\n");
                                     break;

		case PIXEL_FORMAT_422YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 //printf("PIXEL_FORMAT_433YUV8\n");
                                     break;

		case PIXEL_FORMAT_444YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 //printf("PIXEL_FORMAT_444YUV8\n");
                                     break;

		case PIXEL_FORMAT_RGB8:      cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                    // printf("PIXEL_FORMAT_RGB8\n");
									 break;

		case PIXEL_FORMAT_MONO16:    cvImage = cvCreateImageHeader(mySize, 16, 1 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 1;
									 printf("PIXEL_FORMAT_MONO16\n");
									 bColor = false;
                                     break;

		case PIXEL_FORMAT_RGB16:     cvImage = cvCreateImageHeader(mySize, 16, 3 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 3;
                                     printf("PIXEL_FORMAT_RGB16\n");
									 break;

		case PIXEL_FORMAT_S_MONO16:  cvImage = cvCreateImageHeader(mySize, 16, 1 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 1;									
									 bColor = false;
									 printf("PIXEL_FORMAT_S_MONO16\n");
                                     break;

		case PIXEL_FORMAT_S_RGB16:   cvImage = cvCreateImageHeader(mySize, 16, 3 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 3;
									 printf("PIXEL_FORMAT_X_RGB16\n");
                                     break;

		case PIXEL_FORMAT_RAW8:      cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 printf("PIXEL_FORMAT_RAW8\n");
                                     break;

		case PIXEL_FORMAT_RAW16:     cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 printf("PIXEL_FORMAT_RAW16\n");
                                     break;

		case PIXEL_FORMAT_MONO12:    printf("Not supported by OpenCV");
									 bColor = false;
                                     break;

		case PIXEL_FORMAT_RAW12:	 printf("Not supported by OpenCV");
									 break;

		case PIXEL_FORMAT_BGR:       cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 printf("PIXEL_FORMAT_BGR\n");
                                     break;

		case PIXEL_FORMAT_BGRU:      cvImage = cvCreateImageHeader(mySize, 8, 4 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 4;
									 printf("PIXEL_FORMAT_BGRU\n");
                                     break;

		case PIXEL_FORMAT_RGBU:      cvImage = cvCreateImageHeader(mySize, 8, 4 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 4;
									 printf("PIXEL_FORMAT_RGBU\n");
                                     break;

		default: printf("Some error occured...\n");
				 return NULL;
	}

	if(bColor)
	{
		if(!bInit)
		{
			//sets size of buffer to place image data
			colorImage.SetData(new unsigned char[pImage->GetCols() * pImage->GetRows()*3], pImage->GetCols() * pImage->GetRows()*3);
			bInit = true;
		}
		pImage->Convert(PIXEL_FORMAT_BGR, &colorImage); //needs to be BGR format to save 
		//sets IplImage header values
		cvImage->width = colorImage.GetCols();
		cvImage->height = colorImage.GetRows();
		cvImage->widthStep = colorImage.GetStride();
		cvImage->origin = 0;		//interleaved color channels
		cvImage->imageDataOrigin = (char*)colorImage.GetData();	//no region of interest, same pointer
		cvImage->imageData = (char*)(colorImage.GetData());
		cvImage->widthStep = colorImage.GetStride();
		cvImage->nSize = sizeof(IplImage);
		cvImage->imageSize = cvImage->height * cvImage->widthStep;
	}
	else
	{
        cvImage->imageDataOrigin = (char*)(pImage->GetData());
        cvImage->imageData         = (char*)(pImage->GetData());
        cvImage->widthStep         = pImage->GetStride();
        cvImage->nSize             = sizeof (IplImage);
        cvImage->imageSize         = cvImage->height * cvImage->widthStep;
		//at this point cvImage contains a valid IplImage
     }
	return cvImage;
}

//Color Detection 
IplImage* getThresholdedImage(IplImage* img)
{
	//convert image to HSV image (HSV is a color format: Hue, Saturation, value aka brightness)
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
	cvCvtColor(img, imgHSV, CV_BGR2HSV);

	IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

	//what color you want to detect
	cvInRangeS(imgHSV, cvScalar(2,160,50), cvScalar(7, 210, 150), imgThreshed);
	cvReleaseImage(&imgHSV);

	return imgThreshed;
}

int main(int argc, char* argv[])
{
	//Setup Aria stuff
	ArRobot robot;
	ArKeyHandler keyHandler;
	//using sonar due to issues connecting the laser
	ArSonarDevice sonar;	

	//now camera stuff
	Camera cam;
	Image rawImage;

	char keypress;

	//Connect robot
	Aria::init();

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
		print("Could not connect to robot...abort");
		Aria::shutdown();
		exit(1);
	}

	//motors on, sounds off
	robot.comInt(ArCommands::ENABLE, 1);
	robot.comInt(ArCommands::SOUNDTOG, 0);
	robot.runAsync(true);

	//Deal with camera now
	PrintBuildInfo();
	Error error;

	BusManager busMgr;
	unsigned int numCameras;

	error = busMgr.GetNumOfCameras(&numCameras);
	if(error != PGRERROR_OK)
	{
		PrintError(error);
		exit(1);
	}
	printf("Number of cameras detected: %u\n", numCameras);
	PGRGuid guid;
	for(unsigned int i = 0; i < numCameras; i++)
	{
		error = busMgr.GetCameraFromIndex(i, &guid);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}
	}

	//connect and startup camera
	error = cam.Connect(&guid);
	if(error != PGRERROR_OK)
	{
		PrintError(error);
		exit(1);
	}

	CameraInfo camInfo;
	error = cam.GetCameraInfo(&camInfo);
	if(error != PGRERROR_OK)
	{
		PrintError(error);
		exit(1);
	}

	error = cam.StartCapture();
	if(error != PGRERROR_OK)
	{
		PrintError(error);
		exit(1);
	}

	while(1)
	{
		//grab image
		error = cam.RetrieveBuffer(&rawImage);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}

		//convert raw image to opencv format IplImage
		destImage = ConvertImageToOpenCV(&rawImage);
		
		//color detection stuff
		IplImage* imgColorThreshold = getThresholdedImage(destImage);

		//Get moment of thresholded object to track
		CvMoments* moments = (CvMoments*)malloc(sizeof(CvMoments));
		cvMoments(imgColorThreshold, moments, 1);

		double moment10 = cvGetSpatialMoment(moments, 1, 0);
		double moment01 = cvGetSpatialMoment(moments, 0, 1);
		double area = cvGetCentralMoment(moments, 0, 0);

		//hold x/y position of center of gravity
		static int posX = 0;
		static int posY = 0;
		int lastX = posX;
		int lastY = posY;

		posX = moment10/area;
		posY = moment01/area;
		printf("Position (%d, %d)\n", posX, posY);

		//check if front is clear from sonar data
		centerReading = sonar.currentReadingPolar(-10, 10, &readingAngle);	//sensor_3, sensor_4
		if(centerReading > 700)
		{
			print("Front is clear, following line");
			if(posX < 482 && posX > 282)
			{
				robot.setRotVel(15);
				print("Center of xFrame is too far to the right, turning left");
			}
			else if(posX < 282)
			{
				robot.setRotVel(40);
				print("Center of xFrame is wayyyyy to far to the right, turning left faster");
			}
			else if(posX > 542 && posX < 742)
			{
				robot.setRotVel(-15);
				print("Center of xFrame is too far to the left, turning right");
			}
			else if(posX > 742)
			{
				robot.setRotVel(-40);
				print("Center of xFrame is wayyyyy to far to the left, turning right faster");
			}
			else 
			{
				robot.setRotVel(0);
				robot.setVel(200);
				print("Center of xFrame is withing the bounding box...following line");
			}
		}
		else if (centerReading < 700)
		{
			robot.setVel(0);
			print("Checking to find shortest distance");
			leftReading  = sonar.currentReadingPolar( 30,  50, &readingAngle);
			rightReading = sonar.currentReadingPolar(-30, -50, &readingAngle);

			if( leftReading > rightReading)
			{
				print("Right side shorter, turning right.");
				robot.setDeltaHeading(-90);
				ArUtil::sleep(1000);
				robot.setVel(100);
				ArUtil::sleep(500);
				for(;;)
				{
					leftSideReading = sonar.currentReadingPolar(60, 90, &readingAngle);
					leftReading  = sonar.currentReadingPolar( 30,  50, &readingAngle);
					cout << "leftside " << leftSideReading << endl;
					cout << "leftReading" << leftReading << endl;
					//grab image
					error = cam.RetrieveBuffer(&rawImage);
					if(error != PGRERROR_OK)
					{
						PrintError(error);
						exit(1);
					}

					//convert raw image to opencv format IplImage
					destImage = ConvertImageToOpenCV(&rawImage);
					
					//color detection stuff
					IplImage* imgColorThreshold = getThresholdedImage(destImage);

					//Get moment of thresholded object to track
					CvMoments* moments = (CvMoments*)malloc(sizeof(CvMoments));
					cvMoments(imgColorThreshold, moments, 1);

					double moment10 = cvGetSpatialMoment(moments, 1, 0);
					double moment01 = cvGetSpatialMoment(moments, 0, 1);
					double area = cvGetCentralMoment(moments, 0, 0);
					cout << "Moment10 " << moment10 << endl;
					cout << "Moment01 " << moment01 << endl;
					cout << "Area " << area << endl;
					//hold x/y position of center of gravity
					static int posX = 0;
					static int posY = 0;
					int lastX = posX;
					int lastY = posY;

					posX = moment10/area;
					posY = moment01/area;
					cvShowImage("Color detection", imgColorThreshold);
					keypress = cvWaitKey(10);

					
					//keep set distance away from wall
					if(leftSideReading > DISTANCE_THRESHOLD || leftReading > DISTANCE_THRESHOLD + 100)
					{
						
						robot.setVel(200);
						robot.setRotVel(20);
					}

					if(leftSideReading > DISTANCE_THRESHOLD && leftSideReading < DISTANCE_THRESHOLD + 100 
						|| leftReading > DISTANCE_THRESHOLD + 100 && leftReading < DISTANCE_THRESHOLD + 200)
					{
						robot.setRotVel(0);
						robot.setVel(150);
					}
					if ( leftSideReading < DISTANCE_THRESHOLD)		
					{
						robot.setVel(150);
						robot.setRotVel(-20);			
					}
					if(area > 2500)
					{
						print("Line found");
						break;
					}

					cvReleaseImage(&imgColorThreshold);
					delete moments;
				}			
			}
			else if( rightReading > leftReading)
			{
				print("Left side shorter, turning left.");
				robot.setDeltaHeading(90);
				ArUtil::sleep(1000);
				robot.setVel(100);
				ArUtil::sleep(500);
				for(;;)
				{
					rightSideReading = sonar.currentReadingPolar(-60, -90, &readingAngle);
					rightReading  = sonar.currentReadingPolar( -30,  -50, &readingAngle);
					cout << "rightSideReading " << rightSideReading << endl;
					cout << "rightReading" << rightReading << endl;
					//grab image
					error = cam.RetrieveBuffer(&rawImage);
					if(error != PGRERROR_OK)
					{
						PrintError(error);
						exit(1);
					}

					//convert raw image to opencv format IplImage
					destImage = ConvertImageToOpenCV(&rawImage);
					
					//color detection stuff
					IplImage* imgColorThreshold = getThresholdedImage(destImage);

					//Get moment of thresholded object to track
					CvMoments* moments = (CvMoments*)malloc(sizeof(CvMoments));
					cvMoments(imgColorThreshold, moments, 1);

					double moment10 = cvGetSpatialMoment(moments, 1, 0);
					double moment01 = cvGetSpatialMoment(moments, 0, 1);
					double area = cvGetCentralMoment(moments, 0, 0);
					cout << "Moment10 " << moment10 << endl;
					cout << "Moment01 " << moment01 << endl;
					cout << "Area " << area << endl;
					//hold x/y position of center of gravity
					static int posX = 0;
					static int posY = 0;
					int lastX = posX;
					int lastY = posY;

					posX = moment10/area;
					posY = moment01/area;
					cvShowImage("Color detection", imgColorThreshold);
					keypress = cvWaitKey(10);

					
					//keep set distance away from wall
					if(rightSideReading > DISTANCE_THRESHOLD || rightReading > DISTANCE_THRESHOLD + 100)
					{
						
						robot.setVel(200);
						robot.setRotVel(-20);
					}

					if(rightSideReading > DISTANCE_THRESHOLD && rightSideReading < DISTANCE_THRESHOLD + 100 
						|| rightReading > DISTANCE_THRESHOLD + 100 && rightReading < DISTANCE_THRESHOLD + 200)
					{
						robot.setRotVel(0);
						robot.setVel(150);
					}
					if ( rightSideReading < DISTANCE_THRESHOLD)		
					{
						robot.setVel(150);
						robot.setRotVel(20);
					}
					if(area > 2500)
					{
						print("Line found");
						break;
					}

					cvReleaseImage(&imgColorThreshold);
					delete moments;
				}			
			}
			else
				print("lol");
		}

		cvShowImage("Color detection", imgColorThreshold);
		keypress = cvWaitKey(10);

		cvReleaseImage(&imgColorThreshold);
		delete moments;
	}

	cvReleaseImageHeader(&destImage);
	cvReleaseImage(&destImage);

	error = cam.StopCapture();
	if(error != PGRERROR_OK)
	{
		PrintError(error);
		exit(1);
	}

	error = cam.Disconnect();
	if(error != PGRERROR_OK)
	{
		PrintError(error);
		exit(1);
	}
	print("Done");
	Aria::shutdown();

	return 0;
}
