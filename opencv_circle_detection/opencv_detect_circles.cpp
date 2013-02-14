/************************************************************************************************
 *	Using Stereovision, detection circles in both frame, find Moments, and get distance of 
 *	circle object from the caamera
 *
 *	Camera setup is based on Point-Grey's setup code
 *		Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
 *
 *	Created By: Daniel Kulas, Bethune-Cookman University
 *				10/3/12
 ************************************************************************************************/

/*======================================================= 
 *				!!CONFLICTING FUNCTIONS!!
 *		ARIA.H NEEDS TO BE BEFORE ALL OTHER INCLUDES	
 *=======================================================*/		

#include "Aria.h"
#include <iostream>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <stdio.h>
#include <stdlib.h>

#include "FlyCapture2.h"

using namespace cv;
using namespace std;
using namespace FlyCapture2;

//opencv class
IplImage* leftImage;
IplImage* leftImage_smooth;
IplImage* rightImage;
IplImage* rightImage_smooth;
IplImage* leftGray;
IplImage* rightGray;

//FlyCapture class
Image colorImage;
Error error;
bool bInit = false;

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
 *  Important function!	[Point Grey code]
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
									// printf("PIXEL_FORMAT_MON08()\n");
									 break;

		case PIXEL_FORMAT_411YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									// printf("PIXEL_FORMAT_411YUV8\n");
                                     break;

		case PIXEL_FORMAT_422YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
								//	 printf("PIXEL_FORMAT_433YUV8\n");
                                     break;

		case PIXEL_FORMAT_444YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
								//	 printf("PIXEL_FORMAT_444YUV8\n");
                                     break;

		case PIXEL_FORMAT_RGB8:      cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                    // printf("PIXEL_FORMAT_RGB8\n");
									 break;

		case PIXEL_FORMAT_MONO16:    cvImage = cvCreateImageHeader(mySize, 16, 1 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 1;
									// printf("PIXEL_FORMAT_MONO16\n");
									 bColor = false;
                                     break;

		case PIXEL_FORMAT_RGB16:     cvImage = cvCreateImageHeader(mySize, 16, 3 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 3;
                                   //  printf("PIXEL_FORMAT_RGB16\n");
									 break;

		case PIXEL_FORMAT_S_MONO16:  cvImage = cvCreateImageHeader(mySize, 16, 1 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 1;									
									 bColor = false;
									// printf("PIXEL_FORMAT_S_MONO16\n");
                                     break;

		case PIXEL_FORMAT_S_RGB16:   cvImage = cvCreateImageHeader(mySize, 16, 3 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 3;
									// printf("PIXEL_FORMAT_X_RGB16\n");
                                     break;

		case PIXEL_FORMAT_RAW8:      cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 //printf("PIXEL_FORMAT_RAW8\n");
                                     break;

		case PIXEL_FORMAT_RAW16:     cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 //printf("PIXEL_FORMAT_RAW16\n");
                                     break;

		case PIXEL_FORMAT_MONO12:    printf("Not supported by OpenCV");
									 bColor = false;
                                     break;

		case PIXEL_FORMAT_RAW12:	 printf("Not supported by OpenCV");
									 break;

		case PIXEL_FORMAT_BGR:       cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
									 //printf("PIXEL_FORMAT_BGR\n");
                                     break;

		case PIXEL_FORMAT_BGRU:      cvImage = cvCreateImageHeader(mySize, 8, 4 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 4;
									 //printf("PIXEL_FORMAT_BGRU\n");
                                     break;

		case PIXEL_FORMAT_RGBU:      cvImage = cvCreateImageHeader(mySize, 8, 4 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 4;
									// printf("PIXEL_FORMAT_RGBU\n");
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
		cvImage->width = 640;//colorImage.GetCols();
		cvImage->height = 480;//colorImage.GetRows();
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

int main(int argc, char* argv[])
{
	//Setup robot stuff
	ArRobot robot;
	ArKeyHandler keyHandler;
	Camera cam;
	Image rawImage_left;
	Image rawImage_right;
	float* p_left;
	float* p_right;
	//char keypress;

	double focalLen = 6;	//6 millimeters
	int distance_between_camera = 120;	// in millimeters
	int distance_from_object;
	int left_x;
	int right_x;

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
	if(!connector.connectRobot(&robot))
	{
		std::cout << "Could not connect to robot...abort" << std::endl;
		Aria::shutdown();
		return -1;
	}

	robot.comInt(ArCommands::ENABLE, 1);
	robot.comInt(ArCommands::SOUNDTOG, 0);

	//Setup camera stuff
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
	for (unsigned int i=0; i < numCameras; i++)
    {
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            exit(1);
		}

    }

    //Connect, get info, start getting images
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

	//run robot on background thread
	robot.runAsync(true);
	
	while(1)
	{
		//grab LEFT image. 		
		//WriteRegister(Register to write too, value to write to register, broadcast this image)
		error = cam.WriteRegister(0x884, 0x82000000, true);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}
		
		error = cam.RetrieveBuffer(&rawImage_left);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}

		//get source image
		leftImage = ConvertImageToOpenCV(&rawImage_left);

		//Set to false...or else things get weird
		error = cam.WriteRegister(0x884, 0x82000000, false);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}
	
		//smooth image to prevent false detection of circles
		cvSmooth(leftImage, leftImage);

		//allocate memory for leftGray, the "1" parameter at the end states how many channels it has
		//aka, how many colors. 1 channel = gray scale, 3 channels = RGB
		leftGray = cvCreateImage(cvSize( leftImage->width, leftImage->height), IPL_DEPTH_8U, 1);

		//convert to gray scale
		cvCvtColor(leftImage, leftGray, CV_BGR2GRAY);

		//Allocates memory to store the locations of circles detected
		CvMemStorage* circles_storage_left = cvCreateMemStorage(0);

		//magic happens here! Apply the Hough Transform algorithm to detect circles, any circles detected gets 
		//stored in the circles_storage_left CvMemStorage variable
		CvSeq* circles_left = cvHoughCircles(leftGray, circles_storage_left, CV_HOUGH_GRADIENT, 2, 20);

		/* 
			For-loop condition might be a "what the crap is this" for you as it was for me
			This is called a Ternary Operator.
				Binary = consisting of two
				Ternary = consisting of three

			Essentially, if 'circles_left' == true 
				then the expression evalutes to circles_left->total
			If false
				it evalutes to 0

			To break it down some more...
			If "circles_left" has values inside of it, find out how many values it contains, in this case, circles detected
			If "circles_left" has no values inside of it, then there are no circles detected. Don't go in for-loop.
		*/
			
		for(size_t i = 0; i < (circles_left ? circles_left->total : 0); i++)
		{
			p_left = (float *)cvGetSeqElem(circles_left, i);
			cvCircle(leftImage, cvPoint(cvRound(p_left[0]), cvRound(p_left[1])), 3, CV_RGB(0, 255, 0), -1, 8, 0); 
			left_x = p_left[0];
		}

		/* ===== RELEASE THINGS ONCE DONE WITH IT!!!! =====*/
		// Unless you want crap loads of memory leaks and programs that crash on you
		cvShowImage("Circle Detection on LEFT camera", leftImage);
		cvReleaseImage(&leftImage);
		cvReleaseImage(&leftGray);
		cvReleaseImage(&leftImage_smooth);
		cvReleaseMemStorage(&circles_storage_left);
		cv::waitKey(100);

		/*=========================================================*/
		/*================grab RIGHT image=========================*/
		/*=========================================================*/
		error = cam.WriteRegister(0x884, 0x82000001, true);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}

		error = cam.RetrieveBuffer(&rawImage_right);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}

		rightImage = ConvertImageToOpenCV(&rawImage_right);

		error = cam.WriteRegister(0x884, 0x82000001, false);
		if(error != PGRERROR_OK)
		{
			PrintError(error);
			exit(1);
		}

		//smooth image to prevent false detection of circles
		cvSmooth(rightImage, rightImage);

		rightGray = cvCreateImage(cvSize( rightImage->width, rightImage->height), IPL_DEPTH_8U, 1);
		cvCvtColor(rightImage, rightGray, CV_BGR2GRAY);
		CvMemStorage* circles_storage_right = cvCreateMemStorage(0);

		CvSeq* circles_right = cvHoughCircles(rightGray, circles_storage_right, CV_HOUGH_GRADIENT, 2, 20);

		for(size_t i = 0; i < (circles_right ? circles_right->total : 0); i++)
		{
			p_right = (float *)cvGetSeqElem(circles_right, i);
			cvCircle(rightImage, cvPoint(cvRound(p_right[0]), cvRound(p_right[1])), 3, CV_RGB(0, 255, 0), -1, 8, 0); 
			right_x = p_right[0];
		}

		cvShowImage("Circle Detection on RIGHT camera", rightImage);

		cvReleaseImage(&rightImage);
		cvReleaseImage(&rightGray);
		cvReleaseImage(&rightImage_smooth);
		cvReleaseMemStorage(&circles_storage_right);

		cv::waitKey(100);

		/*====================================================*/
		/*=========CALCULATE DISTANCE FROM MOMENTS============*/
		/*====================================================*/

		/* one thing to point out if you haven't noticed.
			In color detection, I used moments to figure out the position of the colored object
			Moments don't seem to work for circle detection. Didn't dwell too deep as to the cause of this. */

		distance_from_object = distance_between_camera * (focalLen/(left_x - right_x)) * 34/2.62;
		cout << "Distance from camera: " << distance_from_object << endl;
		cout << "p_left: " << left_x << endl;
		cout << "p_right: " << right_x << endl;
		cout << "---------------------------" << endl;

		/*====================================================*/
		/*=====================ROBOT ROCK=====================*/
		/*====================================================*/

		
		if( left_x > 145 && left_x < 250 )
		{
			robot.setRotVel(9);
			//turn right, off center
		}
		else if( left_x < 145 )
		{
			robot.setRotVel(14);
			//turn right faster, way off center
		}
		else if( left_x > 400 && left_x < 495 )
		{
			robot.setRotVel(-9);
			//turn left, off center
		}
		else if( left_x > 495 )
		{
			robot.setRotVel(-14);
			//turn left faster, way off center
		}
		else if ( left_x > 250 && left_x < 400)
		{

			if(distance_from_object > 200)
			{
				robot.setVel(300);	//object is far, speed up
				robot.setRotVel(0);
				cout << " < 200 [cm] away from object" << endl;
			}
			else if(distance_from_object < 200 && distance_from_object > 100)
			{
				robot.setVel(100);	//object near, drive normal speed
				robot.setRotVel(0);
				cout << " > 200 [cm] away from object" << endl;
			}
			else if ( distance_from_object < 100)
			{
				robot.setVel(0);	//object to close. Stop
				robot.setRotVel(0);
				cout << " stop" << endl;
			}

		}
	}

	//Memory management
	cvReleaseImageHeader(&leftImage);
	cvReleaseImage(&leftImage);
	cvReleaseImageHeader(&leftGray);
	cvReleaseImage(&leftGray);

	cvReleaseImage(&rightImage);
	cvReleaseImageHeader(&rightImage);
	cvReleaseImageHeader(&rightGray);
	cvReleaseImage(&rightGray);

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
	printf("Done\n");

	Aria::shutdown();
	
    return 0;
}
