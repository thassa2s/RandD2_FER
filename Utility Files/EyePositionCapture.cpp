/*
 * EyePositionCapture.cpp
 *
 *  Created on: Jul 25, 2013
 *      Author: teenarahul
 */
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>

#include <opencv2/opencv.hpp>

CvPoint left_eye_pos, right_eye_pos;
bool both_eye_positions_captured = false;

using namespace std;

void mouseClickEvent(int event, int pixel_x, int pixel_y, int flags, void* param)
{
	static unsigned int click_event_count = 0;
    if( event == CV_EVENT_LBUTTONDOWN )
    {
        cout << "Coordinates received: " << pixel_x << ", " << pixel_y << endl;
        click_event_count++;
        if ( click_event_count == 1 )
        {
        	cout << "Person's right eye" << endl;
        	right_eye_pos.x = pixel_x;
        	right_eye_pos.y = pixel_y;
        	both_eye_positions_captured = false;
        }
        else
        {
        	cout << "Person's left eye" << endl;
        	left_eye_pos.x = pixel_x;
        	left_eye_pos.y = pixel_y;
        	both_eye_positions_captured = true;
        	click_event_count = 0;
        }
    }
}


void show_image( string full_imagename, string window_name )
{
    cvNamedWindow( window_name.c_str(), CV_WINDOW_NORMAL );
    cvSetMouseCallback( window_name.c_str(), mouseClickEvent, 0 );
    IplImage* image = cvLoadImage( full_imagename.c_str() );
    cvShowImage( window_name.c_str(), image );
    do
    {
    	cvWaitKey( 0 );
    }while ( !both_eye_positions_captured ); //Do not close the window without capturing both the eye positions.
    cvDestroyWindow( window_name.c_str() );
    cvReleaseImage( &image );
}

void save_eye_positions( string folder_path, string image_name )
{
	//Enhance to metadata file, if the correct expression can be retrieved from file name of the image.
	//Also, get the name of the metadata file as command line arguments.

	ofstream outfile;
	string full_filename = folder_path + string( "ImageEyePositions.txt" );
	outfile.open( full_filename.c_str(), ios::app );
    if ( !outfile )
    {
    	cout << "Could not open file " << full_filename << " for writing." << endl;
    	return;
    }
    if ( outfile.tellp() != 0 )
    {
    	outfile << endl;
    }
    else
    {
    	outfile << "** Format: <image file name> <left eye position: X> < left eye position: Y> <right eye position: X> < right eye position: Y>" << endl;
    }
    outfile << image_name << " ";
    outfile << left_eye_pos.x << " " << left_eye_pos.y << " ";
    outfile << right_eye_pos.x << " " << right_eye_pos.y;
    outfile.close();
}

int main( int argc, char *argv[] )
{
	if ( argc < 3 )
	{
		cout << "Provide the following details of file that contains the image names." << endl;
		cout << "Expected command line arguments: <full path to directory> <filename>" << endl;
		return -1;
	}

	string folder_path = argv[1];
	string full_filename = folder_path + argv[2];
    ifstream infile;
    infile.open( full_filename.c_str() );
    if ( !infile )
    {
    	cout << "Could not open file: " << full_filename << endl;
    	return -1;
    }

    cout << "***** Instructions *****" << endl;
    cout << "First left-click on the person's right eye, and then left-click on the left eye." << endl;
    cout << "After an even number of left-clicks, press any key to close the image window." << endl;
    cout << "************************" << endl;

    string image_name;
    string full_imagename;
    infile >> image_name; //Works even if the file is empty.
    while ( infile.peek() != EOF )
    {
    	full_imagename = folder_path + image_name;
    	both_eye_positions_captured = false;
    	show_image( full_imagename, image_name );
    	save_eye_positions( folder_path, image_name );
    	infile >> image_name;
    }
    infile.close();
}




