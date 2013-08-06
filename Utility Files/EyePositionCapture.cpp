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
#include <boost/tokenizer.hpp>

#include <opencv2/opencv.hpp>

#define JAFFE 0

using namespace std;

CvPoint left_eye_pos, right_eye_pos;
bool both_eye_positions_captured = false;

ofstream outfile;

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

string get_emotion_label_from_char( char ch1, char ch2 )
{
	ch1 = tolower( ch1 );
	ch2 = tolower( ch2 );
	switch ( ch1 )
	{
	case 'a': return string("anger");
	case 'd': return string("disgust");
	case 'f': return string("fear");
	case 'h': return string("joy");
	case 's': if ( JAFFE && ( ch2 == 'u' ) )
	              return string("surprise");
		      return string("sadness");
	case 'n': return string("neutral");
	}
}

void save_metadata( string image_name )
{
	boost::char_separator<char> separator("_");

    int index = -1;
	if ( JAFFE )
	{
		boost::char_separator<char> separator1(".");
		separator = separator1;
		index = 2;
	}
	else
	{
		boost::char_separator<char> separator1("_");
		separator = separator1;
		index = 4;
	}
    boost::tokenizer< boost::char_separator<char> > string_tokens(image_name, separator);
	char ch1 = 'a';
	char ch2 = 'n';
    int i = 0;

    for( boost::tokenizer< boost::char_separator<char> >::iterator it = string_tokens.begin(); it != string_tokens.end(); it++ )
    {
		i++;
		if ( i == index )
		{
			ch1 = it.current_token().at( 0 );
			if ( JAFFE )
				ch2 = it.current_token().at( 1 );
		}
	}
    string emotion_label;
    if ( JAFFE )
    {
    	emotion_label = get_emotion_label_from_char( ch1, ch2 );
    }
    else
    {
    	emotion_label = get_emotion_label_from_char( ch1, ' ' );
    }
    outfile << endl;
    outfile << image_name << " ";
    outfile << left_eye_pos.x << " " << left_eye_pos.y << " ";
    outfile << right_eye_pos.x << " " << right_eye_pos.y << " ";
    outfile << emotion_label;

    cout << image_name << " ";
    cout << left_eye_pos.x << " " << left_eye_pos.y << " ";
    cout << right_eye_pos.x << " " << right_eye_pos.y << " ";
    cout << emotion_label;
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
    if ( !infile.is_open() )
    {
    	cout << "Could not open file: " << full_filename << endl;
    	return -1;
    }

	string full_output_filename = folder_path + string( "Metadata.txt" );
	outfile.open( full_output_filename.c_str(), ios::out );
    if ( !outfile.is_open() )
    {
    	cout << "Could not open file " << full_filename << " for writing." << endl;
    	return -1;
    }
	outfile << "** Format: <image file name> <left eye position: X> < left eye position: Y> <right eye position: X> < right eye position: Y> <expression label>";


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
    	save_metadata( image_name );
    	infile >> image_name;
    }
    infile.close();
}




