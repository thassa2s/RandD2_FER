/*
 * FlandmarkFaceEyeDetector.cpp
 *
 *  Created on: Jul 31, 2013
 *      Author: teenarahul
 *      Comments: Base code sourced from: http://cmp.felk.cvut.cz/~uricamic/flandmark/
 */
#include <opencv2/opencv.hpp>


#include "flandmark_detector.cpp"
#include "liblbp.cpp"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace cv;

ofstream outfile;

CvPoint left_eye, right_eye;

FLANDMARK_Model * model;

bool find_eye_positions_in_image( string full_imagename, bool show_feature_positions, bool show_eye_positions );
void save_metadata( string image_name );
string get_emotion_label_from_char( char ch1, char ch2 );

#define JAFFE 0

int main(int argc, char * argv[])
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

    // load flandmark model structure and initialize
 	model = flandmark_init("/home/teenarahul/RandD2/flandmark/flandmark/data/flandmark_model.dat");

 	if ( model == NULL )
 	{
 		cout << "Could not load flandmark model." << endl;
 		return -1;
 	}

	string full_output_filename = folder_path + string( "Metadata_Flandmark.txt" );
	outfile.open( full_output_filename.c_str(), ios::out );
    if ( !outfile.is_open() )
    {
    	cout << "Could not open file " << full_filename << " for writing." << endl;
    	return -1;
    }
	outfile << "** Format: <image file name> <left eye position: X> < left eye position: Y> <right eye position: X> < right eye position: Y> <expression label>";

	bool success = false;
    string image_name;
    string full_imagename;
    infile >> image_name; //Works even if the file is empty.

    while ( infile.peek() != EOF )
    {
    	success = false;
    	full_imagename = folder_path + image_name;
    	success = find_eye_positions_in_image( full_imagename, false, false );
        if ( !success )
        {
        	cout << "Eye positions not detected in image: " << image_name << "." << endl;
        }
        else
        {
    	    save_metadata( image_name );
        }
    	infile >> image_name;
    }
    infile.close();
    outfile.close();
}

bool find_eye_positions_in_image( string full_imagename, bool show_feature_positions, bool show_eye_positions )
{
 	IplImage *img = cvLoadImage( full_imagename.c_str(), CV_LOAD_IMAGE_COLOR );
  //  cv::Mat mat_img = cv::imread("/home/teenarahul/Passbild.jpg", CV_LOAD_IMAGE_COLOR);
	IplImage *img_grayscale = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img, img_grayscale, CV_BGR2GRAY);

	// bbox with detected face (format: top_left_col top_left_row bottom_right_col bottom_right_row)
	//int bbox[] = {72, 72, 183, 183};  //worked for JAFFE
	int bbox[] = {400, 800, 2500, 3000}; //for higher resolution images
	//int bbox[] = {72, 72, 250, 350};  //for pictures
	//int bbox[] = {20, 20, 90, 120};  //for thumbnails
	// detect facial landmarks (output are x, y coordinates of detected landmarks)
	double * landmarks = (double*)malloc(2*model->data.options.M*sizeof(double));

	//int margin[2] = { 70, 70 }; //worked for JAFFE
	int margin[2] = { 2, 2 };

	//std::cout << bbox[0] << landmarks[0] << std::endl;
	//std::cout << (model->data.options.M) << std::endl;

	int retval = flandmark_detect(img_grayscale, bbox, model, landmarks, margin );

	if ( retval != 0 )
	{
		return false;
	}

	//std::cout << bbox[0] << landmarks[0] << " " << landmarks[1];
	//Left eye
	left_eye.x = (landmarks[12] + landmarks[4])/2;
	left_eye.y = (landmarks[13] + landmarks[5])/2;

    //Right eye
    right_eye.x = (landmarks[2] + landmarks[10])/2;
    right_eye.y = (landmarks[3] + landmarks[11])/2;

	bool show_image = (show_feature_positions || show_eye_positions);
	if ( show_feature_positions )
	{
        CvPoint pt1, pt2;
        cv::Scalar color( 255, 0, 0 );
        CvPoint center;
        int radius = cvRound( 3 );
        pt1.x = bbox[0];
        pt1.y = bbox[1];
        pt2.x = bbox[2];
        pt2.y = bbox[3];
        cvRectangle( img, pt1, pt2, color, 10, 8, 0 );
        for( int j = 0; j < model->data.options.M; j++ )
        {
            center.x =  landmarks[2*j];
            center.y = landmarks[2*j+1];
           cvCircle( img, center, radius, color, 10, 8, 0 );
        }
	}
	if ( show_eye_positions )
	{
        int radius = cvRound( 3 );
        cv::Scalar new_color( 0, 255, 0 );
        cvCircle( img, left_eye, radius, new_color, 10, 8, 0 );
        cvCircle( img, right_eye, radius, new_color, 10, 8, 0 );
	}
	if ( show_image )
	{
  	    cvNamedWindow( "landmarks", CV_WINDOW_NORMAL );
	    cvShowImage( "landmarks", img );
	    int c = cvWaitKey(0);
	    if ( c == 'n' )
	    {
	    	cout << "unsatisfactory: " << full_imagename << endl;
	    }
	}
	cvReleaseImage( &img_grayscale );
	cvReleaseImage( &img );
	free( landmarks );
	return true;
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
		cout << it.current_token() << endl;
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
    outfile << left_eye.x << " " << left_eye.y << " ";
    outfile << right_eye.x << " " << right_eye.y << " ";
    outfile << emotion_label;
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
