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

int main(int argc, char * argv[])
{
    // load flandmark model structure and initialize
 	FLANDMARK_Model * model = flandmark_init("/home/teenarahul/RandD2/flandmark/flandmark/data/flandmark_model.dat");

	// load input image
 	IplImage *img = cvLoadImage("/home/teenarahul/Passbild.jpg", CV_LOAD_IMAGE_COLOR);
    cv::Mat mat_img = cv::imread("/home/teenarahul/Passbild.jpg", CV_LOAD_IMAGE_COLOR);
    // convert image to grayscale
	IplImage *img_grayscale = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img, img_grayscale, CV_BGR2GRAY);

	// bbox with detected face (format: top_left_col top_left_row bottom_right_col bottom_right_row)
	//int bbox[] = {72, 72, 183, 183};  //worked for JAFFE
	int bbox[] = {400, 800, 2500, 3000}; //for higher resolution images
	// detect facial landmarks (output are x, y coordinates of detected landmarks)
	double * landmarks = (double*)malloc(2*model->data.options.M*sizeof(double));
	//int margin[2] = { 70, 70 }; //worked for JAFFE
	int margin[2] = { 7, 7 };

	std::cout << bbox[0] << landmarks[0] << std::endl;
	std::cout << (model->data.options.M) << std::endl;
	flandmark_detect(img_grayscale, bbox, model, landmarks, margin );
	std::cout << bbox[0] << landmarks[0] << " " << landmarks[1];
    CvPoint pt1, pt2;
    pt1.x = bbox[0];
    pt1.y = bbox[1];
    pt2.x = bbox[2];
    pt2.y = bbox[3];
    cv::Scalar color( 255, 0, 0 );
    CvPoint center;
    int radius = cvRound( 3 );
    cvRectangle( img, pt1, pt2, color, 10, 8, 0 );
    for( int j = 0; j < model->data.options.M; j++ )
     {
       center.x =  landmarks[2*j];
       center.y = landmarks[2*j+1];

       cvCircle( img, center, radius, color, 10, 8, 0 );
     }

    cv::Scalar new_color( 0, 255, 0 );
    //Left eye
    center.x = (landmarks[12] + landmarks[4])/2;
    center.y = (landmarks[13] + landmarks[5])/2;

    cvCircle( img, center, radius, new_color, 10, 8, 0 );

    //Right eye
    center.x = (landmarks[2] + landmarks[10])/2;
    center.y = (landmarks[3] + landmarks[11])/2;

    cvCircle( img, center, radius, new_color, 10, 8, 0 );

	cvNamedWindow( "landmarks", CV_WINDOW_NORMAL );
	cvShowImage( "landmarks", img );
	cvWaitKey(0);
	cvReleaseImage( &img_grayscale );
	cvReleaseImage( &img );

	free( landmarks );
}

