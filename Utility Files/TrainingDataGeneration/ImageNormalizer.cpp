/*
 * ImageNormalizer.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: teenarahul
 *      Adaptation of code written by Geovanny Macedo
 */

#define PI 3.14159

#include <math.h>
#include <highgui.h>
#include <cxcore.h>
#include <cv.h>

#define SHOW_IMAGES 0

using namespace::std;

CvPoint left_eye, right_eye, upper_left_corner, lower_right_corner;
int scale_width, scale_height;

/**
/*	This functions will compute the rotation angle of the image.
/*
/*	Positive values of slope mean counter-clockwise rotation (the coordiate
/*	origin is assumed at top-left corner).
/*
/*	angle: is the rotation angle in degress.
**/
float computeLineOfEyesAngle()
{
	float slope = (float)(left_eye.y - right_eye.y) / (left_eye.x - right_eye.x);
	float angle = atan(slope)*180/PI;

	/*cout<<endl <<"slope: " <<slope <<endl;
	cout<<"angle: " <<angle <<endl;*/

	return angle;
}

void rotateImage(IplImage* src, IplImage* &dest, float angle, CvPoint2D32f img_center)
{
	float isoScale = 1;																		//Isotropic scale factor.

	CvMat *rotMat = cvCreateMat(2, 3, CV_32FC1);					//2�3 map matrix.

	cv2DRotationMatrix(img_center, angle, isoScale, rotMat);
	cvWarpAffine(src, dest, rotMat);

	cvReleaseMat(&rotMat);
}

void computePositionOfEyesAfterRotating(float angle, CvPoint2D32f *img_center)
{
	float angle_rad = angle * 3.14159/180;
	float alpha =	cos(angle_rad);
	float beta =	sin(angle_rad);
	float factor1 = (1 - alpha)*img_center->x - beta*img_center->y;
	float factor2 = (beta * img_center->x) + ((1 - alpha) * img_center->y);

	float temp1 = 0, temp2 = 0;

	//Left Eye
	temp1 = (-beta * left_eye.x);
	temp2 = (alpha * left_eye.y);

	left_eye.x = (alpha * left_eye.x) + (beta * left_eye.y) + factor1;
	left_eye.y = temp1 + temp2 + factor2;

	//Right Eye
	temp1 = (-beta * right_eye.x);
	temp2 = (alpha * right_eye.y);

	right_eye.x = (alpha * right_eye.x) + (beta * right_eye.y) + factor1;
	right_eye.y = temp1 + temp2 + factor2;
}


void prepareFaceRectangle(CvRect &face_rectg)
{
	int faceWidth=0;
	int faceHeight=0;

	float vert_left = 0.4;	//0.4     vert_left means the left vertical edge in the rectangle for cropping.
    float vert_right = 1.8;	//1.8     vert_right means the right vertical edge in the rectangle for cropping.
    float hrz_up = 0.5;	    //1.0     hrz_up means the upper horizontal edge in the rectangle for cropping.
    float hrz_down = 1.5;		//1.8     hrz_down means the lower horizontal edge in the rectangle for cropping.

	//Computes the horizontal distance between the eyes
	int d = abs(left_eye.x - right_eye.x);

	//compute values of face rectangle
	upper_left_corner.x = right_eye.x - (vert_left * d);
    upper_left_corner.y = right_eye.y - (hrz_up * d);
    faceWidth = vert_right * d;
    faceHeight = (hrz_up + hrz_down) * d;

	//create face rectangle
    face_rectg = cvRect(upper_left_corner.x, upper_left_corner.y, faceWidth, faceHeight);

	//fill lower_right_corner structure
	lower_right_corner.x = upper_left_corner.x + faceWidth;
	lower_right_corner.y = upper_left_corner.y + faceHeight;
}


void cropImage(IplImage* src,
                                IplImage* &dest,
                                CvRect &faceRegion)
{
	dest = cvCloneImage(src);							//dest = ROI.
	cvSetImageROI(dest, faceRegion);

	/*if(showCroppedImages==true)
	{
		cvNamedWindow( "cropped", CV_WINDOW_AUTOSIZE );
		cvShowImage( "cropped", ROI);
		if(saveCroppedImages==true)
			cvSaveImage("cropped.jpg",ROI);
	}*/
}

void scaleImage(IplImage* src, IplImage* &dest)
{
	cvResize(src, dest, CV_INTER_AREA);
}


void histEqualization(IplImage* src, IplImage* &dest)
{
	cvEqualizeHist(src, dest);
}

void convertFrom_BGR2GRAY(IplImage* src, IplImage* &dest)
{
	cvCvtColor( src, dest, CV_BGR2GRAY );
}

void computePositionOfEyesAfterCropping()
{
	//Left Eye
	left_eye.x = left_eye.x - upper_left_corner.x;
	left_eye.y = left_eye.y - upper_left_corner.y;

	//Right Eye
	right_eye.x = right_eye.x - upper_left_corner.x;
	right_eye.y = right_eye.y - upper_left_corner.y;

	lower_right_corner.x = lower_right_corner.x - upper_left_corner.x;
	lower_right_corner.y = lower_right_corner.y - upper_left_corner.y;

	cout << "Eye position after cropping: Left: " << left_eye.x << "," << left_eye.y << " Right: " << right_eye.x << "," << right_eye.y << endl;
}

void computePositionOfEyesAfterScaling()
{
	//Left Eye

	left_eye.x = (left_eye.x * scale_width) / lower_right_corner.x;
	left_eye.y = (left_eye.y * scale_height) / lower_right_corner.y;

	//Right Eye

	right_eye.x = (right_eye.x * scale_width) / lower_right_corner.x;
	right_eye.y = (right_eye.y * scale_height) / lower_right_corner.y;

	cout << "Eye position after scaling: Left: " << left_eye.x << "," << left_eye.y << " Right: " << right_eye.x << "," << right_eye.y << endl;
}

/**
 *	This function returns the normalized version (dest) of the src image.
 *	It will first rotate, then translate and finally scale the src image
 *	in order to obtain the normalized image.
**/
IplImage* normalizeImage(	IplImage* src,				//in:	original image
							CvPoint LEyePosin,	    	//in: 2D coordinates of Left eye (with respect to photographed person)
							CvPoint REyePosin,	    	//in: 2D coordinates of Right eye (with respect to photographed person)
							CvRect &faceRegion,   		//in: parameters delimiting the face rectangle.
							int scale_w,				//in: width of scaled image
							int scale_h,				//in: height of scaled image
							bool doRotation,			//in: flag to (de-)activate rotation
							bool doHistEq )				//in: flag to (de-)activate Histogram Equalization
{
	IplImage * dest;
	left_eye.x = LEyePosin.x;
  	left_eye.y = LEyePosin.y;
  	right_eye.x = REyePosin.x;
  	right_eye.y = REyePosin.y;

  	scale_width = scale_w;
  	scale_height = scale_h;

  	CvRect rect;
  	prepareFaceRectangle( rect );

	IplImage* rotated_img = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);

	if( doRotation == true )
	{
		CvPoint2D32f img_center;
		img_center.x = src->width/2;
		img_center.y = src->height/2;
		float angle = computeLineOfEyesAngle();

		rotateImage(src, rotated_img, angle, img_center);
		computePositionOfEyesAfterRotating(angle, &img_center);

		if( SHOW_IMAGES )
		{
			cvNamedWindow("rotated", 1);
			cvShowImage("rotated", rotated_img);
			//cvSaveImage("rotated_+3.jpg", rotated_img);
			cvWaitKey(0);
			cvDestroyWindow("rotated");
		}
	}
	else
	{
		cvCopy(src, rotated_img);
	}

	IplImage* cropped_img;
	cropImage(rotated_img, cropped_img, faceRegion);
	computePositionOfEyesAfterCropping();
	if( SHOW_IMAGES )
	{
		cvNamedWindow("cropped", 1);
		cvShowImage("cropped", cropped_img);
		//cvSaveImage("cropped.jpg", cropped_img);
		cvWaitKey(0);
		cvDestroyWindow("cropped");
	}

    IplImage * dest_img = cvCreateImage(cvSize(scale_width, scale_height), 8, 1);
	scaleImage(cropped_img, dest_img);
	computePositionOfEyesAfterScaling();

	if( SHOW_IMAGES )
	{
		cvNamedWindow("scaled", 1);
		cvShowImage("scaled", dest_img);
		//cvSaveImage("scaled.jpg", dest_img);
		cvWaitKey(0);
		cvDestroyWindow("scaled");
	}

	/*histEqualization(dest_img, dest_img);
	if( SHOW_IMAGES )
	{
		cvNamedWindow("histEqlzd", 1);
		cvShowImage("histEqlzd", dest_img);
		//cvSaveImage("histEqlzd.jpg", dest_img);
		cvWaitKey(0);
		cvDestroyWindow("histEqlzd");
	}*/

	cvReleaseImage(&rotated_img);
	cvReleaseImage(&cropped_img);

	return dest_img;
}

CvRect estimateFaceBoundaries( CvPoint &LEyePos, CvPoint &REyePos )
{
	CvRect faceRegion;
	CvPoint upperLeftCorner;
	int faceWidth;
	int faceHeight;

	float vert_left = 0.4;	//0.4     vert_left means the left vertical edge in the rectangle for cropping.
	float vert_right = 1.8;	//1.8     vert_right means the right vertical edge in the rectangle for cropping.
	float hrz_up = 0.5;		  //1.0     hrz_up means the upper horizontal edge in the rectangle for cropping.
	float hrz_down = 1.5;		//1.8     hrz_down means the lower horizontal edge in the rectangle for cropping.

	//Computes the horizontal distance between the eyes
	int d = abs(LEyePos.x - REyePos.x);

	//compute values of face rectangle
	upperLeftCorner.x = REyePos.x - (vert_left * d);
  	upperLeftCorner.y = REyePos.y - (hrz_up * d);

	faceWidth = vert_right * d;
	faceHeight = (hrz_up + hrz_down) * d;

	  //Create face rectangle
  	faceRegion = cvRect(upperLeftCorner.x, upperLeftCorner.y, faceWidth, faceHeight);

  	return faceRegion;
}

