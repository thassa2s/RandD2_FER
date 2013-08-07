/*
 * ImageNormalizer.h
 *
 *  Created on: Aug 7, 2013
 *      Author: teenarahul
 */

#ifndef IMAGENORMALIZER_H_
#define IMAGENORMALIZER_H_

CvRect estimateFaceBoundaries( CvPoint &LEyePos, CvPoint &REyePos );
IplImage* normalizeImage(	IplImage* src,				//in:	original image
							CvPoint LEyePosin,	    	//in: 2D coordinates of Left eye (with respect to photographed person)
							CvPoint REyePosin,	    	//in: 2D coordinates of Right eye (with respect to photographed person)
							CvRect &faceRegion,   		//in: parameters delimiting the face rectangle.
							int scale_w,				//in: width of scaled image
							int scale_h,				//in: height of scaled image
							bool doRotation,			//in: flag to (de-)activate rotation
							bool doHistEq );				//in: flag to (de-)activate Histogram Equalization


#endif /* IMAGENORMALIZER_H_ */
