#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "brsu_msgs/FaceList.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include "brsu_srvs/ReturnString.h"

#include <fstream>
#include <ImageNormalizer.cpp>
#include <FaceRegionEstimator.cpp>

namespace enc = sensor_msgs::image_encodings;

void expression_callback(const std_msgs::String& msg)
{
  std::cout << "Expression identified as: " << msg.data << std::endl;
}

int main( int argc, char **argv ) 
{
  // Initialize ROS Node
  ros::init ( argc, argv, "fer_test" );
  ros::NodeHandle nh;

  ros::Subscriber expLabel_sub;
  ros::Publisher faceList_pub;

  // Advertise image messages on a topic
  faceList_pub = nh.advertise<brsu_msgs::FaceList>( "/brsu_face_recognition/face_list", 1 );
  // Listen for label of expression and setup callback
  expLabel_sub = nh.subscribe( "/brsu_facial_expression/expression", 1, expression_callback);
  //Or, should we use the get_expression service?? But this service returns the previously recognized expression.
  //recognizeExpressionServer = nh.advertiseService( "/brsu_facial_expression/get_expression", &FacialExpressionNode::recognizeExpression, this );
	
  // Read the file and publish to FaceList
  /*unsigned int coordinate;
  ifstream infile;
  infile.open("test.txt");
  infile >> coordinate;
  std::cout << "Coordinate: " << coordinate << std::endl;*/
  //Create sensor_msg
  cv_bridge::CvImage cvImage;
  //cvImage.image = cv::imread("/home/teenarahul/Passbild.jpg", CV_LOAD_IMAGE_COLOR);
  IplImage *iplImg =  cvLoadImage("/home/teenarahul/Passbild.jpg", CV_LOAD_IMAGE_COLOR);
 // if ( !cvImage.image.data && iplImg == NULL )
  if ( !iplImg )
  {
    std::cout << "Failed to load image. " << std::endl;
  }
  else
  {
    bool verbose = true;
    CvPoint lEyePos; 
    lEyePos.x = 1903;
    lEyePos.y = 1375;
    CvPoint rEyePos;
    rEyePos.x = 1221;
    rEyePos.y = 1408;

    //get information about the boundaries of the face.
    FaceRegionEstimator *faceRegionEst = new FaceRegionEstimator(verbose);
    FaceRegion faceRegion = faceRegionEst->estimateFaceBoundaries(lEyePos, rEyePos);

    // Convert iplImage to grey scale.
    if (verbose)
            cout << endl << "converting iplImage to grey scale.";
    
    IplImage *greyImage = cvCreateImage(cvGetSize(iplImg), IPL_DEPTH_8U, 1);
    cvCvtColor(iplImg, greyImage, CV_RGB2GRAY);
    
    //Normalize image

    ImageNormalizer *normalizer = new ImageNormalizer(200, 200, verbose);
    IplImage *normalizedImage = cvCreateImage(cvSize(200, 200), 8, 1);
    normalizer->normalizeImage(greyImage, normalizedImage, lEyePos, rEyePos, faceRegion, false, false);
    cout << endl << "after normalizing image." << endl;

    cvCvtColor(greyImage, iplImg, CV_GRAY2RGB);

    std::cout << "Creating the FaceList message" << std::endl;
    //Set header
    //Set encoding
    brsu_msgs::FaceList face_list;
    brsu_msgs::Face face;

    face_list.num_faces = 1;
    face.leftEyeCenterX = normalizer->getLeftEyePosition().x;
    face.leftEyeCenterY = normalizer->getLeftEyePosition().y;
    face.rightEyeCenterX = normalizer->getRightEyePosition().x;
    face.rightEyeCenterY = normalizer->getRightEyePosition().y;
  
    std::cout << "Eye positions after normalization: Left: " << face.leftEyeCenterX << "," << face.leftEyeCenterY << " Right: " << face.rightEyeCenterX << "," << face.rightEyeCenterY << std::endl;
    cvImage.encoding = enc::BGR8;
    try {
      cv::Mat tmpImg(iplImg);
      cvImage.image = tmpImg;
      cvImage.toImageMsg(face.image);
    }catch( cv_bridge::Exception& e ) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return 0;
    }
    face_list.faces.push_back(face);

    //publish face list
    faceList_pub.publish(face_list);
  }

  // Spin ...
  //ros::spin ();
  double rate = 50; //50Hz
  ros::Rate loop_rate(rate);
  while (ros::ok()) 
  {
    ros::spinOnce();
    //read data from file; convert to sensor_msg; and publish
    
    //invoke the service or leave it to call back.. better option: service
  }
  // ... until done
  return 0;
}
