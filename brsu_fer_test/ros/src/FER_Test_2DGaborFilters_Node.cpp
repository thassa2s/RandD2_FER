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
  cvImage.image = cv::imread("/home/teenarahul/Passbild.jpg", CV_LOAD_IMAGE_COLOR);
  if ( !cvImage.image.data )
  {
    std::cout << "Failed to load image. " << std::endl;
  }
  else
  {
    std::cout << "Creating the FaceList message" << std::endl;
    //Set header
    //Set encoding
    brsu_msgs::FaceList face_list;
    brsu_msgs::Face face;

    face_list.num_faces = 1;
    face.leftEyeCenterX = 1221;
    face.leftEyeCenterY = 1408;
    face.rightEyeCenterX = 1903;
    face.rightEyeCenterY = 1375;

    try {
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
