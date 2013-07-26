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

ros::Subscriber expLabel_sub;
ros::Publisher faceList_pub;

namespace enc = sensor_msgs::image_encodings;
bool expressionLabelReceived = true;
string recognized_expression_label = "";

//Actual values for the following variables are obtained from the corresponding ROS parameters
bool verbose = false;
bool normalize_image = false;
bool save_normalized_image = false;
string folder_path = ".";
string metadata_filename = "MetaData_FACES_Database.txt";
string normalized_images_foldername = "normalized/";
string normalized_images_metadata_filename = "MetaData_Normalized_Images.txt";
string FER_results_filename_prefix = "FER_Gabor_Results_";

void update_global_vars_from_ROS_params( ros::NodeHandle nh )
{
    if ( !nh.getParam("/brsu_fer_test/verbose", verbose) )
    {
    	ROS_ERROR("Parameter verbose is not specified.");
    }
    if ( !nh.getParam("/brsu_fer_test/normalize_image", normalize_image) )
    {
    	ROS_ERROR("Parameter normalize_image is not specified.");
    }
    if ( !nh.getParam("/brsu_fer_test/save_normalized_image", save_normalized_image) )
    {
    	ROS_ERROR("Parameter save_normalized_image is not specified.");
    }
    if ( !nh.getParam("/brsu_fer_test/folder_path", folder_path) )
    {
    	ROS_ERROR("Parameter folder_path is not specified.");
    }
    if ( !nh.getParam("/brsu_fer_test/metadata_filename", metadata_filename) )
    {
    	ROS_ERROR("Parameter metadata_filename is not specified.");
    }
    if ( !nh.getParam("/brsu_fer_test/normalized_image_foldername", normalized_images_foldername) )
    {
    	ROS_ERROR("Parameter normalized_image_foldername is not specified.");
    }
    if ( !nh.getParam("/brsu_fer_test/normalized_images_metadata_filename", normalized_images_metadata_filename) )
    {
    	ROS_ERROR("Parameter normalized_images_metadata_filename is not specified.");
    }
    if ( verbose )
    {
		cout << "**********************************************************" << endl;
    	cout << "Updated global variables are as follows: " << endl;
    	cout << "verbose: " << verbose << endl;
    	cout << "normalize_image: " << normalize_image << endl;
    	cout << "save_normalized_image: " << save_normalized_image << endl;
    	cout << "folder_path: " << folder_path << endl;
    	cout << "metadata_filename: " << metadata_filename << endl;
    	cout << "normalized_images_foldername: " << normalized_images_foldername << endl;
    	cout << "normalized_images_metadata_filename: " << normalized_images_metadata_filename << endl;
    }
}

bool read_next_image_details_from_metadatafile( ifstream &metadata_file, string &image_filename_out, CvPoint &l_eyepos_out, CvPoint &r_eyepos_out, string &expression_label_out )
{
	if ( metadata_file.peek() == EOF )
	{
		return false;
	}
	metadata_file >> image_filename_out;
	metadata_file >> l_eyepos_out.x >> l_eyepos_out.y >> r_eyepos_out.x >> r_eyepos_out.y;
	metadata_file >> expression_label_out;
	char ch;
	metadata_file.get(ch); //to catch the newline character (\n) at the end of each line
	if ( verbose )
	{
		cout << "**********************************************************" << endl;
		cout << "Details of next image read from metadata file: " << endl;
		cout << "Image file name: " << image_filename_out << endl;
		cout << "Left eye position: " << l_eyepos_out.x << ", " << l_eyepos_out.y << endl;
		cout << "Right eye position: " << r_eyepos_out.x << ", " << r_eyepos_out.y << endl;
		cout << "Expression label: " << expression_label_out << endl;
	}
	return true;
}

void append_to_metadata_file( string file, string image_filename, CvPoint l_eyepos, CvPoint r_eyepos, string expression_label="" )
{
	ofstream outfile;
	outfile.open(file.c_str(), ios::app);
    if ( !outfile )
    {
    	cout << "Could not open new metadata file " << file << " for writing." << endl;
    	return;
    }
    if ( outfile.tellp() != 0 )
    {
    	outfile << endl;
    }
    else
    {
    	outfile << "** Format: <image file name> <left eye position: X> < left eye position: Y> <right eye position: X> < right eye position: Y> <expression label>" << endl;
    }
    outfile << image_filename << " ";
    outfile << l_eyepos.x << " " << l_eyepos.y << " " << r_eyepos.x << " " << r_eyepos.y << " ";
    outfile << expression_label;
    outfile.close();
}

IplImage* getIplImage( string folder_path, string image_filename, CvPoint l_eyepos_in, CvPoint r_eyepos_in, string expression_label, CvPoint &l_eyepos_out, CvPoint &r_eyepos_out)
{
	string full_filename = folder_path + image_filename;
	IplImage *iplImg =  cvLoadImage(full_filename.c_str(), CV_LOAD_IMAGE_COLOR);
	if ( !iplImg )
	{
		cout << "Failed to load image. " << endl;
		return NULL;
	}
	if ( !normalize_image )
	{
		l_eyepos_out = l_eyepos_in;
		r_eyepos_out = r_eyepos_in;
	    return iplImg;
	}
	FaceRegionEstimator *faceRegionEst = new FaceRegionEstimator(verbose);
	FaceRegion faceRegion = faceRegionEst->estimateFaceBoundaries(l_eyepos_in, r_eyepos_in);
    if (verbose)
    {
      	cout << "Converting IplImage to grayscale..." << endl;
    }
    IplImage *greyImage = cvCreateImage(cvGetSize(iplImg), IPL_DEPTH_8U, 1);
    cvCvtColor(iplImg, greyImage, CV_RGB2GRAY);
    if (verbose)
    {
     	cout << "Normalizing image..." << endl;
    }
   	ImageNormalizer *normalizer = new ImageNormalizer(200, 200, verbose);
   	IplImage *normalizedImage = cvCreateImage(cvSize(200, 200), 8, 1);
   	normalizer->normalizeImage(greyImage, normalizedImage, l_eyepos_in, r_eyepos_in, faceRegion, false, false);
   	IplImage *normalizedColorImg = cvCreateImage(cvGetSize(normalizedImage), IPL_DEPTH_8U, 3);
   	cvCvtColor(normalizedImage, normalizedColorImg, CV_GRAY2RGB);
   	l_eyepos_out = normalizer->getLeftEyePosition();
   	r_eyepos_out = normalizer->getRightEyePosition();
    if ( verbose )
    {
    	cout << endl << "Eye position after normalization: Left: " << l_eyepos_out.x << "," << l_eyepos_out.y << " Right: " << r_eyepos_out.x << "," << r_eyepos_out.y << endl;
    }
   	if ( save_normalized_image )
   	{   if ( verbose )
   		{
   			cout << "Saving normalized image... " << endl;
   		}
   		string normalized_image_file = folder_path + string("../") + normalized_images_foldername + image_filename;
   		if ( !cvSaveImage( normalized_image_file.c_str(), normalizedColorImg ) )
   		{
   			cout << "Normalized image not saved. " << endl;
   		}
   		else
   		{
   			string new_metadata_file = folder_path + string("../") + normalized_images_foldername + normalized_images_metadata_filename;
   			append_to_metadata_file( new_metadata_file, image_filename, l_eyepos_out, r_eyepos_out, expression_label );
   		}
   	}
   	return normalizedColorImg;
}

bool constructCvImage( string folder_path, string image_filename, CvPoint l_eyepos_in, CvPoint r_eyepos_in, string expression_label, cv_bridge::CvImage &cvImage_out, CvPoint &l_eyepos_out,  CvPoint &r_eyepos_out)
{
	IplImage *iplImg = getIplImage( folder_path, image_filename, l_eyepos_in, r_eyepos_in, expression_label, l_eyepos_out, r_eyepos_out);
	if ( !iplImg )
	{
		cout << "Failed to get IplImage. " << endl;
		return false;
	}
    cvImage_out.encoding = enc::BGR8;
    try
    {
    	cv::Mat tmpImg(iplImg);
        cvImage_out.image = tmpImg;
    }catch( cv_bridge::Exception& e )
    {
    	ROS_ERROR("cv_bridge exception: %s", e.what());
    	return false;
    }
    return true;
}

void construct_and_publish_FaceList_msg(cv_bridge::CvImage cvImage, CvPoint l_eyepos, CvPoint r_eyepos)
{
    brsu_msgs::FaceList face_list;
    brsu_msgs::Face face;
    if ( verbose )
    {
    	cout << endl << "Creating FaceList message" << std::endl;
    }
    face_list.num_faces = 1;
    face.leftEyeCenterX = l_eyepos.x;
    face.leftEyeCenterY = l_eyepos.y;
    face.rightEyeCenterX = r_eyepos.x;
    face.rightEyeCenterY = r_eyepos.y;
    try
    {
    	cvImage.toImageMsg(face.image);
    }catch( cv_bridge::Exception& e )
    {
    	ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    face_list.faces.push_back(face);
    if ( verbose )
    {
    	cout << "Publishing FaceList message" << std::endl;
    }
    faceList_pub.publish(face_list);
}

void expression_callback(const std_msgs::String& msg)
{
	recognized_expression_label = msg.data;
	if ( verbose )
	{
		cout << "Expression identified as: " << recognized_expression_label << std::endl;
	}
	expressionLabelReceived = true;
}

string get_date_time_string()
{
	time_t curr_time = time(0);
    char time_char_arr[27];
    strftime(time_char_arr, 27, "Date_%Y_%m_%d_Time_%H_%M", gmtime(&curr_time));
    return string(time_char_arr);
}

void append_to_results_file( string results_file, string image_filename, string expr_label )
{
	ofstream outfile;
	outfile.open(results_file.c_str(), std::ofstream::app);
    if ( !outfile )
    {
    	cout << "Could not open results file." << endl;
    	return;
    }
    if ( outfile.tellp() != 0 )
    {
    	outfile << endl;
    }
    else
    {
    	cout << "** Format: <Image_file_name> <correct_expression> <recognized_expression>" << endl;
    }
    outfile << image_filename << " " << expr_label << " " << recognized_expression_label;
    outfile.close();
}

int main( int argc, char **argv )
{
	ros::init ( argc, argv, "brsu_fer_test" );
	ros::NodeHandle nh;

	faceList_pub = nh.advertise<brsu_msgs::FaceList>( "/brsu_face_recognition/face_list", 1 );
	expLabel_sub = nh.subscribe( "/brsu_facial_expression/expression", 1, expression_callback);
	//Or, should we use the get_expression service?? But this service returns the previously recognized expression.
	//recognizeExpressionServer = nh.advertiseService( "/brsu_facial_expression/get_expression", &FacialExpressionNode::recognizeExpression, this );

    update_global_vars_from_ROS_params(nh);

    cv_bridge::CvImage cvImage;
    CvPoint l_eyepos, r_eyepos;
    CvPoint l_eyepos_new, r_eyepos_new;
    string image_filename;
    string expression_label;

    ifstream infile;
    string metadata_full_path = folder_path + metadata_filename;
    infile.open(metadata_full_path.c_str());

    //First line in the metadata file is reserved for comments.
    string line_read;
    getline( infile, line_read );

    string results_file = folder_path + string("../Results/") + FER_results_filename_prefix + get_date_time_string() + string(".txt");

    double rate = 50; //50Hz
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
    	ros::spinOnce();
       	if ( expressionLabelReceived )
        {
       		if ( !image_filename.empty() )
       		{
       			append_to_results_file( results_file, image_filename, expression_label);
       		}
       		if ( read_next_image_details_from_metadatafile( infile, image_filename, l_eyepos, r_eyepos, expression_label ) )
       	    {
        		if ( constructCvImage( folder_path, image_filename, l_eyepos, r_eyepos, expression_label, cvImage, l_eyepos_new, r_eyepos_new ) )
        		{
        			construct_and_publish_FaceList_msg( cvImage, l_eyepos_new, r_eyepos_new );
        			expressionLabelReceived = false;
        		}
        		else
        		{
        			cout << "Error processing " << image_filename << endl;
        		}
        	}
       		else
       		{
       			if ( verbose )
       			{
       				cout << "**********************************************************" << endl;
       				cout << "No next image details available." << endl;
       			}
       			cout << "Exiting brsu_fer_test node..." << endl;
       			infile.close();
       			return 0;
       		}
        	//invoke the service or leave it to call back.. better option: service
        }
    }
    infile.close();
    return 0;
}
