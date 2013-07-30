/*
 * TrainingDataGenerator.cpp
 *
 *  Created on: Jul 30, 2013
 *      Author: teenarahul
 */
#include <GFExtractorOnImage.cpp>

ofstream training_data_outfile;
ofstream test_data_outfile;

bool getFeatureVector( string full_image_filename, string feature_descriptor, vector1Df &feature_vector_out )
{
	IplImage* image = cvLoadImage( full_image_filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if ( !image )
    {
    	cout << "Error opening image: " << full_image_filename << "." << endl;
    	return false;
    }
    if ( strcmp( feature_descriptor.c_str(), "gabor" ) == 0 )
    {
    	GFExtractorOnImage *gabor_feature_extractor = new GFExtractorOnImage();
    	gabor_feature_extractor->extractFeaturesOnImage(image, feature_vector_out);
    	//cout << "Feature vector size: " << feature_vector_out.size() <<"\t Computed size: " << image->height * image->width * 40 << endl;
    }
    return true;
}

void append_to_training_data_file( vector1Df &feature_vector, string emotion_label )
{
	vector1Df::const_iterator it;
	for( it = feature_vector.begin(); it < feature_vector.end(); it++ )
	{
		training_data_outfile << *it << "\t";
	}
	training_data_outfile << emotion_label << endl;
}

void append_to_test_data_file( vector1Df &feature_vector )
{
	vector1Df::const_iterator it;
	for( it = feature_vector.begin(); it < feature_vector.end(); it++ )
	{
		test_data_outfile << *it << "\t";
	}
	test_data_outfile << endl;
}

void generateTrainingData( string folder_path, string full_image_list_filename, string feature_descriptor, bool create_test_data_file_flag )
{
	ifstream infile;
	infile.open( full_image_list_filename.c_str() );
	if ( !infile.is_open() )
	{
		cout << "Error opening file: " << full_image_list_filename <<"." << endl;
		return;
	}
	string full_training_data_filename = folder_path +  string( "Training_Data_" ) + feature_descriptor + string( ".txt" );
	training_data_outfile.open( full_training_data_filename.c_str(), ios::out );
	if ( !training_data_outfile.is_open() )
	{
		cout << "Error opening training data file: " << full_training_data_filename << "." << endl;
		return;
	}

	if ( create_test_data_file_flag )
	{
	    string full_test_data_filename = folder_path + string( "Test_Data_from_Training_Data_" ) + feature_descriptor + string( ".txt" );
	    test_data_outfile.open( full_test_data_filename.c_str(), ios::out );
	    if ( !test_data_outfile.is_open() )
	    {
			cout << "Error opening test data file: " << full_test_data_filename << "." << endl;
			return;
	    }
	}

    string image_filename;
    string full_image_filename;
    string emotion_label;

    infile >> image_filename;

    unsigned int no_of_images_processed = 0;
	vector1Df feature_vector;

    while ( infile.peek() != EOF )
    {
        infile >> emotion_label;

    	full_image_filename = folder_path + image_filename;
    	feature_vector.clear();

    	if ( getFeatureVector( full_image_filename, feature_descriptor, feature_vector ) )
        {
        	append_to_training_data_file( feature_vector, emotion_label );
        	if ( create_test_data_file_flag )
        	{
            	append_to_test_data_file( feature_vector );
        	}
        	no_of_images_processed++;
        }
        infile >> image_filename;
    }
    cout << "Total number of images processed: " << no_of_images_processed << "." << endl;
    infile.close();
    training_data_outfile.close();
    test_data_outfile.close();
}

int main( int argc, char *argv[] )
{
	if ( argc < 5 )
	{
		cout << "Expected command line arguments: <full path to directory> <image_list_filename> <feature descriptor> <0/1>" << endl;
		cout << "Possible values for feature descriptor: gabor, lbp, ldp" << endl;
		cout << "Give 1 if a test data file should be created from the same images. Give 0, otherwise" << endl;
		return -1;
	}
	string folder_path = argv[1];
	string full_image_list_filename = folder_path + argv[2];
	string feature_descriptor = argv[3];
	bool create_test_data_file_flag =  argv[4][0] - '0';
    generateTrainingData( folder_path, full_image_list_filename, feature_descriptor, create_test_data_file_flag );

	return 0;
}


