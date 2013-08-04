/*
 * RecognitionOutputAnalyser.cpp
 *
 *  Created on: Aug 2, 2013
 *      Author: teenarahul
 */

#include <fstream>
#include <iostream>
#include <string.h>
#include <boost/tokenizer.hpp>

using namespace std;

#define FACES 1

unsigned int confusion_matrix_count[ 7 ][ 7 ]; //7 expressions
double confusion_matrix_percentage[ 7 ][ 7 ]; //7 expressions
unsigned int expression_count[ 7 ];
double expression_percentage[ 7 ];
unsigned int count_correct_recognition = 0;
unsigned int total_image_count = 0;
double expression_wise_recognition_rate[ 7 ];
double overall_recognition_rate = 0.0;

unsigned int confusion_matrix_count_age_group_wise[ 3 ][ 7 ][ 7 ]; //3 age-groups, max 7 expressions
double confusion_matrix_percentage_age_group_wise[ 3 ][ 7 ][ 7 ];
unsigned int expression_count_age_group_wise[ 3 ][ 7 ];
unsigned int age_group_count[ 3 ];
unsigned int count_correct_recognition_age_group_wise[ 3 ];
double age_group_wise_recognition_rate[ 3 ];

const string statistics_output_file_name = "FER_Statistics.txt";

enum expressions { UNKNOWN, ANGER, DISGUST, FEAR, JOY, NEUTRAL, SADNESS, SURPRISE } expr_actual_enum, expr_recognized_enum;

enum age_groups { INVALID, YOUNG, MIDDLE, OLD } age_group_enum;

expressions get_expr_enum_const( string expr_label )
{
	if ( strcmp( expr_label.c_str(), "anger" ) == 0 )
		return ANGER;
	if ( strcmp( expr_label.c_str(), "disgust" ) == 0 )
		return DISGUST;
	if ( strcmp( expr_label.c_str(), "fear" ) == 0 )
		return FEAR;
	if ( strcmp( expr_label.c_str(), "joy" ) == 0 )
		return JOY;
	if ( strcmp( expr_label.c_str(), "neutral" ) == 0 )
		return NEUTRAL;
	if ( strcmp( expr_label.c_str(), "sadness" ) == 0 )
		return SADNESS;
	if ( strcmp( expr_label.c_str(), "surprise" ) == 0 )
		return SURPRISE;
	return UNKNOWN;
}

string get_expr_enum_string( expressions enum_const )
{
	switch ( enum_const )
	{
	case ANGER		: 	return string( "anger" );
	case DISGUST	: 	return string( "disgust" );
	case FEAR		:	return string( "fear" );
	case JOY		: 	return string( "joy" );
	case NEUTRAL	:	return string( "neutral" );
	case SADNESS	:	return string( "sadness" );
	case SURPRISE	:	return string( "surprise" );
	default			:	return string( "unknown" );
	}
}

age_groups get_age_group_enum_const( char age_grp )
{
	switch ( age_grp )
	{
	case 'y' : return YOUNG;
	case 'm' : return MIDDLE;
	case 'o' : return OLD;
	default  : return INVALID;
	}
}

string get_age_group_enum_string( age_groups enum_const )
{
	switch ( enum_const )
	{
	case YOUNG		: 	return string( "young" );
	case MIDDLE		: 	return string( "middle-aged" );
	case OLD		:	return string( "old" );
	default			:	return string( "invalid" );
	}
}

age_groups find_age_group( string image_name )
{
	int i = 0;
	char ch = 'n';
	boost::char_separator<char> separator("_");
	boost::tokenizer< boost::char_separator<char> > string_tokens(image_name, separator);
    for( boost::tokenizer< boost::char_separator<char> >::iterator it = string_tokens.begin(); it != string_tokens.end(); it++ )
    {
		i++;
		if ( i == 2 )
		{
			ch = it.current_token().at( 0 );
			break;
		}
	}
    return get_age_group_enum_const( ch );
}

void initialise_global_vars()
{
	count_correct_recognition = total_image_count = overall_recognition_rate = 0;
    for ( int k = 0; k < 7; k++ )
    {
    	expression_count[ k ] = expression_percentage[ k ] = expression_wise_recognition_rate[ k ] = 0;
    	for ( int j = 0; j < 7; j++ )
    		confusion_matrix_count[ k ][ j ] = confusion_matrix_percentage[ k ][ j ] = 0;
    }
    for ( int k = 0; k < 3; k++ )
    {
    	age_group_count[ k ] = count_correct_recognition_age_group_wise[ k ] = age_group_wise_recognition_rate[ k ] = 0;
    	for ( int j = 0; j< 7; j++ )
    	{
    		expression_count_age_group_wise[ k ][ j ] = 0;
    		for ( int i = 0; i < 7; i++ )
    			confusion_matrix_count_age_group_wise[ k ][ j ][ i ] = confusion_matrix_percentage_age_group_wise[ k ][ j ][ i ] = 0;
    	}
    }
}
void update_statistics()
{
	 for ( int k = 0; k < 7; k++ )
	 {
	    for ( int j = 0; j < 7; j++ )
	    {
	        confusion_matrix_percentage[ k ][ j ] = confusion_matrix_count[ k ][ j ] * 100/( ( double ) expression_count[ k ] );
		}
	    expression_percentage[ k ] = expression_count[ k ] * 100 / ( ( double ) total_image_count );
	    expression_wise_recognition_rate[ k ] = confusion_matrix_count[ k ][ k ] * 100 / ( ( double ) expression_count[ k ] );
	}
	overall_recognition_rate = count_correct_recognition * 100 / ( ( double ) total_image_count );

	if ( FACES )
	{
		for ( int k = 0; k < 3; k++ )
		{
		  	for ( int j = 0; j< 7; j++ )
		  	{
		  		for ( int i = 0; i < 7; i++ )
		  		{
		  			confusion_matrix_percentage_age_group_wise[ k ][ j ][ i ] = confusion_matrix_count_age_group_wise[ k ][ j ][ i ] * 100/( ( double ) expression_count_age_group_wise[ k ][ j ] );
		  		}
		  		count_correct_recognition_age_group_wise[ k ] += confusion_matrix_count_age_group_wise[ k ][ j ][ j ];
		  	}
		   	age_group_wise_recognition_rate[ k ] = count_correct_recognition_age_group_wise[ k ] * 100 / ( ( double ) age_group_count[ k ] );
		}
	}
}

void write_statistics_to_file( string folder_path )
{
	string full_filename = folder_path + statistics_output_file_name;
	ofstream outfile;
	outfile.open( full_filename.c_str() );
	if ( !outfile.is_open() )
	{
		cout << "Error opening statistics file: " << full_filename << endl;
		return;
	}
	outfile << endl << "Confusion_matrix count " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_expr_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	  	for ( int j = 0; j < 7; j++ )
	   	{
	        outfile << '\t' << confusion_matrix_count[ k ][ j ];
	  	}
	    outfile << endl;
	}
	outfile << endl << "Confusion_matrix percentage " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_expr_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
     	for ( int j = 0; j < 7; j++ )
	   	{
	        outfile << "\t " << confusion_matrix_percentage[ k ][ j ];
	   	}
	  	outfile << endl;
	}
	outfile << endl << "Expression count " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_expr_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	   	outfile << '\t' << expression_count[ k ] << endl;
	}
	outfile << endl << "Expression percentage " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_expr_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	 	outfile << '\t' << expression_percentage[ k ] << endl;
	}
	outfile << endl << "Expression wise recognition rate " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_expr_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	 	outfile << '\t' << expression_wise_recognition_rate[ k ] << endl;
	}
	outfile << endl << "Total images: " << total_image_count << endl;
	outfile << endl << "Success count: " << count_correct_recognition << endl;
	outfile << endl << "Overall recognition rate: " << overall_recognition_rate << endl;

	if ( FACES )
	{
		outfile << endl << "Age-group-wise statistics" << endl;
		outfile << "************************" << endl;

		outfile << endl << "Confusion_matrix count: Age-group-wise" << endl << endl;
		for (int k = 0; k < 3; k++ )
		{
			outfile << get_age_group_enum_string( static_cast< age_groups >( k + 1 ) ) << "\t\t";
			for ( int j = 0; j < 7; j++ )
			{
				outfile << get_expr_enum_string( static_cast< expressions >( j + 1 ) ) << '\t';
				for ( int i = 0; i < 7; i++ )
				{
					outfile << '\t' << confusion_matrix_count_age_group_wise[ k ][ j ][i];
				}
				outfile << endl << "\t\t";
			}
			outfile << endl;
		}
		outfile << endl << "Confusion_matrix percentage: Age-group-wise" << endl << endl;
		for ( int k = 0; k < 3; k++ )
		{
			outfile << get_age_group_enum_string( static_cast< age_groups >( k + 1 ) ) << "\t\t";
	     	for ( int j = 0; j < 7; j++ )
		   	{
				outfile << get_expr_enum_string( static_cast< expressions >( j + 1 ) ) << '\t';
				for ( int i = 0; i < 7; i++ )
				{
					outfile << '\t' << confusion_matrix_percentage_age_group_wise[ k ][ j ][i];
				}
				outfile << endl << "\t\t";
		   	}
			outfile << endl;
		}
		outfile << "Total number of images per age-group" << endl << endl;
		for ( int k = 0; k < 3; k++ )
		{
			outfile << get_age_group_enum_string( static_cast< age_groups >( k + 1 ) ) << "\t\t" << age_group_count[ k ] << endl;
		}
		outfile << endl << "Total number of images correctly recognized per age-group" << endl << endl;
		for ( int k = 0; k < 3; k++ )
		{
			outfile << get_age_group_enum_string( static_cast< age_groups >( k + 1 ) ) << "\t\t" << count_correct_recognition_age_group_wise[ k ] << endl;
		}
		outfile << endl << "Age-group-wise recognition rate" << endl << endl;
		for ( int k = 0; k < 3; k++ )
		{
			outfile << get_age_group_enum_string( static_cast< age_groups >( k + 1 ) ) << "\t\t" << age_group_wise_recognition_rate[ k ] << endl;
		}
	}
	outfile.close();
}

int main( int argc, char *argv[] )
{
    if ( argc < 3 )
    {
    	cout << "Provide the following details of file that contains the FER results." << endl;
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

    string line;
    getline( infile, line ); //First line is comment
    string imgname;

    initialise_global_vars();

    while ( infile.peek() != EOF )
    {
    	string actual_expr = "neutral";
    	string recognized_expr = "joy";

    	infile >> imgname;
    	infile >> actual_expr;
    	infile >> recognized_expr;

    	expr_actual_enum = get_expr_enum_const( actual_expr );
    	expr_recognized_enum = get_expr_enum_const( recognized_expr );

    	if ( FACES )
    	{
    		age_group_enum = find_age_group( imgname );
    		confusion_matrix_count_age_group_wise[ age_group_enum - 1 ][ expr_actual_enum - 1 ][ expr_recognized_enum - 1 ]++;
    		expression_count_age_group_wise[ age_group_enum - 1 ][ expr_actual_enum - 1 ]++;
    		age_group_count[ age_group_enum - 1 ]++;
    	}

    	confusion_matrix_count[ expr_actual_enum - 1 ][ expr_recognized_enum - 1 ]++;
    	expression_count[ expr_actual_enum - 1 ]++;
    	total_image_count++;

    	if ( strcmp( actual_expr.c_str(), recognized_expr.c_str()) == 0 )
        {
        	count_correct_recognition++;
        }
    }

    update_statistics();

    write_statistics_to_file( folder_path );

    infile.close();
}

