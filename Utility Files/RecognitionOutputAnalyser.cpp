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

unsigned int confusion_matrix_count[ 7 ][ 7 ]; //7 expressions
double confusion_matrix_percentage[ 7 ][ 7 ]; //7 expressions
unsigned int expression_count[ 7 ];
double expression_percentage[ 7 ];
unsigned int count_correct_recognition = 0;
unsigned int total_image_count = 0;
double expression_wise_recognition_rate[ 7 ];
double overall_recognition_rate = 0.0;

const string statistics_output_file_name = "FER_Statistics.txt";

enum expressions { UNKNOWN, ANGER, DISGUST, FEAR, JOY, NEUTRAL, SADNESS, SURPRISE } expr_actual_enum, expr_recognized_enum;

expressions get_enum_const( string expr_label )
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

string get_enum_string( expressions enum_const )
{
	if ( enum_const == ANGER )
		return string( "anger" );
	if ( enum_const == DISGUST )
		return string( "disgust" );
	if ( enum_const == FEAR )
		return string( "fear" );
	if ( enum_const == JOY )
		return string( "joy" );
	if ( enum_const == NEUTRAL )
		return string( "neutral" );
	if ( enum_const == SADNESS )
		return string( "sadness" );
	if ( enum_const == SURPRISE )
		return string( "surprise" );
	return string( "unknown" );

}

void initialise_global_vars()
{
	count_correct_recognition = total_image_count = overall_recognition_rate = 0;
    for ( int k = 0; k < 7; k++ )
    {
    	expression_count[ k ] = expression_percentage[ k ] = expression_wise_recognition_rate[ k ] =0;
    	for ( int j = 0; j < 7; j++ )
    		confusion_matrix_count[ k ][ j ] = confusion_matrix_percentage[ k ][ j ] = 0;
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
		outfile << get_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	  	for ( int j = 0; j < 7; j++ )
	   	{
	        outfile << '\t' << confusion_matrix_count[ k ][ j ];
	  	}
	    outfile << endl;
	}
	outfile << endl << "Confusion_matrix percentage " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
     	for ( int j = 0; j < 7; j++ )
	   	{
	        outfile << "\t " << confusion_matrix_percentage[ k ][ j ];
	   	}
	  	outfile << endl;
	}
	outfile << endl << "Expression count " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	   	outfile << '\t' << expression_count[ k ] << endl;
	}
	outfile << endl << "Expression percentage " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	 	outfile << '\t' << expression_percentage[ k ] << endl;
	}
	outfile << endl << "Expression wise recognition rate " << endl << endl;
	for ( int k = 0; k < 7; k++ )
	{
		outfile << get_enum_string( static_cast< expressions >( k + 1 ) ) << '\t';
	 	outfile << '\t' << expression_wise_recognition_rate[ k ] << endl;
	}
	outfile << endl << "Total images: " << total_image_count << endl;
	outfile << endl << "Success count: " << count_correct_recognition << endl;
	outfile << endl << "Overall recognition rate: " << overall_recognition_rate << endl;
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

    	expr_actual_enum = get_enum_const( actual_expr );
    	expr_recognized_enum = get_enum_const( recognized_expr );

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

