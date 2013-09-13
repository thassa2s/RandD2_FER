#!/bin/bash

for i in {1..57}	
do
/home/teenarahul/ComputerVision/workspace1/EyePositionCapture/Debug/EyePositionCapture /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/$i/ Metadata_test.txt lbp_u2 0
/home/teenarahul/ComputerVision/workspace1/EyePositionCapture/Debug/EyePositionCapture /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/$i/ Metadata_train.txt lbp_u2 0
done
