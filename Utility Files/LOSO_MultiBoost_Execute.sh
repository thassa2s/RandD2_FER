#!/bin/bash

for i in {1..58}	
do
#train
/home/teenarahul/RandD2/SourceCode/MultiBoost/MultiBoost-Build/multiboost --configfile /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Young/$i/MultiBoost_Config_File_Train_Generated.txt
#test
/home/teenarahul/RandD2/SourceCode/MultiBoost/MultiBoost-Build/multiboost --configfile /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Young/$i/MultiBoost_Config_File_Test_Generated.txt
#confusion matrix
/home/teenarahul/RandD2/SourceCode/MultiBoost/MultiBoost-Build/multiboost --configfile /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Young/$i/MultiBoost_Config_File_CMatrix_Generated.txt
done
