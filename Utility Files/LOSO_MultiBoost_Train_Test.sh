#!/bin/bash
<<comment
for i in {57..57}	
do
filename="./LOSO_Old/"
filename=$filename$i"/MultiBoost_Config_File_Train_Generated.txt"
echo -e "stronglearner AdaBoostMH \nlearnertype MultiThresholdStumpLearner \nfileformat simple \nclassend \nverbose 5" >> $filename
echo -e "train /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/"$i"/Feature_Data_Metadata_train.txt_lbp_u2.txt 100" >> $filename
echo -e "outputinfo /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/"$i"/training_output_old_loso_"$i"_100iters.txt e01ham" >> $filename
echo -e "shypname /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/"$i"/LBPu2_model_old_loso_"$i"_100iters.xml" >> $filename
done
comment

<<comment1
for i in {1..1}	
do
filename="./LOSO_Middle_Age/"
filename=$filename$i"/MultiBoost_Config_File_Test_Generated.txt"
echo -e "stronglearner AdaBoostMH \nlearnertype MultiThresholdStumpLearner \nfileformat simple \nclassend \nverbose 5" >> $filename
echo -e "test /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Middle_Age/"$i"/Feature_Data_Metadata_test.txt_lbp_u2.txt /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Middle_Age/"$i"/LBPu2_model_middle_loso_"$i"_100iters.xml 100 /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Middle_Age/"$i"/Test_Results_LBPu2_Middle_loso_"$i"_100iters.txt" >> $filename
done
comment1

for i in {1..57}	
do
filename="./LOSO_Old/"
filename=$filename$i"/MultiBoost_Config_File_CMatrix_Generated.txt"
echo -e "stronglearner AdaBoostMH \nlearnertype MultiThresholdStumpLearner \nfileformat simple \nclassend \nverbose 5" >> $filename
echo -e "cmatrix /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/"$i"/Feature_Data_Metadata_test.txt_lbp_u2.txt /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/"$i"/LBPu2_model_old_loso_"$i"_100iters.xml /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/"$i"/LBPu2_CMatrix_Old_loso_"$i"_100iters.txt" >> $filename
done

