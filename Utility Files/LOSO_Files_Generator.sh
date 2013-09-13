#!/bin/bash
#cat Metadata_Faces_Pictures_Manual_Eye_Pos_Young.txt| awk '{split($0, a,  "_y_"); print a[1] }' | uniq > young_faces_person_id;
#cat Metadata_Faces_Pictures_Manual_Eye_Pos_Middle.txt| awk '{split($0, a,  "_m_"); print a[1] }' | uniq > middle_age_faces_person_id;
#cat Metadata_Faces_Pictures_Manual_Eye_Pos_Old.txt| awk '{split($0, a,  "_o_"); print a[1] }' | uniq > old_faces_person_id;
underscore='_';
i=1;
while read line
do
#echo $line
mkdir ./LOSO_Old/$i
while read line_metadata
do 
pattern=$line$underscore;
if [ `echo $line_metadata | grep -c $pattern` -gt 0 ]
then
echo $line_metadata >> ./LOSO_Old/$i/Metadata_test.txt
else
echo $line_metadata >> ./LOSO_Old/$i/Metadata_train.txt
fi
done < Metadata_Faces_Pictures_Manual_Eye_Pos_Old.txt
i=`expr $i + 1`	
done < old_faces_person_id
