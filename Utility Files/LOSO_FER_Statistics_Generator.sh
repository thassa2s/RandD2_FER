#!/bin/bash
n_subjects=`expr 58 + 56 + 57`
n_expressions=6
n_per_expr_samples=`expr $n_subjects \\* 2`
n_images=`expr $n_expressions \\* $n_per_expr_samples`

n_per_expr_samples_old=`expr 57 \\* 2`
n_images_old=`expr $n_per_expr_samples_old \\* $n_expressions`

echo $n_subjects $n_expressions $n_per_expr_samples $n_images $n_per_expr_samples_old $n_images_old


echo -e "Total number of subjects: $n_subjects" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
echo -e "Total number of expressions: $n_expressions" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
echo -e "Total number of images per expression: $n_per_expr_samples" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
echo -e "Total number of images: $n_images" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
echo -e "\nTotal number of images per expression in older age-group: $n_per_expr_samples_old" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
echo -e "Total number of images in older age-group: $n_images_old" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
echo -e "\nConfusion matrix in percentage/100" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
echo -e "........................\n" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt


idx=0
while read v1 v2 v3 v4 v5 v6
do
p[0]=$(echo "scale=5;$v1 / $n_per_expr_samples_old" | bc)
p[1]=$(echo "scale=5;$v2 / $n_per_expr_samples_old" | bc)
p[2]=$(echo "scale=5;$v3 / $n_per_expr_samples_old" | bc)
p[3]=$(echo "scale=5;$v4 / $n_per_expr_samples_old" | bc)
p[4]=$(echo "scale=5;$v5 / $n_per_expr_samples_old" | bc)
p[5]=$(echo "scale=5;$v6 / $n_per_expr_samples_old" | bc)
echo ${p[@]}
echo -e ${p[@]} >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
save[$idx]=${p[$idx]}
echo ${save[$idx]}
idx=`expr $idx + 1`
done < /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/Confusion_Matrix_Overall.txt
overall_recognition_rate=$(echo "${save[0]} + ${save[1]} + ${save[2]} + ${save[3]} + ${save[4]} + ${save[5]}" | bc)
overall_recognition_rate=$(echo "scale=5;$overall_recognition_rate / $n_expressions" | bc)
echo $overall_recognition_rate

echo -e "\nOverall recognition rate: $overall_recognition_rate" >>  /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/FER_Statistics.txt
