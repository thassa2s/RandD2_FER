#!/bin/bash
carray1=(0 0 0 0 0 0)
carray2=(0 0 0 0 0 0)
carray3=(0 0 0 0 0 0)
carray4=(0 0 0 0 0 0)
carray5=(0 0 0 0 0 0)
carray6=(0 0 0 0 0 0)
p=0
for j in {1..57} 
do
p=`expr $p + 1`
n=0
fname="LBPu2_CMatrix_Old_loso_"$j"_100iters.txt"
while read v0 v1 v2 v3 v4 v5 v6 
do
if [ `echo $n` -gt 0 ]
then
echo $v1 $v2 $v3 $v4 $v5 $v6
case $n in 
	1)
	carray1[0]=`expr ${carray1[0]} + $v1`
	carray1[1]=`expr ${carray1[1]} + $v2`
	carray1[2]=`expr ${carray1[2]} + $v3`
	carray1[3]=`expr ${carray1[3]} + $v4`
	carray1[4]=`expr ${carray1[4]} + $v5`
	carray1[5]=`expr ${carray1[5]} + $v6`;;
	2)
	carray2[0]=`expr ${carray2[0]} + $v1`
	carray2[1]=`expr ${carray2[1]} + $v2`
	carray2[2]=`expr ${carray2[2]} + $v3`
	carray2[3]=`expr ${carray2[3]} + $v4`
	carray2[4]=`expr ${carray2[4]} + $v5`
	carray2[5]=`expr ${carray2[5]} + $v6`;;
	3)
	carray3[0]=`expr ${carray3[0]} + $v1`
	carray3[1]=`expr ${carray3[1]} + $v2`
	carray3[2]=`expr ${carray3[2]} + $v3`
	carray3[3]=`expr ${carray3[3]} + $v4`
	carray3[4]=`expr ${carray3[4]} + $v5`
	carray3[5]=`expr ${carray3[5]} + $v6`;;
	4)
	carray4[0]=`expr ${carray4[0]} + $v1`
	carray4[1]=`expr ${carray4[1]} + $v2`
	carray4[2]=`expr ${carray4[2]} + $v3`
	carray4[3]=`expr ${carray4[3]} + $v4`
	carray4[4]=`expr ${carray4[4]} + $v5`
	carray4[5]=`expr ${carray4[5]} + $v6`;;
	5)
	carray5[0]=`expr ${carray5[0]} + $v1`
	carray5[1]=`expr ${carray5[1]} + $v2`
	carray5[2]=`expr ${carray5[2]} + $v3`
	carray5[3]=`expr ${carray5[3]} + $v4`
	carray5[4]=`expr ${carray5[4]} + $v5`
	carray5[5]=`expr ${carray5[5]} + $v6`;;
	6)
	carray6[0]=`expr ${carray6[0]} + $v1`
	carray6[1]=`expr ${carray6[1]} + $v2`
	carray6[2]=`expr ${carray6[2]} + $v3`
	carray6[3]=`expr ${carray6[3]} + $v4`
	carray6[4]=`expr ${carray6[4]} + $v5`
	carray6[5]=`expr ${carray6[5]} + $v6`;;
esac
#arrname="carray"$n
#{$arrname}[0] = `expr ${$arrname[0]} + $v1`
#echo ${$arrname[0]};
fi
n=`expr $n + 1`
done < /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/$j/$fname
echo "---------"
done
echo ${carray1[@]} >> /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/Confusion_Matrix_Overall.txt
echo ${carray2[@]} >> /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/Confusion_Matrix_Overall.txt
echo ${carray3[@]} >> /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/Confusion_Matrix_Overall.txt
echo ${carray4[@]} >> /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/Confusion_Matrix_Overall.txt
echo ${carray5[@]} >> /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/Confusion_Matrix_Overall.txt
echo ${carray6[@]} >> /home/teenarahul/RandD2/ImageDatabase/Faces-collection/pictures/LOSO_Old/Confusion_Matrix_Overall.txt

echo $p
<<comment



	0	1	2	3	4	5
0	2	0	0	0	0	0
1	0	1	0	0	1	0
2	0	1	0	0	0	1
3	0	0	0	2	0	0
4	0	0	0	0	2	0
5	2	0	0	0	0	0

comment
