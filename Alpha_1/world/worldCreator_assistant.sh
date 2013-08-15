source ../../../devel/setup.bash
echo $1
nohup rosrun Alpha_1 R0 &

#for (( i=0; i<$1; i++ ))
#do
#	sheepNumber=$((5+$i))
#	echo $sheepNumber		
#	#nohup rosrun Alpha_1 R1 $sheepNumber &
#done
