#!/bin/bash
source ../../../devel/setup.bash
echo $1
nohup rosrun alpha_two R0 &

for (( i=0; i<$1; i++ ))
do
	sheepNumber=$((6+$i))
	echo $sheepNumber		
	gnome-terminal -x bash worldCreator_assistant2.sh $sheepNumber
done
