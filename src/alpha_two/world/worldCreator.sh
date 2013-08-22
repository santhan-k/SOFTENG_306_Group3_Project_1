#!/bin/bash

if [ $# -eq 2 ]
then 
	grass=20
	#rm test4321.txt
 
	for (( c=0; c<= $1-1; c++ ))
	do
		if [ $2 -eq 1 ]
		then
			x=$(((($RANDOM+$RANDOM) % 28) + 1))
			y=$(((($RANDOM+$RANDOM) % 28) + 1))
		elif [ $2 -eq 2 ]
		then
			x=$(((($RANDOM+$RANDOM) % 28) + 1))
			y=-$(((($RANDOM+$RANDOM) % 28) + 1))
		elif [ $2 -eq 3 ]
		then
			x=-$(((($RANDOM+$RANDOM) % 28) + 1))
			y=-$(((($RANDOM+$RANDOM) % 28) + 1))
		elif [ $2 -eq 4 ]
		then
			x=-$(((($RANDOM+$RANDOM) % 28) + 1))
			y=$(((($RANDOM+$RANDOM) % 28) + 1))
		fi
		angle=$(((($RANDOM+$RANDOM) % 358) + 1))
		echo $x $y
		echo "myRobot( pose [" $x $y "0 "$angle" ] name \"r"$c"\" color \"red\" )" >>test4321.txt
		sheepNumber=$((12+$c))
		gnome-terminal -x bash worldCreator_assistant2.sh $sheepNumber $x $y $angle
		
	done
	for (( i=0; i<$grass; i++ ))
	do
		x=$(((($RANDOM+$RANDOM) % 28) +1))
		y=$(((($RANDOM+$RANDOM) % 28) +1))
		echo "myGrass( pose [ " $x $y "0 0 ] name \"g"$i"\" color \"green\" )" >>test4321.txt
	done
	
	for (( i=0; i<$grass; i++ ))
	do
		x=$(((($RANDOM+$RANDOM) % 28) +1))
		y=-$(((($RANDOM+$RANDOM) % 28) +1))
		echo "myGrass( pose [ " $x $y "0 0 ] name \"g"$i"\" color \"green\" )" >>test4321.txt
	done

	for (( i=0; i<$grass; i++ ))
	do
		x=-$(((($RANDOM+$RANDOM) % 28) +1))
		y=-$(((($RANDOM+$RANDOM) % 28) +1))
		echo "myGrass( pose [ " $x $y "0 0 ] name \"g"$i"\" color \"green\" )" >>test4321.txt
	done

	for (( i=0; i<$grass; i++ ))
	do
		x=-$(((($RANDOM+$RANDOM) % 28) +1))
		y=$(((($RANDOM+$RANDOM) % 28) +1))
		echo "myGrass( pose [ " $x $y "0 0 ] name \"g"$i"\" color \"green\" )" >>test4321.txt
	done
	
	
	cat myworld.world test4321.txt > newWorld.world
	rm test4321.txt
	rosrun stage stageros newWorld.world &
			
	gnome-terminal -x bash worldCreator_assistant.sh $1
	

	echo 
else 
	echo "Usage: worldCreator <numSheep> <FieldNumber>"
	
fi


