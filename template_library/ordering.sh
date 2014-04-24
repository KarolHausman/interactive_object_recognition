#!/bin/bash

#clear

#echo "script starts now!"


    

for D in *; do
    if [ -d "${D}" ]; then
	cd ${D}

	result=${PWD##*/}
	echo $result
	i=0
	for file in *; 
	do
	    if [ $file = "ordering.sh" ] || [ -d "${file}" ]; then
		 echo $file
	    else
		 let "i += 1" 
		 echo $i	 	
		 mv "$file"  "${i}_${result}.pcd"
	    fi
	done
	cd ..
    fi
done
