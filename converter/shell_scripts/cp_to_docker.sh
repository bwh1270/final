#! /bin/bash

# move file from NUC local to NUC docker
TARGET=$1
OPTION=$2
CONTAINER_NAME=$(docker container ls | grep -i carrot | tail -n1 | awk '{print $NF}')

if [ $OPTION == "r" ]
then
	echo "Start moving the file from NUC Docker container as $CONTAINER_NAMEto NUC Local"	
	docker cp $CONTAINER_NAME:/root/uav_ws/src/$TARGET ./

else
	echo "Start moving the file from NUC Local to NUC Docker container as $CONTAINER_NAME"	
	docker cp $TARGET $CONTAINER_NAME:/root/uav_ws/src/
fi
