#! /bin/bash

# move file from laptop to NUC
TARGET=$1
OPTION=$2

if [[ $OPTION == 'r' ]]
then
	scp falconblack@falconblack-nuc.local:/home/falconblack/ICUAS23/CARROT/$TARGET ./

else
	scp $TARGET falconblack@falconblack-nuc.local:/home/falconblack/ICUAS23/CARROT
	ssh falconblack@falconblack-nuc.local
fi
