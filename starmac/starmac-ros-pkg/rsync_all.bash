#!/bin/bash
if [ $# -ne 1 ]
then
	echo "Usage: rsync_all.bash hostname"
	echo "e.g.: rsync_all.bash pelican1"
	exit 65
fi

DEST_HOST=${1}
for stack in starmac_{common,flyer,demos,vicon,robots,sensors}
do
	./rsync_stack_to_pelican.bash ${stack} ${DEST_HOST}
	if [ $? -ne 0 ]
	then
		echo "Error, aborting."
		exit 1
	fi
done