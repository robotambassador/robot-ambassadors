#!/bin/bash
# usage: rsync_stack_to_pelican.bsh <stackname> <destination hostname>
# eg: rsync_stack_to_pelican.bsh starmac_flyer pelican2
if [ $# -ne 2 ]
then
	echo "Usage: rsync_stack_to_pelican.bash <stackname> <destination hostname>"
	echo "e.g.: rsync_stack_to_pelican.bash starmac_flyer pelican2"
	exit 65
fi

STACK=$1
DEST_HOST=$2

SOURCE=$(rosstack find ${STACK})
if [ $? -ne 0 ]
then
	echo "Could not find stack '${STACK}', aborting."
	exit 1
fi
echo SOURCE=${SOURCE}
DEST=starmac@${DEST_HOST}:ros/starmac-ros-pkg/${STACK}/
echo DEST=${DEST}

rsync -avz --exclude-from=${SOURCE}/.rsync-exclude ${SOURCE}/ ${DEST}
exit $?
