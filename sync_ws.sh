#!/bin/bash
# ===========================================================================
# Author      : Steven Gallego
# Description : Synchronizes a list of folders to a remote folder at REMOTE_IP.
#               Used to synchronize ROS workspace packages.
# ===========================================================================

# ANSI color codes
RED='\033[0;31m'
NC='\033[0m' # No Color

# ===========================================================================
# Configurable Parameters
# ===========================================================================
ROS_WS_NAME=mini_bream_ws
REMOTE_USER=pi
REMOTE_WS_FOLDER=/home/$REMOTE_USER/$ROS_WS_NAME/src/


REMOTE_IP=192.168.168.100

# Define the list of folders to sync
LOCAL_FOLDERS=("backseat" "frontseat" "mission_planner")
PATH_TO_LOCAL_FOLDERS=./src/autonomous_catamaran_ws/src/

echo ===============================
echo Synchronizing Folders:
printf '%s ' "${LOCAL_FOLDERS[@]}"
printf '\n===============================\n'

printf '\n===============================\n'
echo Remote Path: $REMOTE_WS_FOLDER
echo Remote User: $REMOTE_USER
echo Remote IP  : $REMOTE_IP
echo ===============================

# Loop through each folder
for folder in "${LOCAL_FOLDERS[@]}"; do
    if ! rsync -avz --exclude={'logs','.git','__pycache__','unused'} --progress "$PATH_TO_LOCAL_FOLDERS/$folder" "$REMOTE_USER@$REMOTE_IP:$REMOTE_WS_FOLDER" 2>&1; then
        echo -e "${RED}Error occurred while synchronizing $folder.${NC}"
    fi
done

echo ===============================
echo Synchronization completed at
echo "   $(date)"
echo ===============================