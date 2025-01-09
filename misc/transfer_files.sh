#!/bin/bash
# Define the source and destination directories
SOURCE_DIR="../output"
DEST_DIR="/media/aidan/Aidan_Thumb/PiTrack"

# Function to transfer files
transfer_files() {
    echo "Starting file transfer..."
    if cp -r "$SOURCE_DIR"/* "$DEST_DIR"; then
        echo "Files transferred successfully."
    else
        echo "File transfer failed. Check permissions and paths."
        exit 1
    fi
}

# Main logic to check if drive is mounted and transfer files
while true; do
    echo "Checking if drive is mounted..."
    if mount | grep -q "/media/aidan/Aidan_Thumb"; then
        echo "Drive mounted, starting file transfer..."
        transfer_files
        break  # Exit loop after transferring files
    else
        echo "Drive not mounted. Please plug in the drive."
        sleep 5
    fi
done

