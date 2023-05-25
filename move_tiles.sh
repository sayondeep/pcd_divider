#!/bin/bash

# Read the file containing directory paths (one path per line)
while IFS= read -r directory; do
    # Move all files from the current directory to the destination directory
    cp "$directory" ./tiled_pcd/
done < pcd_tiles.txt
