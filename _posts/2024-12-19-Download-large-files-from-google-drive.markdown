---
layout: default
title:  "Download Dataset script"
date:   2024-12-19
categories: Anything
---


# Download Dataset script
**_Guide to run_**
**_for ubuntu_**

```shell
chmod +x download_files.sh
```

**download_files.sh**
```shell
#!/bin/bash

if ! command -v wget &> /dev/null
then
    echo "Error: wget is not installed. Please install wget and try again."
    exit 1
fi

usage() {
    echo "Usage: $0 <target_directory> <comma_separated_gdrive_links>"
    exit 1
}

if [ "$#" -ne 2 ]; then
    usage
fi

TARGET_DIR="$1"
LINKS="$2"

if [ ! -d "$TARGET_DIR" ]; then
    echo "Creating target directory: $TARGET_DIR"
    mkdir -p "$TARGET_DIR"
    if [ $? -ne 0 ]; then
        echo "Error: Failed to create target directory."
        exit 1
    fi
fi

cd "$TARGET_DIR" || exit

IFS=',' read -r -a LINK_ARRAY <<< "$LINKS"

for LINK in "${LINK_ARRAY[@]}"; do
    echo "Starting download for: $LINK"
    wget --show-progress "$LINK"
    if [ $? -eq 0 ]; then
        echo "Download complete: $LINK"
    else
        echo "Error: Failed to download $LINK"
    fi
    echo "--------------------------------------------------"
done

echo "All downloads completed."
```

**Example**
```shell
./download_files.sh /path/to/save "https://drive.google.com/file/d/FILE_ID1,https://drive.google.com/file/d/FILE_ID2"
```