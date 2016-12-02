#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd )"
cd $DIR

FOLDER=faster_rcnn_models
FILE=VGG16_faster_rcnn_final.caffemodel
URL=http://www.cs.berkeley.edu/~rbg/faster-rcnn-data/$FOLDER.tgz
CHECKSUM=6fe7a7bbb30752006bf05c8181be825b

if [ -f $FILE ]; then
  echo "File already exists. Checking md5..."
  os=`uname -s`
  if [ "$os" = "Linux" ]; then
    checksum=`md5sum $FILE | awk '{ print $1 }'`
  elif [ "$os" = "Darwin" ]; then
    checksum=`cat $FILE | md5`
  fi
  if [ "$checksum" = "$CHECKSUM" ]; then
    echo "Checksum is correct. No need to download."
    exit 0
  else
    echo "Checksum is incorrect. Need to download again."
  fi
fi

echo "Downloading Faster R-CNN model..."

wget $URL -O $FOLDER.tgz

echo "Unzipping..."

tar zxvf $FOLDER.tgz

mv $FOLDER/$FILE $FILE
rm -rf $FOLDER
rm -rf $FOLDER.tgz

echo "Done. Please run this command again to verify that checksum = $CHECKSUM."
