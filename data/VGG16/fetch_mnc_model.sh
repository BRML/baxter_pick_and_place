#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd )"
cd $DIR

FILE=mnc_model.caffemodel.h5
URL=https://www.brml.tum.de/BaxterCollision/caffe_models/$FILE
CHECKSUM=ac116844f66aefe29587214272054668

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

wget $URL -O $FILE

echo "Done. Please run this command again to verify that checksum = $CHECKSUM."