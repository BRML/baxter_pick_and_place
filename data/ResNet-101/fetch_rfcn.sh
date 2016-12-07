#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd )"
cd $DIR

FILE=rfcn_ohem_490000.caffemodel
URL=https://www.brml.tum.de/BaxterCollision/caffe_models/$FILE
CHECKSUM=7dec80528eea0dcbab5b06182ee58f1e

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

echo "Downloading R-FCN model..."

wget $URL -O $FILE

echo "Done. Please run this command again to verify that checksum = $CHECKSUM."
