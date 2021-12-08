#!/bin/bash

cd libsegwayrmp
if [ ! -e lib ]; then
    mkdir lib
fi
if [ $USER = 'tristar' ]; then
    cp ./ftd2xx/linux/arm64/libftd2xx.a ./lib/
elif [ $USER = 'ojima' ]; then
    cp ./ftd2xx/linux/x64/libftd2xx.a ./lib/
fi


make
cd .. && catkin_make
