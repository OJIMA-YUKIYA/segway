#!/bin/bash

cd libsegwayrmp
if [ ! -e lib ]; then
    mkdir lib
fi
if [ $(arch) = 'arm64' ]; then
    cp ./ftd2xx/linux/arm64/libftd2xx.a ./lib/
elif [ $(arch) = 'x86_64' ]; then
    cp ./ftd2xx/linux/x64/libftd2xx.a ./lib/
fi


make
cd .. && catkin_make
