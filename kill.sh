#!/bin/bash

if [[ $(ps | grep roscore) ]]; then
    killall rosmaster
fi

if [[ $(ps | grep momo_x64) ]]; then
    killall momo_x64
fi

if [[ $(ps | grep momo_arm64) ]]; then
    killall momo_arm64
fi

if [[ $(ps | grep socat) ]]; then
    killall socat
fi

if [[ $(ps | grep cmder.run) ]]; then
    killall cmder.run
fi

if [[ $(ps | grep accel_cmd) ]]; then
    killall accel_cmd
fi
