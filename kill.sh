#!/bin/bash

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

if [[ $(ps | grep roscore) ]]; then
    killall rosmaster
fi

if [[ $(ps | grep rostopic) ]]; then
    killall rostopic
fi

if [[ $(ps | grep segway_rmp_node) ]]; then
    killall segway_rmp_node
fi

if [[ $(ps | grep sora_x64.run) ]]; then
    killall sora_x64.run
fi

if [[ $(ps | grep sora_arm64.run) ]]; then
    killall sora_arm64.run
fi


if [[ $(ps | grep joy_node) ]]; then
    killall joy_node
fi
