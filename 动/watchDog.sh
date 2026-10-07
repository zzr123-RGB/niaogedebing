#!/bin/bash

# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/girt/.local/lib/python3.10/site-packages/onnxruntime/capi
# source /opt/ros/humble/setup.bash
sleep 1

mkdir -p /home/girt/123456/build
cd /home/girt/123456/build || exit 1

cmake .. || exit 1
make     || exit 1

# Ctrl+C 时杀掉当前进程组所有进程
trap 'echo "========"; kill 0' INT TERM

./Darts_Aimer
#./DartTest
