#!/bin/bash

# ROS 2の環境設定を読み込む
source /opt/ros/humble/setup.bash
if [ -f "/home/gritcat/ekagaku_gritcat/grit_ws/install/setup.bash" ]; then
    source /home/gritcat/ekagaku_gritcat/grit_ws/install/setup.bash
fi
# Pythonスクリプトを実行する
exec /usr/bin/python3 /home/gritcat/ekagaku_gritcat/app/main.py