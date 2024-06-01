# xsugv_c6

C6 ROS

1. 拷贝 xsugv_c6 源码到 /data/ros 目录


2. 在Xavier 脚本 ~/.bashrc 中配置
  
    source /data/ros/xsugv_c6/devel/setup.bash

    export XSUGV_ID=xxx   # xxx为车辆编号

    export XSUGV_NAME=C6_$XSUGV_ID  # 车辆名称

    export XSUGV_IP=172.24.$XSUGV_ID.93  # 底盘控制器IP

    export ROS_IP=172.24.$XSUGV_ID.101  # ROS控制器IP


3. 在 Startup Applications 中添加开机启动项， 启动脚本 /data/ros/xsugv_c6/src/xsugv/script/startup.sh

