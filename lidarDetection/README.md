# lidar Detection

`foxy,ubuntu 20.04`

Lidar点云障碍物检测,并发布障碍物位置话题.
![](src/lidarDetection/static/rviz.png)
## dependency
```bash
sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions libpcl-dev
git submodule update --init --recursive
wget http://fishros.com/install -O fishros && . fishros # 选择安装rosdep

cd <workspace>
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro foxy
```

## build
```bash
colcon build --symlink-install --paths .
colcon build --symlink-install --packages-select sensing_msgs
```
![](src/lidarDetection/static/rqt.png)
## dynamic reconfigure
在开启obstacle_detector_node节点前,运行以下命令启动动态参数配置界面:
```bash
./lidarDetection/scripts/reconfigure.sh
```
ROS2 foxy没有像ROS1 的dynamic_reconfigure package,所以这里使用了一个自定义的动态参数配置界面,可以动态调整点云预处理和聚类的相关参数,但是调整的手感感觉稀烂,有时候参数会设置失败.目前在`go2.yaml`的参数是一组效果还可以的参数.
## attention


