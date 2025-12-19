# lidar Detection

`foxy,ubuntu 20.04`

![](src/lidarDetection/static/rviz.png)
```bash
git submodule update --init --recursive
wget http://fishros.com/install -O fishros && . fishros # 选择安装rosdep

cd <workspace>
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro foxy
```

依赖安装:
```bash
sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions libpcl-dev
```

```bash
colcon build --symlink-install --paths .
colcon build --symlink-install --packages-select sensing_msgs
```
![](src/lidarDetection/static/rqt.png)

## attention
// 输入的球相对位置话题,目前使用的是obstacle_information_to_baselink的话题,将所有障碍物当作"球"处理了,后续需要调整成真正的球检测话题
input_ball_topic_ = this->declare_parameter<std::string>("input_ball_topic", "/obstacle_information_to_baselink");