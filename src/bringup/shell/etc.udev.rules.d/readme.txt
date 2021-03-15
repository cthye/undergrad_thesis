USB 与 tty 串口号绑定
如果只需要将外部的串口设备（如： 底盘控制的单片机、雷达、 IMU） 临时接入工控机，请参考方法 1，这
一设置断电重启后会失效；
如果需要对外部的串口设备长期绑定，请参考方法 2，这一方法设置完成后不得变更各串口的 USB 口接入
位置。
1.临时绑定
#拔掉所有 USB 串口,然后插入需要临时使用的串口
#在终端查看串口的设备号
ll /dev/ |grep ttyUSB*
#假设该设备号为/dev/ttyUSB0,给设备赋予可读写权限
sudo chmod 777 /dev/ttyUSB0
#修改对应*.launch 文件的设备号
#以~/catkin_ws/src/agvcar/agvcar_bringup/launch/minimal.launch 为例
cd ~/catkin_ws/src/agvcar/agvcar_bringup/launch/
#打开 minimal.launch 文件，修改 com_port 的取值为对应的设备号，例如
<param name="com_port" value="/dev/ttyUSB0"/>
2.长期绑定
#创建 rules 文件，我已经创建好了，直接打开编辑就行了
#打开/etc/udev/rules.d/99-agvcar-usb-serial.rules
每一行对应一个串口设备，例如
#lidar
ATTRS{idProduct}=="ea60", ATTRS{idVendor}=="10c4", KERNELS=="1-1.4.3",
SYMLINK+="lidar", MODE="0777"
#确定新插入的串口设备属性
#执行下面的命令， <devpath>参数请用具体的设备号取代（例如/dev/ttyUSB0）
udevadm info -a -p $(udevadm info -q path -n <devpath>)
#在输出的数据中从上到下找（如 KERNELS=="1-1.4.3:1.0"形式的项）下一个不
带“： ” 的 KERNELS 就是我们要找的，将对应的
ATTRS{idProduct}
ATTRS{idVendor}
KERNELS
的取值填入到 1 中的 rules 文件，其中 SYMLINK+中为这个设备的别名， MODE 设
置 0777 为读写权限
#将需要绑定的串口依次插入，然后按照上面的步骤进行绑定就行了
#注意，绑定串口取名如下：
运动控制底盘 SYMLINK+="agvcar"
激光雷达 SYMLINK+="lidar"
IMU SYMLINK+="imu"
#使设置生效
sudo udevadm control --reload-rules && sudo service udev restart && sudo
udevadm trigger
或者
sudo reboot
#测试
#分别打开底盘、雷达、 IMU 的*.launch 文件
#打开底盘*.launch 文件
~/catkin_ws/src/agvcar/agvcar_bringup/launch/minimal.launch
#确保底盘 com_port 取值如下
<param name="com_port" value="/dev/agvcar"/>
#打开雷达*.launch 文件
~/catkin_ws/src/talker/launch/ls01D.launch
#确保雷达 com_port 取值如下
<param name="com_port" value="/dev/lidar"/>
#打开 IMU 的*.launch 文件
~/catkin_ws/src/myserial/launch/imu_without_output_screen.launch
#确保雷达 com_port 取值如下
<param name="com_port" value="/dev/imu"/>
#依次启动底盘、雷达、 IMU
source ~/catkin_ws/devel/setup.bash
roslaunch agvcar_bringup minimal.launch
roslaunch talker ls01D.launch
roslaunch myserial imu_without_output_screen.launch
#用 rostopic echo 依次检验底盘、 雷达、 IMU 是否有数据
rostopic echo /odom
rostopic echo /scan
rostopic echo /imu
#如果数据正常输出，则绑定成功了。
