# build
* a012nano 참고
```bash
# install docker
sudo apt-get install curl -y
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
sudo usermod -aG docker $USER
sudo reboot

# install pm2
curl -sL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt update && sudo apt install nodejs -y
sudo npm install pm2 -g
pm2 install pm2-logrotate && pm2 set pm2-logrotate:max_size 1K

# build jazzy
docker compose build jazzy

# hwt901
sudo vi /etc/udev/rules.d/99-hwt901.rules
KERNEL=="ttyUSB*", SUBSYSTEMS=="usb", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="imu", GROUP="dialout", MODE="0666"

# mwahrs
sudo vi /etc/udev/rules.d/99-mwahrs.rules
KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK="mwahrs"

# driver
sudo vi /etc/udev/rules.d/99-driver.rules
KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK="driver"

# rplidar s3
sudo vi /etc/udev/rules.d/99-rplidar.rules
KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK="rplidar"

sudo udevadm control --reload-rules
sudo udevadm trigger

# 두개의 USB ID가 같을때
lsusb -d 0403:6001 -v | grep -i serial
Bus 001 Device 015: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
  idProduct          0x6001 FT232 Serial (UART) IC
  iSerial                 3 A74XSC8M
Bus 001 Device 011: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
  idProduct          0x6001 FT232 Serial (UART) IC
  iSerial                 3 A75BR3OF

# realsense d435 안될때
lsusb -t
5000M 나오는지 확인

# install jazzy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt install ros-jazzy-desktop -y
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

```

## run
```bash
ros2 run lidar_filter lidar_filter
ros2 run z8015_mobile_bringup z8015_driver
ros2 launch sllidar_ros2 sllidar_s3_launch.py
ros2 launch mw_ahrs_ros2 mw_ahrs_launch.py
ros2 run sonar sen0591
ros2 launch witmotion_ros wt905_launch.py 
ros2 launch z8015_mobile_bringup z8015_driver

docker compose up bringiup
다른 x86 pc에서
docker compose up slam

mkdir -p ~/.docker
echo '{"experimental": "enabled"}' > ~/.docker/config.json
docker compose watch bringup
ssh bcc@...
docker compose watch slam
docker compose logs -f slam
docker compose restart slam
```