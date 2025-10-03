

## usage
```bash


# install 일출몰시간대
sud apt udpate
sudo apt install python3-pip
pip install astral


# install smach
cd ~/ws
mkdir 3rdparty
git clone https://github.com/gabrielsr/standalone-smach.git standalone-smach && \
sed -i '247s/isAlive()/is_alive()/g' standalone-smach/smach/concurrence.py && \
cd ~/ws/3rdparty/standalone-smach
pip install --use-pep517 .

cd ~/ws
colcon build --packages-select accessory_client accessory_msgs


cp start_image_capture_server.sh start_accessory_client.sh
vi start_accessory_client.sh
#!/bin/bash
source /home/zeta/ws/install/setup.bash
sleep 30
ros2 run accessory_client accessory_fsm
pm2 start /home/zeta/ws/start_accessory_client.sh --name accessory_client --env HOME=/home/zeta --env PATH=$PATH  <== 꼭이렇게 하세요 
pm2 save
```