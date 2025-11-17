1) Download ROS2 from the documentation

2) Create a virtual envirorment:

sudo apt update  
sudo apt upgrade -y  
sudo apt install -y python3-pip python3-venv git build-essential  
python3 -m venv ~/ros2_env  
source ~/ros2_env/bin/activate  

4) Install necessary librieries
   
pip install --upgrade pip setuptools wheel  
pip install "numpy<2" scipy opencv-python-headless  
pip install rclpy  
sudo apt install -y ros-humble-cv-bridge  
pip install ultralytics deep_sort_realtime  

6) Sorce and building
   
source ~/ros2_env/bin/activate  
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
colcon build --symlink-install  
source install/setup.bash  
ros2 run tracking_pkg tracking_node

OTHERS:
- command to export\import Linux  
wsl --export UbuntuSafe C:\Users\samue\ubuntu-backup.tar  
wsl --import UbuntuSafe D:\WSL\Ubuntu C:\Users\samue\ubuntu-backup.tar --version 2  


