TOKEN: 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


1) Download ROS2 from the documentation

2) Create a virtual envirorment:

sudo apt update  
sudo apt upgrade -y  
sudo apt install -y python3-pip python3-venv git build-essential  
python3 -m venv ~/ros2_env  
source ~/ros2_env/bin/activate
pip install empy==3.3.4  
pip install catkin_pkg  
pip install lark-parser  
pip install ultralytics  

4) Install necessary librieries

python3 -m pip install --user "numpy<2" ultralytics deep_sort_realtime opencv-python-headless scipy  
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

- github upload
git add .
git commit -m "Descrivi qui cosa hai modificato"
git push



