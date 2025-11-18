TOKEN: 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


1) Download ROS2 from the documentation


2) Install necessary librieries:  

python3 -m pip install --user ultralytics deep_sort_realtime opencv-python-headless scipy  
pip install --upgrade pip setuptools wheel  
python3 -m pip install --user --force-reinstall numpy==1.24.4 
pip install scipy opencv-python-headless  
pip install rclpy  
pip install ultralytics deep_sort_realtime  
pip install image_publisher  
sudo apt update  
sudo apt install ros-humble-image-publisher  
sudo apt install -y ros-humble-cv-bridge  
sudo apt install ros-dev-tools  

3) Create a virtual envirorment:

sudo apt update  
sudo apt upgrade -y  
sudo apt install -y python3-pip python3-venv git build-essential  
python3 -m venv ~/ros2_env  
source ~/ros2_env/bin/activate  
pip install --ignore-installed numpy==1.24.4  
pip install ultralytics deep-sort-realtime  
sudo apt install python3-cv-bridge 
pip install rclpy ament-package opencv-python  
pip install Cython  


4) Sorce and building   

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc   
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc  
source ~/ros2_env/bin/activate    
cd ~/ros2_ws   
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



