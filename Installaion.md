

# Installation
```
# Install pip 
sudo apt install python3-pip
sudo apt install git

# Install Robotic Toolbox
pip install roboticstoolbox-python


# Make the workspace and clone armer and armer_msgs packages
mkdir -p ~/armer_ws/src && cd ~/armer_ws/src 
git clone https://github.com/qcr/armer.git && git clone https://github.com/qcr/armer_msgs 

# Install all required packages
pip install -r ~/armer_ws/src/armer/requirements.txt
cd .. && rosdep install --from-paths src --ignore-src -r -y 

# Make and source the workspace (modified by luia2)
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

echo "source ~/armer_ws/devel/setup.bash" >> ~/.bashrc 
source ~/armer_ws/devel/setup.bash
echo "Installation complete!"
```