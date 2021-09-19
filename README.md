# abb_kinectv2_control\\
Provides a functional skeleton-tracking based controller for an abb irb 1600 robot simulation\\

Der folgende Abschnitt beschreibt die Installation der für das Projekt benötigten Softwarepakete. Vor der Installation muss eine ROS-Version auf dem PC installiert sein. Die Installation ist unter der Full-Desktop-Version von ROS-Noetic installiert. Der verwendete Computer nutzt Ubuntu 20.04. Die anfängliche Installation von libfreenect2, TurboJPEG, OpenGL, CUDA und VAAPI sind von der Website \citep{Pung} mit entsprechendem Reporsitory entnommen.\\
	
# Die folgenden Schritte müssen für die Installation abgearbeitet werden:\\

# Das Git von Libfreenect2 downloaden:

cd catkin_ws 
cd src
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2 

# Herunterladen der upgrade deb files:

cd depends; ./download_debs_trusty.sh

I# nstallieren der build tools:

sudo apt-get install build-essential cmake pkg-config

I# nstallieren von libusb - die Version muss neuer 1.0.20 sein:

sudo apt-get install libusb-1.0-0-dev

# Installieren von TurboJPEG:

sudo apt-get install libturbojpeg0-dev

# Installieren von OpenGL für Intel CPU:

sudo apt-get install beignet-dev

# Die Installation von CUDA erfolgt im ersten Schritt über die Seite von NVIDIA:

https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64 &Distribution=Ubuntu&target_version=20.04&target_type=deb_local

# Die folgenden Schritte sind auf der NVIDIA Seite wiederzufinden:
	
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/ x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.4.1/local_installers/ cuda-repo-ubuntu2004-11-4-local_11.4.1-470.57.02-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-4-local_11.4.1-470.57.02-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-4-local/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda

# Installieren von VAAPI:

sudo apt-get install libva-dev libjpeg-dev

# Installieren von OpenNI2:

sudo apt-get install libopenni2-dev

# Build durchführen:

mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install

# CMake benötigt eine Spezifikation:

cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2

# Setzen der udev Regeln für den Gerätezugriff:

sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

# In separaten Fenster wird Roscore gestartet:

roscore

# An dieser Stelle muss ein build- und bash-Befehl durchgeführt werden: 

cd catkin_ws	
catkin_make_isolated
source devel_isolated/setup.bash

# Testprogramm in $catkin\_ws/src/libfreenect2/build$ laufen lassen :

./bin/Protonect

# Als nächstes wird NiTE2 installiert. Hierzu müssen folgende Schritte durchgeführt werden:

cd PATHTO/HandTrack/src
wget http://jaist.dl.sourceforge.net/project/roboticslab/External/nite /NiTE-Linux-x64-2.2.tar.bz2
tar xjvf NiTE-Linux-x64-2.2.tar.bz2 && rm NiTE-Linux-x64-2.2.tar.bz2
cd NiTE-Linux-x64-2.2
sudo ./install.sh
cd  ..
cp libfreenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so NiTE-Linux-x64-2.2/Samples/Bin/OpenNI2/Drivers/
cp OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so NiTE-Linux-x64-2.2/Samples/Bin
source setup_nite.bash

# Der im vorherigen Abschnitt ausgewählte Tracker wird aus dem entsprechenden Git in den $/src$ -Ordner heruntergeladen.
	
cd catkin_ws 
cd src
git clone https://github.com/mcgi5sr2/kinect2_tracker
cd ..

# An dieser Stelle muss ein build- und bash-Befehl durchgeführt werden.

cd catkin_ws	
catkin_make_isolated
source devel_isolated/setup.bash

# Der Tracker kann mittels dem Launch-file getestet werden:

roslaunch kinect2_tracker tracker.launch

# Das vom Tracker aufgezeichnete Skelett soll später mit der Robotersimulation ABB IRB 1600-6/1.2 verknüpft werden. Hierzu wird der Treiber $abb\_egm\_driver$ für die Robotersimulation in Rviz aus dem folgenden GitHub in $/src$ geklont \citep{ABB_EGM}.

sudo apt update
sudo apt dist-upgrade
sudo apt install git swig libnlopt-cxx-dev ros-noetic-nlopt ros-noetic-catkin ros-noetic-moveit
git clone https://github.com/ros-industrial/industrial_core.git
cd industrial_core
sudo apt install git-extras
git pr 258
cd ..
cd src
git clone --recursive https://gitlab.cvh-server.de/jweber/abb_egm_driver.git
catkin build
cd src
git clone https://bitbucket.org/traclabs/trac_ik.git

# Es muss die Zeile 35 in trac\_ik/trac\_ik\_lib/include/nlopt\_ik.hpp durch den folgenden Ausdruck ersetzt werden.

#include <nlopt/nlopt.hpp>

# An dieser Stelle muss ein build- und bash-Befehl durchgeführt werden.
	
cd catkin_ws
catkin_make_isolated
source devel_isolated/setup.bash

# In einem separaten Fenster wird das Skelett in das Koordinatensystem des Roboters gelegt.
	
rosrun tf static_transform_publisher 0 0 0 -0.5 0.5 0.5 0.5 kinect/user_1/left_foot base_link 10

# Der letzte Befehl öffnet Rviz und bildet die Robotersimulation und das aufgezeichnete Skeleton-Tracking ab.

roslaunch abb_irb1600_6_12_moveit_config demo.launch

-----------------------------------------------------------------------------------
Die abliegenden Python-Skripte müssen in dem Paket kinect2_tracker/ abgelegt werden.
abb_kinectv2_control.launch muss in dem Ordner kinect2_tracker/launch/ abgelegt werden.

# Zum Ausführen ist der folgende Befehl auszuführen: 

cd catkin_ws
catkin_make_isolated
source devel_isolated/setup.bash
roslaunch kinect2_tracker abb_kinectv2_control.launch
