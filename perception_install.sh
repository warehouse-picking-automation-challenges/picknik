# AUTO INSTALL PERCEPTION
# Make sure perception code is located in ~/ros/perception

# Dependencies:

sudo apt-get install -y autoconf2.13 libglm-dev libtinyxml2-dev libglew-dev doxygen

#Install protobuf

cd ~/ros/perception/protobuf
./autogen.sh
./configure
make -j3
sudo make install all
sudo cp /usr/local/lib/libproto* /usr/lib/

### Add building shortcut

function cmaker()
{
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j6
}

### Install CoreDev

cd ~/ros/perception/CoreDev
cmaker

### Install kangaroo (for SDF fusion, ray casting):

cd ~/ros/perception/Kangaroo
cmaker

### Install wallaby (for grid sdf fusion and voxel hashing):

cd ~/ros/perception/wallaby
cmaker

### Install DDTR

cd ~/ros/perception/DDTR
git submodule init
git submodule update
cmaker
