---
layout: post
title: Installation things - 1
---

## JdeRobot-base

Lets start with the installation of the JdeRobot-base repository (since this in itself is a challenge XD). The instructions for this can be found both on the [official page](http://jderobot.org/Installation) and the [github readme](https://github.com/JdeRobot/base).

{: .box-note}
**Note:** For best results, always check if there are any modifications to the official installation instructions for any dependancies.

{: .box-warning}
**Disclaimer:** The following instructions worked for me. Hopefully they shall work for you too.

1. Install ROS and Gazebo:  
    At present, it's best to install ROS Kinetic with Ubuntu 16.04.6 LTS on your system. Assuming you already have a system running Ubuntu 16.04.x, the [official instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) are sufficient for a successful install. For the purpose of dumping the commands in one place:

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
    ```

2. Add the Ice repository:  
    Since JdeRobot uses Ice 3.6, we can follow the instructions provided [here](https://zeroc.com/downloads/ice/3.6) for Ubuntu 16.04 for adding the repository. The required packages of Ice will automatically be installed in the next step. The code (for the same reason as above):

    ```bash
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 5E6DA83306132997
    sudo apt-add-repository "deb http://zeroc.com/download/Ice/3.6/ubuntu16.04 stable main"
    sudo apt-get update
    ```

3. Add the JdeRobot repository and install the deps:  
    Simply run:

    ```bash
    sudo sh -c 'cat<<EOF>/etc/apt/sources.list.d/jderobot.list
    # for ubuntu 16.04 LTS (64 bit)

    deb [arch=amd64] http://jderobot.org/apt xenial main
    EOF'
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 24E521A4
    sudo apt update
    sudo apt install jderobot-deps-dev
    ```

4. Install the Ice 3.6 Python dependancies:
    Step 3 does not install zeroc-ice for python and you will not be able to build the project without this:

    ```bash
    sudo pip2 install zeroc-ice
    ```

5. Get, build and install:
    Finally, get the latest source code and use make to compile and install it. (This step will take quite some time)

    ```bash
    git clone http://github.com/JdeRobot/JdeRobot.git
    cd JdeRobot
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```

That's all folks! You shall now (hopefully) have JdeRobot-base installed on your system.

{: .box-note}
**Note:** There was a build issue due to the entry removed in [issue #1355](https://github.com/JdeRobot/base/issues/1355) this was fixed in [pull #1374](https://github.com/JdeRobot/base/pull/1374)