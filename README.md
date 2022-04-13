# SVEA Starter Suite (PVK Edition)

### Quicklinks:

-   [SVEA website](https://svea.eecs.kth.se)
-   [Tutorials](https://github.com/KTH-SML/svea/tree/main/docs/tutorials)
-   [Sim to Real Tips](https://github.com/KTH-SML/svea#going-from-simulation-to-real)
-   [Testing](https://github.com/KTH-SML/svea#testing)

## A short description

This repo contains a basic library of python objects and scripts to make
development on the Small-Vehicles-for-Autonomy (SVEA) platform simpler
and cleaner.

The design principle of this library is to help create projects that are
more modular and easier to troubleshoot. As opposed to the standard
approach of creating a large web of Subscriber/Publisher nodes, we modularly
wrap different ROS entities in Python objects, while exposing only the useful
features with object-oriented interfaces.

## Useful to know before starting

Before continuing to the next sections, consider taking some time to read up on
two important concepts for this code base: the **Robotic Operating System (ROS)**
and **Object Oriented Programming (OOP)**.

To read up on ROS, check out the
[ROS Start Guide](http://wiki.ros.org/ROS/StartGuide). However, do not spend
too much time diving into the guide. The structure and tutorials are not very
intuitive, but glossing over them will give a sense of what ROS is and how you
are meant to use it. The rest of the learning curve is overcome by trying it out
yourself.

To read up on OOP, check out Real Python's
[introduction on OOP](https://realpython.com/python3-object-oriented-programming/).

# Installation

## System Requirements

This library is developed on and intended for systems running:

1. Ubuntu 18.04 (installation tutorial [here](https://ubuntu.com/tutorials/tutorial-install-ubuntu-desktop#1-overview))
2. ROS Melodic (installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu))
3. Python 2.7

Python 2.7 will be made default when you install ROS. An easy way to check if
Python 2.7 is the default version on your system is to open a terminal and run

```bash
python
```

to make sure you see "Python 2.7" appear somewhere in the header of the text
that appears afterwards.

If you do not want to install Ubuntu onto your computer, consider installing a
[virtual machine](https://www.osboxes.org/ubuntu/) or use
[docker](https://docs.docker.com/install/) with Ubuntu 18.04 images.

Some may need to install some additional python tools (install the **Python 2.7**
versions):

1. [numpy](https://scipy.org/install.html) **(You may need to update your version of numpy to the newest)** You can do this with `pip install numpy`
2. [matplotlib](https://matplotlib.org/users/installing.html)

The installation instructions later on will use `catkin build` instead of
`catkin_make`, so you should also [install catkin tools using apt-get](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get).

If you had a pre-existing ROS Melodic installation, please run:

```bash
sudo apt update
sudo apt upgrade
```

before continuing onto installing the library.

## Installing the library

Start by going to the folder where you want the code to reside.
For example, choose the home directory or a directory for keeping projects in.
Once you are in the chosen directory, use the command:

```bash
git clone https://github.com/ABConnectIO/svea.git
```

to download the library. Then, a new directory will appear called
`./svea`. Go into the directory with command:

```bash
cd svea
```

go to the branch designated for the PVK course

```bash
git checkout pvk
```

To install all of the ROS dependencies that you are missing for this library run:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Finally, compile and link the libraries using:

```bash
catkin build
source devel/setup.bash
rospack profile
```

**IMPORTANT**: Remember to source the workspace `source devel/setup.bash` for every new terminal you open.

# Usage

The intended workflow with the code base is as follows:

1. Write new features/software
2. Debug the new contributions in simulation
3. Perform basic tuning and adjustments in simulation
4. Evaluate actual performance on a SVEA car

The simulated vehicles provide identical interfaces and information patterns
to the real SVEA cars, thus by following this workflow, development work
should always start in simulation and code can be directly ported to the real
cars. However, this does not mean the code will work on a
real vehicle without further tuning or changes.

There are three pre-written scripts to serve as examples of how to use the
core library. See and read the source code in
`svea_core/scripts/core_examples`.

For PVK, the following simulation provides a vehicle driving around a circuit at floor2 in the Q-building:

```bash
roslaunch svea_core floor2.launch
```

where you should see something that looks like:

![key-teleop example](./media/floor2_rviz.png)

Now you are ready to read through the tutorials! You can find them in `svea_starter/docs/tutorials`.

## Using the Lidar and vehicle positions for PVK

When you launch the vehicle example above (`roslaunch svea_core floor2.launch`), two important topics will be published

-   `/scan` of type "sensor_msgs/LaserScan"
-   `/state` of type "svea_msgs/VehicleState"

Here is an example of how you could utilize these two topics to publish them over the ABConnect software to the controller peer (in your case to the Unity client). The following examples are based on the [simple_nodejs_demo](https://github.com/ABConnectIO/simple_nodejs_demo)

### Modifications to `vehicle.js`

The following code runs on the machine which simulates the vehicle

```javascript
const { ABConnect, ROSPlugin } = require("abconnect-sdk-lite");
const connection = new ABConnect(
    {
        host: "wss://intelligent-cluster-manager.herokuapp.com/signal",
        auth: "",
    },
    {
        id: "svea",
        category: "car",
    },
    new ROSPlugin(9092)
);

// Add co-device
connection.addDeviceById("controller").then((controller) => {
    controller.addPublisher(0, "/state", "svea_msgs/VehicleState", {});
    controller.addPublisher(1, "/scan", "sensor_msgs/LaserScan", {});
});
```

### Modifications to `controller.js`

The following code runs on the machine with the Unity application

```javascript
const { ABConnect } = require("abconnect-sdk-lite");
const connection = new ABConnect(
    {
        host: "wss://intelligent-cluster-manager.herokuapp.com/signal",
        auth: "",
    },
    {
        id: "controller",
        category: "tower",
    }
);
connection.addDeviceById("svea").then((vehicle) => {
    vehicle.addSubscriber(0, "/state", "svea_msgs/VehicleState", {}, (msg) => {
        console.log("state:", msg);
    });
    vehicle.addSubscriber(1, "/scan", "sensor_msgs/LaserScan", {}, (msg) => {
        console.log("scan:", msg);
    });
});
```

## Integration ZedM camera

All the work you have already done for the ZedM camera can be included to the `vehicle.js` and `controller.js` examples above. You launch the stereo camera along side the vehicle simulation as follows:

-   Open a new terminal window
-   `cd` to the workspace where the ZedM code lives
-   source `devel/setup.bash` of that workspace
-   Launch the ZedM camera with `roslaunch zed_wrapper zedm.launch`
