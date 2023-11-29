# Package README

## Building the package

```bash
source /knowrob_ws/devel/setup.bash
catkin_make

# or
./src/cocktail_bot/scripts/build.sh
```

This document will guide you through setting up the environment for each task and provide the relevant commands to get started.

## Environment Setup

For each task, make sure to set up the right environment (workspace paths) to ensure a clean run. You can do this by following these general steps:

1. **Set Up Environment:**
```bash
source ./devel/setup.bash
```

## Task 1: Get Scene Object

### Launch file

```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user/exchange/src/cocktail_bot

roslaunch cocktail_bot task1.launch
```

**Description:**
This will launch the gazebo environment as well as the percept and map generator nodes.

### Test service (new terminal)

```bash
rosservice call /get_scene_object_list "{object_name: "all" }"
```

**Description:**
By calling the service with the specified `object_name` we expect to receive the name and pose of every object Tiago has seen.

For testing purposes it might be necessary walking around with the robot so it gathers knowledge about new objects.

## Task 2: ROS Parameters

### Start rosprolog

```bash
roslaunch cocktail_bot task2.launch
```

### Load parameters

```bash
rosparam load ./src/cocktail_bot/config/loadKnowledge.yaml
```

### Start reasoning_node (in the same terminal)

```bash
rosrun cocktail_bot reasoning_node ./src/cocktail_bot
```

**Description:**
If all parameters are well defined in the yaml file, we are expected to have opened and created if the query file didn't previously existed.

Note: It may be necessary to open the file explorer since the file will still be open when running the node, thus not appearing right away.

## Task 3: Load Knowledge

### Start rosprolog

```bash
roslaunch cocktail_bot task3.launch
```

### Load parameters

```bash
rosparam load ./src/cocktail_bot/config/loadKnowledge.yaml
```

### Start knowledge_node (in the same terminal)

```bash
rosrun cocktail_bot knowledge_node ./src/cocktail_bot
```

### Call running service

```bash
rosservice call /load_knowledge "{start: 1 }"
```

**Description:**
This will make the `knowledge_node` load the queries available in the given file. Further testing can be done by directly querying `rosprolog`.

## Task 4: Control Tiago

### Script version (skip to next part if you prefer direct commands)

For this task there are two scripts which can be used to test the code. These need to be modified to the correct workspace path, but the idea is:


```bash
# Terminal 1
./src/cocktail_bot/scripts/gazebo.sh

# Terminal 2
./src/cocktail_bot/scripts/task4.sh

# Call service as explained later
```

### Non-scripted version

#### Start gazebo

```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/<Path_to_Workspace>/src/cocktail_bot

roslaunch cocktail_bot gazebo_ssy236.launch
```

#### Start necessary nodes (new terminal)

```bash
roslaunch cocktail_bot task4.launch
```

### Test controller

```bash
rosservice call /go_to_object "{object_name: beer }"
```

**Description:**
Calling the `go_to_object` service will initiate the movement of the robot towards the given object, if it has already been perceived. One can increase the distance of the percept node to have direct knowledge about all objects.

Once the robot reaches the object a message should be shown stating that same information.