This document describes the basic usage of the 3D environments.

### How to run in Robot Learning environment
Change directory to robot/game_player, then
```bash
python run_robo_player.py
```

### Files to look at
Under directory `ROBOSCHOOL_PATH/roboschool`:
* `scene_conf.py`: the script to specify how to construct the environments.
* `gym_interact_demo.py`: the environment that reads `scene_conf.py` and construct the environment accordingly.

### Environment configuration
The configuration is done in `scene_conf.py` which specifies the function `scene_config`. The input `history` of the function `scene_config` is a list of 0 and 1 where 1 represents a success and 0 a failure case. The most recent record is at the end of the list. The history is used for curriculum learning control.

The expected returns of the function `scene_config` are as follows.
* A list: A list of objects in the environment. Each item in the list is in the form `[x, y, z, object_type, step_function, path_to_the_models]`, where `x, y, z` specify the initial place for this object. There are four built-in object types: `agent`, `block`, `movable`, `goal`. Their behaviors will be detailed later. `step_function` is the function that will be called every step. Note that only movable objects can have meaningful `step_function`. Finally, `path_to_the_models` tells the configuration parser where to look for the 3D models. How to build 3D models will be detailed later.
* A number: the maximum steps that the agent can go before failing the task.
* A list: the allowed actions for the agent. This part will be detailed later.

Currently, the `scene_conf.py` uses a demo configuration for building a random maze. The demo `step_function` is a identity function on the position and the speed of the movable object.

### Object types
* Agent. The agent is the fully controlled object in the environment which can take one of the following actions in each step: 'a' -- turn left, 'w' -- move forward, 'd' -- turn right, 'c' -- collect the goal in front it, 'z' -- change the viewpoint between first person and third person view, and 'j' -- jump. There are can be only one agent in the environment, and the camera will follow it. In the third person view, it's recommended to use model `models_household/ball/ball.urdf`, while in the first person view, it's better to use model `models_household/ball-small/ball.urdf`.
* Block. The block object is the non-movable object. It will always be where it is initially.
* Movable. The movable object is the object that moves according to the physics. You can specify `step_function` to interfere with this process.
* Goal. The goal is a collectable block object. It will be picked up if the agent collects it when standing close to and facing the goal. It will fly up after being collected. If all the goals in the environment are collected, the agent succeeds.

### 3D Models
Free 3D models can be found [here](http://www.sweethome3d.com/freeModels.jsp). The downloaded models usually have a `.obj` file, a `.mtl` file, and some optional image textures. To use the models, create a `.urdf` file for each in the same folder.
```xml
<?xml version="1.0"?>
<robot name="anyname">
    <link name="anyname">
        <visual>
            <origin rpy="1.57 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="yourmodel.obj"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>

        <inertial>
          <mass value="0.0"/>
          <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
        </inertial>
    </link>
</robot>
```
There are two aspects to pay attention to.
* Collision. The tag collision is the mesh used for calculating collision. In this example, the collision mesh is a sphere with a radius of 0.05.
* Mass. The mass of the object. __NOTE:__ if the mass if set to 0, the object will not follow the physics, i.e., become blocking for any other objects. This is used for `block` object.

For editing the models, [Blender](https://www.blender.org/) is powerful and free to use.

### Default setting
In the default setting, the 3D environment is built in a grid way. There is a plane holding everything at (0, 0, -0.05) and facing up. This is specified in the file `ROBOSCHOOL_PATH/roboschool/mujoco_assets/demo.xml`
```xml
<geom name="floor" type="plane" size="1 1 0.1" rgba=".9 0 0 0" pos="0 0 -0.05" friction="1.0 0.1 0.1"/>
```
Every object is within a collision box of size (0.1, 0.1, 0.1).
