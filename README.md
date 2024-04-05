# COMP0129: Robotic Sensing, Manipulation and Interaction

Coursework 3: Pick and Place, Object Detection and Localization

Completed by **Team 6**



## Authors:

Yujie Wang (ucab211@ucl.ac.uk)

Xingyu Chen (xingyu.chen.23@ucl.ac.uk)

Yufeng Wu (yufeng.wu.22@ucl.ac.uk)



## New Features

Compared to cw1, we add the `<mutex>` library to control multiple threads not reading/writing the shared variable at the same time.

Also, we add the `addCollisionOBject()` function to avoid the gripper crash into the objects on the ground when scanning the ground in task 3.



## How to Build and Run the Package:

To build the package, run:
```console
> catkin build
```

To run the package, type the following code in the terminal:
```console
> source devel/setup.bash
> roslaunch cw3_team_6 run_solution.launch 
```

The call the services for each tasks, open a new terminal and source:

```console
> source devel/setup.bash
```

In the same terminal, type one of the following code to start a task service:

```console
> rosservice call /task 1
> rosservice call /task 2
> rosservice call /task 3
```



## Tasks Informations:

#### Total time: 60 hours

#### Task 1: MoveIt! - Pick and Place at given positions [15 hours]

Contributions: Yujie 33%, Yufeng 33%, Xingyu 33%

```console
> roslaunch cw3_team_6 run_solution.launch 
> rosservice call /task 1
```

Performance:

We have tried 20 times to test our program performance.

When set `T1_ANY_ORIENTATION = False` , the success rate of pick and place is 100%.

When set `T1_ANY_ORIENTATION = True` , the success rate of pick and place is about 85%.

We implement the PCA algorithm for point cloud to recognize the rotation degree of object orientation. However, due to uncertainty of simulated sensor, there could be some measurement error which could lead to a wrong grasp orientation. 



#### Task 2: Shape detection [10 hours]

Contributions: Yujie 33%, Yufeng 33%, Xingyu 33%

```console
> roslaunch cw3_team_6 run_solution.launch 
> rosservice call /task 2
```

The result of Task 2 is shown in the ROS console, in the form of: 

```console
[ INFO] [1712314193.412128074, 25.204000000]: shape of mystery is the same as ref 2
```

Performance:

When set `T2_ANY_ORIENTATION = True` , the success rate of shape detection is 100%.

When set `T2_GROUND_PLANE_NOISE = True ` , the success rate of shape detection is 100%.



#### Task 3: Planning and Execution [35 hours]

Contributions: Yujie 33%, Yufeng 33%, Xingyu 33%

**Importance:** For task3, It is necessary to restart the launch file. When testing on our laptops, if we set `T2_GROUND_PLANE_NOISE=50e-3` in task 2 and directly call the service for task3, not relaunching, the gound will be generated as the same height in task 2 while not be the expected `0e-3`. However, in task 3 the ground or grass is always on `0e-3`. So after you have finished evaluating task 2, please **do relaunch** and then call the task 3 service. **That is very important !!!** 

```console
> roslaunch cw3_team_6 run_solution.launch 
> rosservice call /task 3
```

The result of Task 3 is shown in the ROS console, in the form of: 

```console
[ INFO] [1712312298.708354605, 270.271000000]: totoal_num_shapes = 6
[ INFO] [1712312298.708387497, 270.271000000]: num_most_common_shape = 4
```

Performance:

When  `T3_ANY_ORIENTATION = True ` , the success rate of pick and place is about 85%. 

Under other testing conditions, the success rate is almost 100%.

## License:

LICENSE: MIT. 

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2019-2024 Dimitrios Kanoulas except where specified

 
