# Project: Kinematics Pick & Place
### This writeup will attept to explain the processes used to complete the Kinematics Pick & Place Project for the Udacity Robotics Nano-Degree.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/fwd_kin.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  

This Writeup should fulfill the requirement of the first Rubric

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

On inspecting the `kr210.urdf.xacro` file supplied within the project folders, it is possible to extract the physical data for the arm. This includes joint origin co-ordinates (relative to the previous joint), joint rotation limits and the axis of rotation, which can be used to create a Denavit-Hartenberg (D-H) parameter table.
In doing this, I first created a table of data for each link and joint:

Joint | x-dist | y-dist | z-dist | axis
--- | --- | --- | --- | ---
1 | 0.0 | 0.0 | 0.33 | z
2 | 0.35 | 0.0 | 0.42 | y
3 | 0.0 | 0.0 | 1.25 | y
4 | 0.96 | 0.0 | -0.54 | x
5 | 0.54 | 0.0 | 0.0 | y
6 | 0.193 | 0.0 | 0.0 | x
G | 0.11 | 0.0 | 0.0 | y

In transforming this data into D-H values, the following rules were followed:

	1. In describing reference frames for each joint, the z-axis is along the joint rotational axis.
	2. common normals should be identified between joint reference frames.
	3. Where possible parameters should ideally be zero.
	4. Where links are coincident with joint axes, the sum of the link lengths is to be assigned to the link furthest from the last perpendicular joint axis.

This enabled me to produce the following D-H Parameter Table:

i | \alpha~i-1 | a~i-1~ | d~i~ | \theta
--- | --- | --- | --- | ---
T^0^~1~ | 0 | 0 | 0.75 | -
T^1^~2~ | -\pi/2| 0.35 | 0 | -
T^2^~3~ | 0 | 1.25 | 0 | \theta~2~ - \pi/2
T^3^~4~ | -\pi/2 | -0.54 | 1.50 | -
T^4^~5~ | \pi/2 | 0 | 0 | -
T^5^~6~ | -\pi/2 | 0 | 0 | -
T^6^~G~ | 0 | 0 | 0.303 | 0



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The above D-H Parameter table was used to create transformation matrices for each of the joints, all of which took the following form:

Tn~-1~_n =
[cos(n) | -sin(n) | 0 | a n~-1~]
[sin(n)*cos(alpha n~-1~) | cos(n)*cos(alpha n~-1~) | -sin(alpha n~-1~) | -sin(alpha n~-1~)* d n]
[sin(n)*sin(alpha n~-1~) | cos(n)*sin(alpha n~-1~) | cos(alpha n~-1~) | cos(alpha n~-1~) * d n]
[0 | 0 | 0 | 1]

Once give angles for each of the joints, these individual transformation matrices could then be used to produce a matrix for the entire arm. This is done via multiplication of each matrix in series, to produce a final matrix of the form:

R0_G ||| Px
--- | --- | --- |---
|||| Py
|||| Pz
0 | 0 | 0 | 1

Where Px Py and Pz are the co-ordinates of the gripper link, and R0_6 is the rotation matrix the describes it's orientation.

It is at this point that I used the `forward_kinematics.launch` program and `rosrun tf tf_echo`, together with my own code to produce a forward kinematics model, to adjust for any errors between Rviz* and my calculations.

![alt text][image1]
Using Rviz* to perform error correction

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

In order to solve the problem of Inverse Kinematics for this project, it was first necessary to split it into two smaller problems; Position Kinematics for Joints 1,2 and 3, and Orientation Kinematics for Joints 4,5 and 6.
In doing this, we effectively split the arm into two sections. The shoulder and elbow section, containing joints 1, 2 and 3, and the wrist section containing joints 4,5 and 6. We refer to this as a wrist joint due to their collective behaviour, which is similar to the human wrist, and for this reason we can effectively treat them as a single, double-revolute joint.
So to start, it is necessary to find the position of the 'wrist centre' (actually the centre of Joint 4). This is achieved by using the Positional co-ordinates and z-axis components from the overall tranformation matrix. These are input into the following formulae:

	* Wx = Px - d7 * Nx
	* Wy = Py - d7 * Ny
	* Wz = Pz - d7 * Ny

From here, it is possible to find Joints 1, 2 and 3 via the use of triganometric rules.

For Joint 1, pythagoras' theorem:

	* a^2^ = b^2^ + c^2^

Which is adapted to solve for \theta1 from Wx and Wy, using the arctangent:

	* \theta1 = `atan2(Wy, Wx)`

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


