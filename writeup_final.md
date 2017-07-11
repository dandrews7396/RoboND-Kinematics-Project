# Project: Kinematics Pick & Place
### This writeup will attept to explain the processes used to complete the Kinematics Pick & Place Project for the Udacity Robotics Nano-Degree.

### This document uses LaTeX and mathjax to display mathematical formulae
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
[image2]: ./misc_images/theta_1.png
[image3]: ./misc_images/theta_2_3.png
[image4]: ./misc_images/cosine.gif
[image5]: ./misc_images/frames.png

[//]: # (Footnotes)

[^1]: http://www.bbc.co.uk/schools/gcsebitesize/maths/geometry/furthertrigonometryhirev2.shtml

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
--- | :---: | :---: | :---: | ---:
1 | 0.0 | 0.0 | 0.33 | z
2 | 0.35 | 0.0 | 0.42 | y
3 | 0.0 | 0.0 | 1.25 | y
4 | 0.96 | 0.0 | -0.54 | x
5 | 0.54 | 0.0 | 0.0 | y
6 | 0.193 | 0.0 | 0.0 | x
G | 0.11 | 0.0 | 0.0 | y

In transforming this data into D-H values, the following rules were followed:

1. In describing reference frames for each joint, the z-axis is along the joint rotational axis. (See figure below)
2. common normals should be identified between joint reference frames.
3. Where possible parameters should ideally be zero.
4. Where links are coincident with joint axes, the sum of the link lengths is to be assigned to the link furthest from the last perpendicular joint axis.

![alt text][image5]


This enabled me to produce the following D-H Parameter Table:

i | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta$
--- | --- | --- | --- | ---
$T^0_1$ | 0 | 0 | 0.75 | -
$T^1_2$ | $-\pi/2$| 0.35 | 0 | -
$T^2_3$ | 0 | 1.25 | 0 | $\theta_2 - \pi/2$
$T^3_4$ | $-\pi/2$ | -0.54 | 1.50 | -
$T^4_5$ | $\pi/2$ | 0 | 0 | -
$T^5_6$ | $-\pi/2$ | 0 | 0 | -
$T^6_G$ | 0 | 0 | 0.303 | 0



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The above D-H Parameter table was used to create transformation matrices for each of the joints, all of which took the following form:


$$
T_{n-1}^n =
\begin{bmatrix}\cos(n) & -\sin(n) & 0 & a_{n-1} \\ \sin(n)\times\cos(\alpha_{n-1}) & \cos(n)\times\cos(alpha_{n-1}) & -\sin(alpha_{n-1}) & -\sin(alpha_{n-1}) \times d_n \\
\sin(n)\times\sin(alpha_{n-1}) & cos(n)\times\sin(alpha_{n-1}) &
\cos(alpha_{n-1}) & \cos(alpha_{n-1}) \times d_n \\ 0 & 0 & 0 & 1\end{bmatrix}
$$

Once give angles for each of the joints, these individual transformation matrices could then be used to produce a matrix for the entire arm. This is done via multiplication of each matrix in series, to produce a final matrix of the form:

$$
T^0_6 =
\begin{bmatrix} & & & Px \\ & R^0_6 & & Py \\ & & & Pz \\ 0 & 0 & 0 & 1\end{bmatrix}
$$
Where Px Py and Pz are the co-ordinates of the gripper link, and $R^0_6$ is the rotation matrix that describes it's orientation.

It is at this point that I used the `forward_kinematics.launch` program and `rosrun tf tf_echo`, together with my own code to produce a forward kinematics model, to adjust for any errors between Rviz* and my calculations.

![alt text][image1]
Using Rviz* to perform error correction

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

In order to solve the problem of Inverse Kinematics for this project, it was first necessary to split it into two smaller problems; Position Kinematics for Joints 1,2 and 3, and Orientation Kinematics for Joints 4,5 and 6.
In doing this, we effectively split the arm into two sections. The shoulder and elbow section, containing joints 1, 2 and 3, and the wrist section containing joints 4,5 and 6. We refer to this as a wrist joint due to their collective behaviour, which is similar to the human wrist, and for this reason we can effectively treat them as a single, double-revolute joint.
So to start, it is necessary to find the position of the 'wrist centre' (actually the centre of Joint 4). This is achieved by using the Positional co-ordinates and z-axis components from the overall tranformation matrix. These are input into the following formulae:

$$w_x = P_x - d_7 \times N_x$$
$$w_y = P_y - d_7 \times N_y$$
$$w_z = P_z - d_7 \times N_y$$

From here, it is possible to find Joints 1, 2 and 3 via the use of triganometric rules.

For Joint 1, pythagoras' theorem:

$a^2 = b^2 + c^2$

Which is adapted to solve for \theta1 from Wx and Wy, using the arctangent:

$\theta_1$ = `atan2(Wy, Wx)`

and can been seen in this image:

![alt text][image2]


I could now turn my attention to $\theta_2$ and $\theta_3$ which, because $\theta_2$ is offset from the axis of $\theta_1$, I could treat as a completely separate triganometric sequence. However I would be working in the $x,z$-plane rather than the $x,y$-plane, used for $\theta_1$.

![alt text][image3]

Before I could begin calculating any angles, it is first necessary to formulate an equation for the distance between Joint<sub>2</sub> and the wrist centre. In doing this, I know that there is a permanent right angle where the x and z axes meet. So I can again use pythagoras' theorem to obtain the measurement (labelled as `c` in the image above).

$c = \sqrt{(w_x - a_1)^2 + (w_z - d_1)^2}$

Now that I have a value for $c$, I am able to calculate values for both $\theta_2$ and $\theta_3$. This can be done in one of two ways; the first being to calculate both angles using the cosine rule, or to use the cosine rule to calculate one joint angle and use the sine rule for the other. For the purpose of consistency, I have chosen to use the first method, so.

The cosine rule:
$$
\cos A = \frac{b^2 + c^2 - a^2}{2bc}
$$

Where $A$ is the angle opposite the side $a$, as can be seen in the following image [^1].

![alt text][image4]

I shall first deal with $\theta_3$ as, for reasons that didn't occur to me at first, it is the easiest of the two to obtain.

$$
\cos\theta_3 = \frac{a_2^2 + d_4^2 - c^2}{2a_2d_4}
$$

As previously alluded to, obtaining $\theta_2$ is slightly more complex because it is a combination of both the angle between $c$ and ($w_x - a_1$), and the one between $a_2$ and $c$. I shall henceforth, refer to these angles as $\theta_{21}$ and $\theta_{22}$ respectively. Because $\theta_{21}$ will always be part of a right-angle triangle, we can use the equation:

$$
\tan\theta_{21} = \frac{w_z - d_1}{w_x - a_1}
$$

and for $\theta_{22}$, we can again use the cosine rule:

$$
\cos\theta_{22} = \frac{a_2 + c - d_4}{2a_2c}
$$

which means that $\theta_2$ is:

$$\theta_2 = \tan\theta_{21} + \cos\theta_{22}$$

With the positional kinematics taken care of, we can now move onto the orientation kinematics. The start point for this is to obtain the partial rotation matrix, from Joint<sub>0</sub> to Joint<sub>3</sub>, or $R^0_3$. From this, and using the original $R^0_6$, it's possible to obtain the rotation matrix from Joint<sub>3</sub> to Joint<sub>6</sub> as the product of the inverse matrix $R^0_3$ and $R^0_6$.

$$R^3_6 = {R^0_3}^{-1} R^0_6$$

This produces a rotation matrix of the form:

$$
R^3_6 =
\begin{bmatrix} \cos\theta_4\cos\theta_5 & \cos\theta_4\sin\theta_5\sin\theta_6-\sin\theta_4\cos\theta_6 & \cos\theta_4\sin\theta_5\sin\theta_6+\sin\theta_4\sin\theta_6 \\ \sin\theta_4\cos\theta_5 & \sin\theta_4\sin\theta_5\sin\theta_6+\cos\theta_4\cos\theta_6 & \sin\theta_4\sin\theta_5\cos\theta_6-\cos\theta_4\sin\theta_6 \\ -\sin\theta_5& \cos\theta_5\sin\theta_6 & \cos\theta_5\cos\theta_6 \end{bmatrix}
$$

Which we can label in the following manner:

$$
R^3_6 =
\begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}
$$

It is then possible to isolate the angle values using the following methods:

$$
\theta_5 = \tan^{-1}\frac{-r_{31}}{\sqrt{r_{11}^2 + r_{21}^2}}
$$

$$
\theta_6 = \tan^{-1}\frac{r_{32}}{r_{33}}
$$

$$
\theta_4 = \tan^{-1}\frac{r_{21}}{r_{11}}$$
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


My code relies quite heavily on the sympy library, which from my experience sofar is quite slow in it's operation. Sympy is used to assign symbolic references to attributes of the arm, and these are then used in place of the calculated and constant values when solving each of the equations.

Although I originally planned to use the arcosine values for some of these angles, I actually found that using the cosine of the angle in the `atan2()` function returned values between $0$ and $2\pi$, which removed the worry of adjusting for elbow up or down orientation and similar situations.

I also found that I experienced gimbal lock situations in the final three joints, which meant that the arm would not behave as expected. I overcame this by using conditional statements to test for situations where the pitch of the end-effector was at it's most extreme (ie: $\cos\theta = \pm1$) and then setting the yaw angle to zero. In reality, I could have set either yaw *or* roll to zero, but I just picked yaw.

So far, I have not encountered a situation where my shoulder or elbow joints experience imaginary number situations, but I have been told that they are possible. They could be countered by setting the variable to 1 when it equates to more than 1.

I have also found that the arm is a bit hit and miss when it comes to reaching and grasping for the lower right shelf, and I think that this could be countered by fine tuning my correction angles. If I had had more time on this, I would devote it to writing better tests and optimising the code by relying less on sympy. I feel that using the numpy library would assist me in improving the speed of the calculations.

Moving forward, I think time spent on understanding decision making algorithms would be a good idea, so that I would better understand this process. Having completed this and the Rover project, this has been my most difficult concept to grasp. I would also opt to use Git from the very start, as this would allow for backups and better version control as well as availability across platforms.
