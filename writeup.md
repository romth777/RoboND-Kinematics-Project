## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is the Link frames and joint structure of Kuka KR210,  
![image1](./misc_images/writeup2.png)

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

where;  
![image2](./misc_images/writeup3.png)

Also the parameters of the a and d is based on the xacro file of ["kr210.urdf.xacro"](https://github.com/romth777/RoboND-Kinematics-Project/blob/master/kuka_arm/urdf/kr210.urdf.xacro) of 316 line;
```
<!-- joints -->
<joint name="fixed_base_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
<joint name="joint_1" type="revolute">
  <origin xyz="0 0 0.33" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="link_1"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
</joint>
<joint name="joint_2" type="revolute">
  <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
  <parent link="link_1"/>
  <child link="link_2"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
</joint>
<joint name="joint_3" type="revolute">
  <origin xyz="0 0 1.25" rpy="0 0 0"/>
  <parent link="link_2"/>
  <child link="link_3"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
</joint>
<joint name="joint_4" type="revolute">
  <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
  <parent link="link_3"/>
  <child link="link_4"/>
  <axis xyz="1 0 0"/>
  <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
</joint>
<joint name="joint_5" type="revolute">
  <origin xyz="0.54 0 0" rpy="0 0 0"/>
  <parent link="link_4"/>
  <child link="link_5"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
</joint>
<joint name="joint_6" type="revolute">
  <origin xyz="0.193 0 0" rpy="0 0 0"/>
  <parent link="link_5"/>
  <child link="link_6"/>
  <axis xyz="1 0 0"/>
  <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
</joint>
```
And about Gripper, it is on the 202;
```
<joint name="gripper_joint" type="fixed">
  <parent link="link_6"/>
  <child link="gripper_link"/>
  <origin xyz="0.0375 0 0" rpy="0 0 0"/>
</joint>
</xacro:if>
```

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
First of all, the definition of the homogeneous transform from frame i-1_th to frame i_th is below;  
![image3](./misc_images/writeup4.png)
![image4](./misc_images/writeup5.png)
where;
 * Rx/Rz:Rotation around X/Z axis
 * Dx/Dz:Transformation along X/Z axis
 * c:cosine
 * s:sine

The individual transformatnion matrices are below; where the J0,J3,J5 are the roll joint, and J1,J2,J4 are the pitch joint. The basic structure is the same in each roll and pitch joint.
```python
T0_1 = [[cos(q1), -sin(q1), 0, 0],
        [sin(q1), cos(q1),0,0],
        [0,0,1,0.75],
        [0,0,0,1]]

T1_2 = [[sin(q2), cos(q2), 0, 0.35],
        [0,0,1,0],
        [cos(q2),-sin(q2),0,0],
        [0,0,0,1]]

T2_3 = [[cos(q3), -sin(q3), 0, 1.25],
        [sin(q3), cos(q3),0,0],
        [0,0,1,0],
        [0,0,0,1]]

T3_4 = [[cos(q4), -sin(q4), 0, -0.054],
        [0,0,1,1.5],
        [-sin(q4),-cos(q4),0,0],
        [0,0,0,1]]

T4_5 = [[cos(q5), -sin(q5), 0, 0],
        [0,0,-1,0],
        [sin(q5),cos(q5),0,0],
        [0,0,0,1]]

T5_6 = [[cos(q6), -sin(q6), 0, 0],
        [0,0,1,0],
        [-sin(q6),-cos(q6),0,0],
        [0,0,0,1]]

T5_6 = [[1,0,0,0],
        [0,1,0,0],
        [0,0,1,0.303],
        [0,0,0,1]]

```

The Generalized homogeneous transform of base link and gripper link is below;
```python
Matrix([
    [cos(p)*cos(y), sin(p)*sin(r)*cos(y) - sin(y)*cos(r), sin(p)*cos(r)*cos(y) + sin(r)*sin(y), x],
    [sin(y)*cos(p), sin(p)*sin(r)*sin(y) + cos(r)*cos(y), sin(p)*sin(y)*cos(r) - sin(r)*cos(y), y],
    [-sin(p), sin(r)*cos(p), cos(p)*cos(r), z],
    [0, 0, 0, 1]])
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The structure of Joint4,5,6 is the Spherical Wrist, so the Wrist Center(WC) and the orientation of the End Effector(EE) can be palatable independently. To do this, I did solve the position of WC then I address the composition of rotation of the orient of EE. The hole of the axis and angles are in the figures.  
The calculation of WC position is below;
```
pos0_WC = pos0_EE - d7 * rot0_E[:,2]
```
where;
 * pos0_WC: Location of the WC in the base frame
 * pos0_EE: Location of the EE in the base frame
 * d7: Link length from WC to EE
 * rot0_E: Rotation of the EE in the base frame

In below we will calculate the theta1,2,3. In the basis to do that, we already know the position of all the joints in the base frame so that we will re-calculate the angles of joint using the position(=Inverse Kinematics).

First, we calculate the theta 1 of Joint1 which is based on the x and y position of WC;  
![image5](./misc_images/writeup7.png)
```
theta1 = atan2(pos0_WC[1], pos0_WC[0])
```
then next, we calculate the theta2, 3;  
![image6](./misc_images/writeup6.png)
```
theta2 = pi/2 - alpha - delta
theta3 = pi/2 - beta - epsilon

alpha = acos((b**2 + c**2 - a**2) / (2 * b * c))
alpha = acos((a**2 + c**2 - b**2) / (2 * a * c))

delta = atan2(pos0_WC - d1, sqrt(pos0_WC[0]**2 + pos0_WC[1]**2) - a1)
epsilon = atan2(-a3, a4)
```
where these are basis on the Law of cosines. Finally we can calculate the theata4,5,6. We will use the EE pose and theta1,2,3, we need to calculate the rotation between frame3 to 6 as below;
```
R3_6 = inverse(R0_3) * R0_6
```
where;
 * R3_6:Rotation matrix between frame3 to 6
 * R0_3:Rotation matrix calculated with theta1,2,3
 * R0_6:Rotation matrix between base frame and EE

Hence, the thetas can be calculated with the result of above.
```
theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

In the initialize_global_variables() function, I did the initialization of valuables to make the code readable. Of course, I did the Inverse Kinematics to empower the arm, and also implement the Forward kinematics to calculate the EE position Error.

The result of my implementation, the arm can catch the cylinder and drop it into the garbage can. The trajectory of the arm is not wrong, but there is the improbable point. That is the arm path route is not efficiently. The arm first folds, then rotates, moves to the front of the cylinder, and moves to orient the wrist. It may be easier to see the movement in the sense of separating motion, but I think that it is more efficient to carry out all at the same time.

![image7](./misc_images/writeup8.png)  
![image8](./misc_images/writeup9.png)
