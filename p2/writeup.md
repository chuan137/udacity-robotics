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

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

[img1]: ./writeup/image1.jpg
[img2]: ./writeup/image2.png 
[img3]: ./writeup/image3.png
[img4]: ./writeup/image4.png
[img5]: ./writeup/image5.png
[img6]: ./writeup/image6.png
[img7]: ./writeup/image7.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Screenshot of forward_kinematics demo

![Rivz][img2]

First read the parameter of each joint from the urdf file:

Joint | x | y | z | axis
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.33 | 001
2 | 0.35 | 0 | 0.42 | 010
3 | 0 | 0 | 1.25 | 010
4 | 0.96 | 0 | -0.054 | 100
5 | 0.54 | 0 | 0 | 010
6 | 0.193 | 0 | 0 | 100
Gripper | 0.11 | 0 | 0 | 010

Then draw the DH diagram based on the urdf parameters. 
1. In the upper diagram, the links l0, l1, ..., l6 and joints 1, 2, ..., 6 is labeled. The rotation axes are drawn. The normal directions from old axis to new axis are also drawn where applicable. 
2. In the second diagram, the origin of each frame and the x,z axes are shown. Notice that, x3 points upwards, the same direction as x2. This makes the joint angle of frame 3 is simply theta_3. The origins of the frame 4,5,6 are set on the same position, i.e. the wrist center. This choice makes d5 and d6 both equal to 0.
![DH Parameter][img1]
![DH Parameter][img4]

Following are the derived Denavit-Hartenberg Parameters:

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->Gripper | 0 | 0 | 0.303 | 0

The parameters q1, q2, ..., q6 are the rotation angle of each revolute joint. 


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The formula of homogeneous transformation between frames:

    Matrix([[           cos(q),           -sin(q),           0,             a],
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
            [                0,                 0,           0,             1]])

From frame 0 to 1

    [cos(q1), -sin(q1),     0,     0]
    [sin(q1),  cos(q1),     0,     0]
    [      0,        0,     1,  0.75]
    [      0,        0,     0,     1]

From frame 1 to 2

    [sin(q2),  cos(q2),     0,  0.35]
    [      0,        0,     1,     0]
    [cos(q2), -sin(q2),     0,     0]
    [      0,        0,     0,     1]

From frame 2 to 3

    [cos(q3), -sin(q3),     0,  1.25]
    [sin(q3),  cos(q3),     0,     0]
    [      0,        0,     1,     0]
    [      0,        0,     0,     1]

From frame 3 to 4

    [ cos(q4), -sin(q4),     0, -0.054]
    [      0,        0,      1,   1.50]
    [-sin(q4), -cos(q4),     0,      0]
    [      0,        0,      0,      1]

From frame 4 to 5

    [ cos(q5), -sin(q5),     0,     0]
    [      0,        0,     -1,     0]
    [ sin(q5),  cos(q5),     0,     0]
    [      0,        0,      0,     1]

From frame 5 to 6

    [ cos(q6), -sin(q6),     0,     0]
    [       0,        0,     1,     0]
    [-sin(q6), -cos(q6),     0,     0]
    [       0,        0,     0,     1]

From frame 6 to Gripper

    [      1,        0,      0,      0]
    [      0,        1,      0,      0]
    [      0,        0,      1,  0.303]
    [      0,        0,      0,      1]

The homogeneous transformation from fixed base link to end effector link is the multiplication of all the matrices

$T = T_{01} T_{12} T_{23} T_{34} T_{45} T_{56} T_{6G}$ 




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Step 0. Calculate the requested rotaion transform in numerics. 

The input from Gazibo are the quaternion angels. From which, we can first calcuate the Euler angels (roll, pitch, yaw) with the help of the function `euler_from_quaternion`. The rotation transform is thus

$ROT = ROT_z(yaw) * ROT_y(pitch) * ROT_x(roll)$

The $x,y,z$ are the axis about which the rotation are made. Notice the Euler rotations are extrinsic, therefore the back order of the matrices. In addition, in Gazibo (URDF) convention, the Gripper axes are aligned with the base frame. In this case, the x,z axes interchang and the y axis is flipped, therefore with this compensation, the final rotaion transform is

$ROT = ROT_z(yaw) * ROT_y(pitch) * ROT_x(roll) * R_corr$

Where R_corr is just,

    [  0   0   1 ]
    [  0  -1   0 ]
    [  1   0   0 ]

##### Step 1. Inverse Position Kinematics.

The form of the homogeneous transform matrix from base frame to gripper (end effector) frame reads

![transform matrix][img3]

the wrist center is then 

$R_{wc/0} = (W_x, W_y, W_z)^T = (p_x, p_y, p_z )^T - d * (r_{13}, r_{23}, r_{33})^T$

where $d = 0.303 $ is distance from wrist center to EE, read from above DH parameter table. From the position of the wrist center, we can solve the first three angles.

1. The rotation angle $\theta_1$ is the azimuth angle of the wrist center about the base frame origin.
        $tan(\theta_1) = W_y / W_x = (p_y - 0.303*r_{23})/(p_x - 0.303 * r_{13}))$
2. $\theta_2, \theta_3$ are solved by the triangle shape by $O_2-O_3-WC$
![triangle][img5]

##### Step 2. Inverse Orientation Kinematics

In the last step, the positon of wrist center and Gripper are fixed. In this step, the axis orientations of the Gripper will be fixed. The key equation is

    ROT = R0_3 * R3_6
ROT is numerically known (from setp 0). R0_3 is also known, because we have calculated the angles $\theta_1, \theta_2, \theta_3$ in step 1. The only unknowns are the angles in R3_6. With the help of the `simplify` method, R3_6 is simplified and following are found:

    R3_6[1,0] = sin(\theta_5)cos(\theta_6)
    R3_6[1,1] = -sin(\theta_5)sin(\theta_6)

    R3_6[0,2] = -sin(\theta_5)cos(\theta_4)
    R3_6[1,2] = -cos(\theta_5)
    R3_6[2,2] = sin(\theta_4)sin(\theta_5)

From which, the angles can be calculated

    \theta_4 = arctan(-R3_6[2,2] / R3_6[0,2])
    \theta_5 = arccos(-R3_6[1,2])
    \theta_6 = arctan(-R3_6[1,1] / R3_6[1,0])


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code contains two parts: forward kinematics and inverse kinematics server. To save the computing time, forward kinematics code is packed in a separate module *FK*, and is imported at the beginning of execution of *IK_server*. FK provides two functions to calculate the rotation transform in numerics. 

```python
def calc_Rotation_0_3(t1, t2, t3):
  ''' t1,t2,t3: parameters of joint 1, 2, and 3
      return:   rotation matrix from base frame to link 3
  '''
  return R0_3.subs({q1: t1, q2: t2, q3: t3})

def calc_Rotation_rpy(roll, pitch, yaw):
  ''' return:   rotation matrix from base to gripper in DH convention
  ''' 
  return ROT0_G.subs({r: roll, p: pitch, y: yaw})
```

The function `calc_Rotation_rpy` is used to calcuate the Wrist Center (WC).

```python
            R = calc_Rotation_rpy(roll, pitch, yaw)
            EE = Matrix([[px], [py], [pz]])
            WC = EE - 0.303 * R[:,2]
```
With the position of WC, the first three parameter $\theta_1, \theta_2, \theta_3$ are calcuated. Here the $\theta_2$ and $\theta_3$ are calculated using the cosine law of triangles. In practice, due to the rounding errors, it may happen that $ B > C + A $ and the three sides can not form a triangle.

![triangle][img5]

In such cases, I simply set the angle_b to $\pi$.
```python
            if side_b > side_a + side_c:
                angle_a = 0
                angle_b = pi
            else:
                angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2*side_b*side_c))
                angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2*side_a*side_c))
```


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

With the three theta's, the rotaion matrix from base to link 3 is calculated, and therefore the rotation from link 3 to link 6.

```
            R0_3 = calc_Rotation_0_3(theta1, theta2, theta3)
            R3_6 = R0_3.inv() * R
```

Comparing with the rotation matrix obtained from forward kinematic, 


    (Matrix([
    [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.054],
    [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),    1.5],
    [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),      0],
    [                                         0,                                          0,                0,      1]])

the other three parameters $\theta_4, \theta_5, \theta_6$ can be caculated.

```python
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            if sin(theta5) > 0:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            else:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
```
As shown in the below images, the solution for $\theta_5,\theta_6$ are not unique. The two solutions are equivlent for the FK.

![wrist1][img6]
![wrist1][img7]

Finally, I also include a video in the `./writeup/record1.mp4`.
![Pick and Place](./record1.mp4)

