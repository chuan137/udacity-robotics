from sympy import *
from mpmath import radians

### Your FK code here
# Create symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
r, p, y = symbols('r,p,y')

# Create Modified DH parameters
DH_Table = {
    alpha0:     0,  a0:      0,   d1:  0.75,   q1: q1,
    alpha1: -pi/2,  a1:   0.35,   d2:     0,   q2: -pi/2 + q2,
    alpha2:     0,  a2:   1.25,   d3:     0,   q3: q3,
    alpha3: -pi/2,  a3: -0.054,   d4:   1.5,   q4: q4,
    alpha4:  pi/2,  a4:      0,   d5:     0,   q5: q5,
    alpha5: -pi/2,  a5:      0,   d6:     0,   q6: q6,
    alpha6:     0,  a6:      0,   d7: 0.303,   q7:  0
}

# Define Modified DH Transformation matrix
def TF(alpha, a, d, q):
  return Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])

# Create individual transformation matrices
T0_1 = TF(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF(alpha5, a5, d6, q6).subs(DH_Table)
T6_G = TF(alpha6, a6, d7, q7).subs(DH_Table)
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

# Extract rotation matrices from the transformation matrices
R0_3 = T0_1[:3,:3] * T1_2[:3,:3] * T2_3[:3,:3]

# Roll, pictch and yaw rotation matrices
ROT_x = Matrix([[1, 0, 0],
                [0, cos(r), -sin(r)],
                [0, sin(r),  cos(r)]])
ROT_y = Matrix([[cos(p),  0, sin(p)],
                [     0,  1,      0],
                [-sin(p), 0, cos(p)]])
ROT_z = Matrix([[cos(y), -sin(y), 0],
                [sin(y),  cos(y), 0],
                [0, 0, 1]])
ROT0_G = ROT_x * ROT_y * ROT_z

# Compensate for rotation discrepancy between DH parameters and Gazebo
R_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
ROT0_G= ROT0_G * R_corr

# T3_6 = simplify(T3_4 * T4_5 * T5_6)
# print(T3_6, '')

def calc_Rotation_0_3(t1, t2, t3):
  ''' t1,t2,t3: parameters of joint 1, 2, and 3
      return:   rotation matrix from base frame to link 3
  '''
  return R0_3.subs({q1: t1, q2: t2, q3: t3})

def calc_Rotation_rpy(roll, pitch, yaw):
  ''' return:   rotation matrix from base to gripper in DH convention
  ''' 
  return ROT0_G.subs({r: roll, p: pitch, y: yaw})

def calc_Transform_0_G(t1, t2, t3, t4, t5, t6):
  ''' DH Homogeneous Transformation
  '''
  return T0_G.subs({q1: t1, q2: t2, q3: t3, q4: t4, q5: t5, q6: t6})
