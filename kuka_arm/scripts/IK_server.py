#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Problem/Code Completed by Mark Fobert.

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def get_homogenious_tranform_DH(alpha_i_minus_1, a_i_minus_1, d_i, theta_i):
	return Matrix([
				   [cos(theta_i)						, -sin(theta_i)						,	0						,a_i_minus_1				],
				   [sin(theta_i)*cos(alpha_i_minus_1) 	, cos(theta_i)*cos(alpha_i_minus_1)	,  -sin(alpha_i_minus_1)	,-sin(alpha_i_minus_1)*d_i	], 
				   [sin(theta_i)*sin(alpha_i_minus_1) 	, cos(theta_i)*sin(alpha_i_minus_1)	,  cos(alpha_i_minus_1)		,cos(alpha_i_minus_1)*d_i	], 
				   [0									, 0									,  0						,1							]
				  ])


def rotation_matrix_to_hom_transform(rotation_matrix):
    #make a copy of input matrix
    tmp_matrix = rotation_matrix[:,:]
    
    #insert a column
    insert_col = Matrix([[0],[0],[0]])
    tmp_matrix = tmp_matrix.col_insert(3,insert_col)
    
    #insert a row
    insert_row = Matrix([[0,0,0,1]])
    tmp_matrix = tmp_matrix.row_insert(3,insert_row)
    
    return tmp_matrix
    


### Define functions for Rotation Matrices about x, y, and z given specific angle.
def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(q),  -sin(q)],
                  [ 0,         sin(q),  cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [      0,        1,       0],
                  [-sin(q),        0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[ cos(q),  -sin(q),       0],
                  [ sin(q),   cos(q),       0],
                  [      0,        0,       1]])
    
    return R_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

	    #define twist angle symbols
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6= symbols('alpha0:7')

	    #define link length symbols
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

	    #define link offset symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	
	    #define joint angle sumbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	
	    ##define the DH table dictionary - see writeup.md for derivation
        dh = { alpha0: 0        , a0:	0       , d1: 0.75      , q1: q1, 
		       alpha1: -pi/2. 	, a1:	0.35	, d2: 0			, q2: q2-pi/2.0,
		       alpha2: 0 		, a2:	1.25	, d3: 0			, q3: q3,
		       alpha3: -pi/2. 	, a3:	-0.054	, d4: 1.5		, q4: q4,
		       alpha4: pi/2. 	, a4:	0		, d5: 0			, q5: q5,
		       alpha5: -pi/2. 	, a5:	0		, d6: 0			, q6: q6,
		       alpha6: 0 		, a6:	0		, d7: 0.303		, q7: 0
		    }
	
	    # Create individual transformation matrices
        T0_1 = get_homogenious_tranform_DH(alpha0, a0, d1, q1).subs(dh)
        T1_2 = get_homogenious_tranform_DH(alpha1, a1, d2, q2).subs(dh)
        T2_3 = get_homogenious_tranform_DH(alpha2, a2, d3, q3).subs(dh)
        T3_4 = get_homogenious_tranform_DH(alpha3, a3, d4, q4).subs(dh)
        T4_5 = get_homogenious_tranform_DH(alpha4, a4, d5, q5).subs(dh)
        T5_6 = get_homogenious_tranform_DH(alpha5, a5, d6, q6).subs(dh)
        T6_EE = get_homogenious_tranform_DH(alpha6, a6, d7, q7).subs(dh)

        #Get transformation from the base link to the end effector with a chain of homogenious transforms
        T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE

	    # Get correction rotation matrix to account for the difference of the gripper link
        # as defined in URDF vs the DH parameters.

        Rz = rotation_matrix_to_hom_transform(rot_z(pi))
        Ry = rotation_matrix_to_hom_transform(rot_y(-pi/2.))
        
        R_correction = Rz*Ry

        #get total transform from robot base to gripper (Wrist Center - WC), including correction
        T_total = T6_EE*R_correction

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            
            # get error matrix which transforms from URDF space to DH parameter space
            ROT_EE_error = rot_z(pi)*rot_y(-pi/2.)
            
            #get the full desired rotation/orientation of the end effector based on input values
            r, p , y = symbols('r p y')
            ROT_EE = simplify(rot_z(y)*rot_y(p)*rot_x(r)*ROT_EE_error)
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
            
            #get the end effector desired coordinates given in input
            EE = Matrix([   [px],
                            [py],
                            [pz]
                        ])

            #get joint angles - see readme/writeup for in depth derivation
            
            #find the coordiantes of the wrist center based on an offset of .303 (d6) along the z direction 
            #of the end effectors transformation matrix
            WC = EE-(0.303)*ROT_EE[:,2]
            wc_x = WC[0]
            wc_y = WC[1]
            wc_z = WC[2]

            theta_1 = atan2(wc_y,wc_x)
            
            #For visualiztion of A, B, C, a,b,c see readme/writeup
            A = 1.501 #sqrt(w)
            B = sqrt((sqrt(wc_x**2 + wc_y**2) - 0.35)**2 + (wc_z - 0.75)**2)
            C = 1.25

            a = acos((B**2 + C**2 -A**2) / (2 * B * C)) #cosine law
            b = acos((A**2 + C**2 -B**2) / (2 * A * C))
            c = acos((A**2 + B**2 -C**2) / (2 * A * B))
            
            #0.75 and 0.35 are offsets based on the fact the diagram (readme is not started at the origin/base frame)
            theta_2 = pi/2. - a - atan2(wc_z - 0.75, sqrt(wc_x**2 + wc_y**2) - 0.35)
            theta_3 = pi/2. - (b + 0.036)
            
            #rotation for first 3 joints - use forward kinematics based on thetas previously calculated
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta_1, q2: theta_2, q3: theta_3})
            
            #and last joints - find the full rotation matrix from joints 3, 6
            R3_6 = R0_3.transpose() * ROT_EE
            pprint(R3_6)            

            #solve individual equiations for rotations of each wrist joint:
            theta_4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta_5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] +R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta_6 = atan2(-R3_6[1,1], R3_6[1,0])
               
            # send response joint angles in the joint_trajectory_point variable
            joint_trajectory_point.positions = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
            joint_trajectory_list.append(joint_trajectory_point)
       
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
