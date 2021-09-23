import rospy
from lib import odometry, ps4, Udata
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

# Deafault and should always be here
def saturate(u):
    """
    Saturate ensures that the input to the actuator remains bounded to the interval [-1, 1]
    """
    if u > 1:
        u = 1
    elif u < -1:
        u = -1
    return u

def sixaxis2thruster(lStickX, lStickY, rStickX, rStickY, R2, L2):
    """
    sixaxis2thruster()directly maps the sixaxis playstation controller inputs
    to the vessel actuators.
    """
    ### Acutator commands ###
    u1 = -0.5*(L2 - R2)
    u2 = saturate(math.sqrt(lStickX ** 2  + lStickY ** 2))
    u3 = saturate(math.sqrt(rStickX ** 2 + rStickY ** 2))


    ### VSD angles as described in the handbook ###
    alpha1 = math.atan2(lStickX, lStickY)
    alpha2 = math.atan2(rStickX, rStickY)

    u = np.array([u1, u2, u3, alpha1, alpha2])
    return u

### Write your code here ###

def input_mapping(lStickX, lStickY, rStickX, rStickY, R2, L2):
    """
    This function maps the input from the joystick to corresponding deegre of
    freedom. Inputs are from controller, and returns a generelized force 
    vector tau. 
    """
    surge = (lStickY + rStickY)
    sway = (lStickX + rStickX)
    yaw = (R2 - L2)

    tau = np.array([[surge], [sway], [yaw]])
    return tau
    
def extended_thrust_allocation(tau):
    """ 
    An extended thrust algorithm
    """ 
    # Positional value for the thrusters [m]
    lx = np.array([-0.4574, -0.4574, 0.3875])
    ly = np.array([-0.055, 0.055, 0])
    u = np.zeros(5)

    B_ext = np.array([[0, 1, 0, 1, 0], [1, 0, 1, 0, 1], [lx[2], -ly[0], lx[0], -ly[1], lx[1]]])
    K =np.array([
        [2.629, 0, 0, 0, 0],
        [0, 1.030, 0, 0, 0],
        [0, 0, 1.030, 0, 0],
        [0, 0, 0, 1.030, 0],
        [0, 0, 0, 0, 1.030]
    ])

    inv_matrix = np.linalg.pinv(np.dot(B_ext, K))
    u_ext = inv_matrix @ tau
    u[0] = u_ext[0]
    u[1] = math.sqrt(u_ext[1]**2 + u_ext[2]**2)
    u[2] = math.sqrt(u_ext[3]**2 + u_ext[4]**2)
    u[3] = math.atan2(u_ext[2], u_ext[1])
    u[4] = math.atan2(u_ext[4], u_ext[3])
    return u


### End of student code ###
def loop():
    """
    Handle all calls to self written functions and publishers in this function. It is called by the 
    script that creates the ROS node and will loop
    """
    tau = input_mapping(ps4.lStickX, ps4.lStickY, ps4.rStickX, ps4.rStickY, ps4.R2, ps4.L2)
    u = extended_thrust_allocation(tau)
    Udata.publish(u)
