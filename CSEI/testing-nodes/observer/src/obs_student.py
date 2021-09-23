import rospy
import numpy as np
import math
from lib import qualisys, Tau, observer, Gains, Udata
from math_tools import Rzyx


### Write your code here ###
def computeTau(u):
    lx = np.array([-0.4574, -0.4574, 0.3875])
    ly = np.array([-0.055, 0.055, 0])
    u = np.array(u)[np.newaxis].T

    B = np.array([[0, 1, 0, 1, 0], [1, 0, 1, 0, 1], [lx[2], -ly[0], lx[0], -ly[1], lx[1]]])
    K =np.array([
        [2.629, 0, 0, 0, 0],
        [0, 1.030, 0, 0, 0],
        [0, 0, 1.030, 0, 0],
        [0, 0, 0, 1.030, 0],
        [0, 0, 0, 0, 1.030]
    ])

    tau = B@K@u

    return tau

def linear_observer(eta_hat, nu_hat, bias_hat, eta, tau, L1, L2, L3):
    """
    Observer
    """
    # Hardcoded per now, working on a more dynamic solution. 
    L_1 = np.diag(L1)
    L_2 = np.diag(L2)
    L_3 = np.diag(L3)
    M = np.array([[16.11, 0.0, 0.0], [0.0, 24.11, 0.5291], [0.0, 0.5291, 2.7600]])
    D = np.array([[0.66, 0., 0.], [0., 1.3, 2.8], [0., 0., 1.9]])
    R = Rzyx(eta[2])
    M_inv = np.linalg.inv(M)
    dt = 0.01

    eta_tilde = eta - eta_hat # Error 
    eta_hat_dot = R @ nu_hat + L_1 @ eta_tilde

    nu_hat_dot = M_inv @ (-D @ nu_hat + R.T @
                        bias_hat + tau + R.T @ L_2 @ eta_tilde)
    bias_hat_dot = L_3 @ eta_tilde

    # Forward euler
    eta_hat = eta_hat + dt * eta_hat_dot
    nu_hat  = nu_hat + dt * nu_hat_dot
    bias_hat = bias_hat + dt * bias_hat_dot

    return eta_hat, nu_hat, bias_hat

### End of student code ###

def loop():
    """
    Handle all calls to self written functions and publishers in this function. It is called by the 
    script that creates the ROS node and will loop
    """
    u = Udata.getU()
    tau = computeTau(u)
    new_tau = np.zeros(3)
    new_tau[0] = tau[0][0]
    new_tau[1] = tau[1][0]
    new_tau[2] = tau[2][0]
    eta = qualisys.getQualisysOdometry()
    old_eta_hat, old_nu_hat, old_bias_hat = observer_data.get_observer_data()
    L1, L2, L3 = Gains.get_observer_gains()
    eta_hat, nu_hat, bias_hat = linear_observer(old_eta_hat, old_nu_hat, old_bias_hat, eta, new_tau, L1, L2, L3)
    observer_data.publish_observer_data(eta_hat, nu_hat, bias_hat)
    return 0