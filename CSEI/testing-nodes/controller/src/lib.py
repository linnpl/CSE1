import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import numpy as np
import dynamic_reconfigure.client
from std_msgs.msg import Float64MultiArray, Float64
from messages.msg import observer_message, reference_message



class controller():
    """
    The controller listens to the /joy topic and maps all input signals from the DS4 to a variable that can be called
    """
    def __init__(self):
        self.x = self.square = self.circle = self.triangle = self.rightArrow = self.leftArrow = self.upArrow = self.DownArrow = self.L1 = self.R1 = self.L2 = self.R2 = self.L3 = self.R3 = self.share = self.options = self.PS = self.pad = 0
        self.lStickX = self.lStickY = self.rStickX = self.rStickY = self.L2A = self.R2A = 0.0

    def updateState(self, data):
        self.x = data.buttons[3]
        self.square = data.buttons[0]
        self.circle = data.buttons[2]
        self.triangle = data.buttons[1]
        self.rightArrow = data.buttons[16]
        self.leftArrow = data.buttons[14]
        self.upArrow = data.buttons[15]
        self.DownArrow = data.buttons[17]
        self.L1 = data.buttons[4]
        self.R1 = data.buttons[6]
        self.L2 = data.buttons[5]
        self.R2 = data.buttons[7]
        self.L3 = data.buttons[12]
        self.R3 = data.buttons[13]
        self.options = data.buttons[9]
        self.share = data.buttons[8]
        self.PS = data.buttons[10]
        self.pad = data.buttons[11]

        self.lStickX = -data.axes[0]
        self.lStickY = data.axes[1]
        self.rStickX = -data.axes[2]
        self.rStickY = data.axes[3]
        self.L2A = data.axes[4]
        self.R2A = data.axes[5]
    
class UVector():
    """
    The UVector initializing and publishing the computed actuator commands
    """
    def __init__(self):
        #self.leftRotorThrust = 0.0
        #self.rightRotorThrust = 0.0
        #self.bowRotorThrust = 0.0
        #self.leftRotorAngle = 0.0
        #self.rightRotorAngle = 0.0
        self.Udata = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub = rospy.Publisher('CSEI/u', Float64MultiArray, queue_size = 1)
        self.message = Float64MultiArray()

    def publish(self, data):
        self.message.data = data
        self.pub.publish(self.message)

    

class Observer_Listener():
    """
    The Observer_Listener object listens to the CSEI/observer topic. 
    """
    # Initialize position in [0; 0; 0]
    def __init__(self):
        self.observer_msg = observer_message()
        self.eta_hat = np.array([0, 0, 0])[np.newaxis].T
        self.nu_hat = np.array([0, 0, 0])[np.newaxis].T
        self.bias_hat = np.array([0, 0, 0])[np.newaxis].T

    # Callback function is called when the topic is updated
    def callback(self, msg): 
        self.eta_hat = np.array(msg.eta)[np.newaxis].T
        self.nu_hat = np.array(msg.nu)[np.newaxis].T
        self.bias_hat = np.array(msg.bias)[np.newaxis].T

    # Fetches data of observer. Dimensions are 3x1 of vectors
    def get_observer_data(self):
        return self.eta_hat, self.nu_hat, self.bias_hat

class Reference_Converser():
    """
    The reference converser listens and publishes to the CSEI/reference topic
    """
    # Initialize the guidance parameters in [0; 0; 0]
    def __init__(self):
        self.ref_msg = reference_message()
        self.pub = rospy.Publisher('/CSEI/reference/', reference_message, queue_size=1)
        self.eta_d = np.array([0, 0, 0])[np.newaxis].T
        self.nu_d = np.array([0, 0, 0])[np.newaxis].T
        self.nu_d_dot = np.array([0, 0, 0])[np.newaxis].T

    # Callback function is called when the topic is updated
    def callback(self, msg):
        self.eta_d = np.array(msg.eta_d)[np.newaxis].T
        self.nu_d = np.array(msg.nu_d)[np.newaxis].T
        self.nu_d_dot = np.array(msg.nu_d_dot)[np.newaxis].T

    # Publishes new gains to the reference topic. These should be numpy arrays with n=3
    def publish(self, eta_d, nu_d, nu_dot_d):
        self.ref_msg.eta_d = eta_d
        self.ref_msg.nu_d = nu_d
        self.ref_msg.nu_d_dot = nu_dot_d
        self.pub.publish(self.ref_msg)

    # Retrieve the references from the object 
    def get_ref(self):
        return self.eta_d, self.nu_d, self.nu_d_dot
    
class Controller_Gains():
    """
    Controller gains retrieves the parameters from the dynamic_reconfigure server.
    """
    # Initialize all gains to zero
    def __init__(self):
        self.Kp = np.zeros(3)
        self.Kd = np.zeros(3)
        self.Ki = np.zeros(3)
        self.mu = 0
        self.Uref = 0

    # Retrieves the gaines 
    def get_data(self):
        return self.Kp, self.Kd, self.Ki, self.mu, self.Uref

    # Updates gains everytime the parameters are tuned
    def callback(self, config):
        self.Kp = self.string2array(config.Kp)
        self.Kd = self.string2array(config.Kd)
        self.Ki = self.string2array(config.Ki)
        self.mu = config.mu
        self.Uref = config.U_ref

    # dynamic_reconfigure does not handle arrays, so gains like L1 or KP are strings on the form "x11,x12,x13"
    # the server to limit the number of variables. This function converts 
    # the string into a numpy array when they are retrieved. Very scuffed :^)

    def string2array(self, string): 
        return np.array(list(map(float, string.split(',')))) # Not proud of this one

# Build the objects to be imported
ps4 = controller()
Udata = UVector()
odometry = Odometry()
observer = Observer_Listener()
Gains = Controller_Gains()
reference = Reference_Converser()


# Initialize controller node
def controllNodeInit():
    global node
    node = rospy.init_node('controll_node')
    rospy.Subscriber("joy", Joy, ps4.updateState)
    rospy.Subscriber("CSEI/observer", observer_message, observer.callback)
    rospu.Subscriber("CSEI/reference", reference_message, reference.callback)
    gain_client = dynamic_reconfigure.client.Client('gain_server', timeout=30, config_callback = Gains.callback)

# Destroy node when prompted
def nodeEnd():
    global node
    node.destroy_node()

