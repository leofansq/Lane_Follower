from lib_can import CAN, UBYTE_ARRAY
import math

class Vehicle(object):
    """
    Class for the vehicle model and vehicle control
    """
    def __init__(self, wheel_base, width, length, can):
        """
        Init
        
        Parameters:
            wheel_base
            width
            length
            can: the CAN to send the control signal
        """
        self.wheel_base = wheel_base
        self.width = width
        self.length = length

        self.can = can
        self.can_idx = 0
        self.can.InitCAN(self.can_idx)

        self.steer = None
        
        # IDs of the CAN to control the steer
        self.steer_ctrl_id = 0x1E2
        self.steer_get_id = 0x33
    
    def steer_cal(self, curvature, dist_from_center):
        """
        Calculate the steer according to the curvature of the lane and the distance form the center
        
        Parameters:
            curvature
            dist_from_center
        """
        # Calculate the steer according to the curvature of the lane
        alpha = math.acos(self.wheel_base/curvature)
        if (math.pi/2 - alpha) > math.pi/6:
            print ("Wrong Alpha!!!!!!!!!!")
        
        self.steer = (math.pi/2 - alpha) * 18
        
        # Adjust the steer according to the distance form the center
        if abs(dist_from_center)>10:
            beta = -30*dist_from_center/80 if dist_from_center>0 else 30*dist_from_center/80
        else:
            beta = 0
        # Set the boundary
        beta = 30 if beta>30 else beta
        beta = -30 if beta<-30 else beta
        
        self.steer += beta
        
        print (self.steer)


    def steer_ctrl(self):
        """
        Control the steer by sending the signal via CAN
        """
        high = int(self.steer*10)//256
        low = int(self.steer*10)%256

        print (high, low)

        data = [UBYTE_ARRAY(72, 0, 0, high, low, 0, 0, 0)]

        self.can.Send(self.can_idx, self.steer_ctrl_id, len(data), data)
    
    def steer_get(self):
        """
        Get the real steer of the vehicle via the CAN
        """
        temp = self.can.Listen(self.can_idx, self.steer_get_id)
        steer = (int(temp[2])*256 + int(temp[3])) / 10
        
        return steer