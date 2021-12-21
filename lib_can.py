# encoding: utf-8
from ctypes import *

canDLL = cdll.LoadLibrary('./DLL/libcontrolcan.so')

#####################################################
###                  DATA STRUCT                  ###
#####################################################

class VCI_INIT_CONFIG(Structure):  
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)
                ]  

class VCI_CAN_OBJ(Structure):  
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)
                ] 

UBYTE_ARRAY = c_ubyte*8
UBYTE_ARRAY3 = c_ubyte*3

#####################################################
###                 CLASS for CAN                 ###
#####################################################

class  CAN(object):
    """
    Class for CAN
    """
    def  __init__(self, device_type=4, device_idx=0):
        """
        Init

        Parameters:
            device_type:  defalut: VCI_USBCAN2 = 4
            device_id: default 0
        """
        self.device_type = device_type
        self.device_idx = device_idx

        self.initconfig = byref(VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x1C, 0)) #波特率500k
        self.reserved = UBYTE_ARRAY3(0, 0 , 0)

        self.OpenDevice()

    def OpenDevice(self):
        """
        Open the CAN device
        """
        ret = canDLL.VCI_OpenDevice(self.device_type,  self.device_idx, 0)
        if ret == True:
            print('Device {} Open Success \r\n'.format(self.device_idx))
        else:
            print('Device {} Open FAILED \r\n'.format(self.device_idx))
  
    def InitCAN(self, can_idx=0):
        """
        Init the CAN
        
        Parameters:
            can_idx: default 0
        """
        ret = canDLL.VCI_InitCAN(self.device_type, self.device_idx, can_idx, self.initconfig)
        if ret == True:
            print('CAN {} Init Success \r\n'.format(can_idx))
            self.StartCan(can_idx)
        else:
            print('CAN {} Init FAILED \r\n'.format(can_idx))
   
    def StartCan(self, can_idx=0):
        """
        Start the CAN
        
        Parameters:
            can_idx: default 0
        """
        ret = canDLL.VCI_StartCAN(self.device_type, self.device_idx, can_idx)
        if ret == True:
            print('CAN {} Start Success \r\n'.format(can_idx))
        else:
            print('CAN {} Start  FAILED \r\n'.format(can_idx))
 
    def Send(self, can_idx, id, frame_len, data):
        """
        Send messages to CAN
        
        Parameters:
            can_idx: start from 0
            id: the id to send to
            frame_len: the length of the frame
            data: the data to be sent [[], [], []]
        """
        for i in range(frame_len):
            d = data[i]
            print (d)
            obj = VCI_CAN_OBJ(id, 0, 0, 1, 0, 0,  8, d, self.reserved)
            print (list(obj.Data))

            # Send the message
            ret = canDLL.VCI_Transmit(self.device_type, self.device_idx, can_idx, byref(obj), 1)

            if ret == True:
                print('CAN {} Transemit Success \r\n'.format(can_idx))
            else:
                print('CAN {} Transemit FAILED \r\n'.format(can_idx))

    def Listen(self, can_idx, id, try_cnt=10):
        """
        Receive messages from CAN
        
        Parameters:
            can_idx: start from 0
            id: the id to be listen to
        """

        temp = UBYTE_ARRAY(0, 0, 0, 0, 0, 0, 0, 0)
        obj = VCI_CAN_OBJ(id, 0, 0, 0, 0, 0,  0, temp, self.reserved)

        ret = canDLL.VCI_Receive(self.device_type, self.device_idx, can_idx, byref(obj), 2500, 0)

        t = 0
        while ret <= 0 and t<=try_cnt:
            ret = canDLL.VCI_Receive(self.device_type, self.device_idx, can_idx, byref(obj), 2500, 0)
            t += 1

        if ret > 0:
            print('CAN {} Receive Success \r\n'.format(can_idx))
            print('ID:')
            print(obj.ID)
            print('DataLen:')
            print(obj.DataLen)
            print('Data:')
            print(list(obj.Data))
        
        return  list(obj.Data)

    def CloseDevice(self):
        """
        Close the CAN device
        """
        canDLL.VCI_CloseDevice(self.device_type, self.device_idx) 
        print ("Device {} Closed!".format(self.device_idx))

