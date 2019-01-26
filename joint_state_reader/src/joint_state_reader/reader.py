#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy                

from sensor_msgs.msg import JointState

class JointStateReader(object):         

    data = None

    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                
    def __init__(self):  
        self._sub = rospy.Subscriber("joint_states", JointState, self.update_data, queue_size=10)                                                                          
        pass     

    def update_data(self, new_data):
        self.data = new_data
                                                                                                       
    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """         
        curr_data = self.data
        index = curr_data.name.index(name)
        return curr_data.position[index]                                                                                 
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """                     
        curr_data = self.data
        data_names = curr_data.name # type: List
        data_values = curr_data.position
        values = []
        for name in names:
            index = data_names.index(name)
            values.append(data_values[index])
        return values