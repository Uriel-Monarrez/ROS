#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse

class ControllersConnection():
    
    def __init__(self, namespace):

        self.switch_service_name = '/'+namespace+'/controller_manager/switch_controller'
        self.switch_service = rospy.ServiceProxy(self.switch_service_name, SwitchController)

    def switch_controllers(self, controllers_on, controllers_off, strictness=1):
        """
        Give the controllers you wan to switch on or off.
        :param controllers_on: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :param controllers_off: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        
        rospy.wait_for_service(self.switch_service_name)
        
        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.start_controllers = controllers_off
            switch_request_object.strictness = strictness

            switch_result = self.switch_service(switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            rospy.logdebug("Switch Result==>"+str(switch_result.ok))

            return switch_result.ok

        except rospy.ServiceException as e:
            print (self.switch_service_name+" service call failed")

            return None

    def reset_controllers(self, controllers_reset):
        """
        We turn on and off the given controllers
        :param controllers_reset: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        reset_result = False
        
        result_off_ok = self.switch_controllers(controllers_on = [],
                                controllers_off = controllers_reset)
        
        if result_off_ok:
            
            result_on_ok = self.switch_controllers(controllers_on=controllers_reset,
                                                    controllers_off=[])
            
            if result_on_ok:
                
                rospy.logdebug("Controllers Reseted==>"+str(controllers_reset))
                reset_result = True
            else:
                rospy.logdebug("result_on_ok==>" + str(result_on_ok))
        else:
            rospy.logdebug("result_off_ok==>" + str(result_off_ok))
        
        return reset_result

    def reset_monoped_joint_controllers(self):
        
        controllers_reset = ['joint_state_controller',
                             'j_pelvis_r_position_controller',
                             'j_thigh1_r_position_controller',
                             'j_thigh2_r_position_controller',
                             'j_tibia_r_position_controller',
                             'j_ankle1_r_position_controller',
                             'j_ankle2_r_position_controller']
        
        self.reset_controllers(controllers_reset)
        