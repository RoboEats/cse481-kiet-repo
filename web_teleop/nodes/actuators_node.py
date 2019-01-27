#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse, SetHead, SetHeadResponse, OpenGripper, OpenGripperResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()
        self._gripper = robot_api.Gripper()

    def handle_set_torso(self, request):
        # type: (SetTorsoRequest) -> None
        # TODO: move the torso to the requested height
        print "web_teleop/set_torso: setting height to %.2f" % request.height
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_head(self, request):
        # type: (SetHeadRequest) -> None
        print "web_teleop/set_head: setting head. pan: %.2f, tilt: %.2f" % (request.pan, request.tilt)
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse()

    def handle_set_gripper(self, request):
        print ""web_teleop/open_gripper: opening gripper"
        if (request.open):
            self._gripper.open()
        else:
            self._gripper.close(request.effort)
        return OpenGripperResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    print "web_teleop: starting service..."
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                    server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                    server.handle_set_head)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,
                                    server.handle_set_gripper)
    print "web_teleop: service running..."
    rospy.spin()


if __name__ == '__main__':
    main()