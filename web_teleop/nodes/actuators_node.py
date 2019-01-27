#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()

    def handle_set_torso(self, request):
        # type: (SetTorsoRequest) -> None
        # TODO: move the torso to the requested height
        print "web_teleop/set_torso: setting height to %.2f" % request.height
        self._torso.set_height(request.height)
        return SetTorsoResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    print "web_teleop/set_torso: starting service..."
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    print "web_teleop/set_torso: service running..."
    rospy.spin()


if __name__ == '__main__':
    main()