#!/usr/bin/env python

"""Inject an SDF or URDF file into Gazebo"""

import sys
import transformations

import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose


def inject(param_name, initial_pose):
    """Create a ROS node, and use it to call the SpawnEntity service"""

    node = rospy.init_node('inject_node')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    xml = rospy.get_param(param_name)
    rospy.logerr(xml)

    #if not client.service_is_ready():
    #    node.get_logger().info('waiting for spawn_entity service...')
    #    client.wait_for_service()

    request = SpawnModelRequest()
    request.model_name = "tello"
    request.model_xml = xml
    request.initial_pose = initial_pose
    request.robot_namespace = "tello"
    request.reference_frame = "world"
    rospy.logerr(service(request))#future = client.call_async(request)
    rospy.spin()#_until_future_complete(node, future)

    #if future.result() is not None:
    #    node.get_logger().info('response: %r' % future.result())
    #else:
    #    raise RuntimeError('exception while calling service: %r' % future.exception())

    #node.destroy_node()


if len(sys.argv) < 6:
    print('usage: ros2 run tello_gazebo inject_entity.py -- foo.urdf initial_x initial_y initial_z initial_yaw')
    sys.exit(1)

param_name = sys.argv[1]

p = Pose()
p.position.x = float(sys.argv[2])
p.position.y = float(sys.argv[3])
p.position.z = float(sys.argv[4])
q = transformations.quaternion_from_euler(0, 0, float(sys.argv[5]))
p.orientation.w = q[0]
p.orientation.x = q[1]
p.orientation.y = q[2]
p.orientation.z = q[3]

inject(param_name, p)
