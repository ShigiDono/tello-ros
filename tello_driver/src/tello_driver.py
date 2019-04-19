#!/usr/bin/env python
import socket
import threading
import Queue
import rospy
import time
import yaml
import cv2
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tello_msgs.srv import TelloCommand
import numpy
from h264decoder.h264decoder import H264Decoder

class TelloTimeout(object):
    def __init__(self, timeout=None, retries=1, start_time=time.time()):
        self.timeout = timeout
        self.retries = retries
        self.start_time = start_time
        self.end_time = None if timeout == None else start_time + timeout
    def start(self):
        return TelloTimeout(self.timeout, self.retries)
    def retry(self):
        # rospy.logwarn("%s %s %s %s" % (self.timeout, self.retries, self.start_time, self.end_time))
        if self.retries <= 0:
            return False

        self.start_time = time.time()
        self.end_time = None if self.timeout == None else self.start_time + self.timeout
        self.retries = self.retries - 1
        return True

    def remaining(self):
        # rospy.logwarn("%s" % (self.end_time - time.time()))
        if self.end_time == None:
            return None
        # rospy.logwarn("%s" % (self.end_time - time.time()))
        return max(0, self.end_time - time.time())

class TelloArgument(object):
    def __init__(self, name, minimum=None, maximum=None):
        self.name = name
        self.minimum = minimum
        self.maximum = maximum
    def check(self, value):
        if self.minimum != None and value < self.minimum:
            return False
        if self.maximum != None and value > self.maximum:
            return False
        return True


class TelloCommandProxy(object):
    def __init__(self, cmd, arguments=[], timeout=TelloTimeout(timeout=1, retries=5), critical=False):
        self.cmd = cmd
        self.arguments = arguments
        self.timeout = timeout
        self.critical = critical

    @staticmethod
    def parse(file_name):
        with open(file_name, "r") as f:
            try:
                commands = yaml.safe_load(f)
                parsed_commands = {}
                for key, item in commands.items():
                    arguments = []
                    timeout = None
                    timeout_values = item.get("timeout", None)
                    if timeout_values != None:
                        timeout = TelloTimeout(timeout_values.get("timeout", None), timeout_values.get("retries", 1))


                    if "arguments" in item:
                        for argument in item["arguments"]:
                            arguments.append(TelloArgument(
                                argument["name"], 
                                argument.get("minimum", None), 
                                argument.get("maximum", None)
                            ))
                    parsed_commands[key] = TelloCommandProxy(item.get("cmd", key), arguments, timeout, item.get("critical", False))
                return parsed_commands
            except yaml.YAMLError as e:
                rospy.logerr('Unable to load "%s". %s' % (file_name, e))
                return None



class TelloDriver(object):
    error_command = "error_command"
    error_number_of_arguments = "error_number_of_arguments"
    error_argument = "error_argument"
    error_timeout = "timeout"
    
    def __init__(self):
        # Initialize ROS
        rospy.init_node('tello_driver_node', anonymous=False)
        self.available_commands = None #{

        command_description_file = rospy.get_param("command_description", "")

        if len(command_description_file):
            self.available_commands = TelloCommandProxy.parse(command_description_file)
        if self.available_commands == None:
            rospy.logerr("Unable to find/parse command description file")
            return

        # ROS publishers
        # self._flight_data_pub = rospy.Publisher('flight_data', FlightData, queue_size=10)
        self.bridge = CvBridge()

        # ROS subscriptions
        self.s = rospy.Service('cmd', TelloCommand, self.cmd_callback)
        self.ip = rospy.get_param("tello_ip", "192.168.10.1")
        self.command_port = rospy.get_param("tello_command_port", 8889)
        self.command_timeout = 0.5

        self.video_enabled = rospy.get_param("video_enabled", True)

        self.video_port = 11111
        self.telemetry_port = 8890

        self.command_address = (self.ip, self.command_port)
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind(('', self.command_port))
        self.command_lock = threading.Lock()

        self.response_queue = Queue.Queue()

        self.response_thread = threading.Thread(target=self._response_callback)
        self.response_thread.daemon = True
        self.response_thread.start()

        self.telemetry_address = (self.ip, self.telemetry_port)
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.bind(('', self.telemetry_port))

        self.telemetry_thread = threading.Thread(target=self._telemetry_callback)
        self.telemetry_thread.daemon = True
        self.telemetry_thread.start()


        # rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        # rospy.Subscriber('takeoff', Empty, self.takeoff_callback)
        # rospy.Subscriber('land', Empty, self.land_callback)
        # rospy.Subscriber('flip', Flip, self.flip_callback)

        # # ROS OpenCV bridge
        # self._cv_bridge = CvBridge()

        # # Connect to the drone
        # self._drone = tellopy.Tello()
        # self._drone.connect()
        # self._drone.wait_for_connection(60.0)
        # rospy.loginfo('connected to drone')

        # # Listen to flight data messages
        # self._drone.subscribe(self._drone.EVENT_FLIGHT_DATA, self.flight_data_callback)

        # # Start video thread
        # self._stop_request = threading.Event()
        # video_thread = threading.Thread(target=self.video_worker)
        # video_thread.start()

        # # Spin until interrupted
        self.process_command("command", False)
        
        if self.video_enabled and self.process_command("streamon", False) == "ok":
            self.image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
            self.video_decoder = H264Decoder()
            self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # self.video_socket.settimeout(3)
            self.video_socket.bind(('', self.video_port))
            self.video_thread = threading.Thread(target=self._video_callback)
            self.video_thread.daemon = True
            self.video_thread.start()


        r = rospy.Rate(0.2)
        
        while not rospy.is_shutdown():
            r.sleep()
            self.process_command("command", False)

        # # Force a landing
        # self._drone.land()

        # # Stop the video thread
        # self._stop_request.set()
        # video_thread.join(timeout=2)

        # # Shut down the drone
        # self._drone.quit()
        # self._drone = None
    def __del__(self):
        if hasattr(self, "command_socket"):
            self.command_socket.close()
        if hasattr(self, "video_socket"):
            self.video_socket.close()

    def _response_callback(self):
        while not rospy.is_shutdown():
            try:
                response, ip = self.command_socket.recvfrom(2048)
                rospy.loginfo(response)

                self.response_queue.put(response.strip())
            except socket.error as error:
                rospy.logerr(error)
                pass # ideally rospy.shutdown()

    def _telemetry_callback(self):
        while not rospy.is_shutdown():
            try:
                telemetry, ip = self.telemetry_socket.recvfrom(2048)
                rospy.loginfo(telemetry)
                items = {item.split(":")[0]: float(item.split(":")[1]) for item in telemetry.strip()[:-1].split(";")}

                # self.response_queue.put(telemetry.strip())
            except socket.error as error:
                rospy.logerr(error)
                pass # ideally rospy.shutdown()

    def _video_callback(self):
        packet_data = ""
        while not rospy.is_shutdown():
            try:
                video_frame, ip = self.video_socket.recvfrom(2048)

                packet_data += video_frame
                if len(video_frame) != 1460:
                    for frame in self._h264_decode(packet_data):
                        # send frames
                        try:
                            cv_image = frame
                            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))

                        except CvBridgeError as e:
                            rospy.logerr(e)
                    packet_data = ""
            except socket.error as error:
                rospy.logerr(error)

    def _h264_decode(self, packet_data):
        """
        decode raw h264 format data from Tello
        
        :param packet_data: raw h264 data array
       
        :return: a list of decoded frame
        """
        res_frame_list = []
        frames = self.video_decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
                # print 'frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls)

                frame = numpy.fromstring(frame, dtype=numpy.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, ls / 3, 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)

        return res_frame_list

    def cmd_callback(self, command):
        return self.process_command(command.cmd, False, *command.args)

    def query_callback(self, command):
        result = self.process_command(command.cmd, True, *command.args)
        if result.isdigit():
            return int(result)
        return -1

    def _send_cmd(self, cmd, *args):
        _cmd = " ".join([cmd] + [str(_) for _ in args])
        rospy.logdebug('Sending command "%s"' % (_cmd))
        self.command_socket.sendto(_cmd.encode('utf-8'), self.command_address)
        # rospy.loginfo('Sent command "%s"' % (_cmd))

    def process_command(self, cmd, query, *args):
        # check command
        with self.command_lock:
            if not cmd in self.available_commands:
                return TelloDriver.error_command
            command = self.available_commands[cmd]
            if len(command.arguments) != len(args):
                return TelloDriver.error_number_of_arguments
            for argument, description in zip(args, command.arguments):
                if not description.check(argument):
                    return TelloDriver.error_argument

            timeout = command.timeout.start()
            if query:
                cmd = cmd + "?"
            self._send_cmd(cmd, *args)
            response = None
            # TODO this part doesn't look very clean
            while timeout.retry():
                try:
                    _once = True
                    while not self.response_queue.empty() or _once:
                        _once = False
                        _timeout = timeout.remaining()
                        _block = False if _timeout == None else True
                        response = self.response_queue.get(block=_block, timeout=_timeout)
                        self.response_queue.task_done()
                    if response != None:
                        return response
                except Queue.Empty as err:
                    rospy.logwarn('No response for command "%s". Resending...' % (cmd))
                    self._send_cmd(cmd, *args)

            rospy.logerr('Timeout on command "%s".' % cmd)
            if command.critical:
                rospy.logerr("Critical command failed. Shutting down")
                rospy.signal_shutdown("Critical command failed.")
            return TelloDriver.error_timeout







    # def flight_data_callback(self, event, sender, data, **args):
    #     flight_data = FlightData()

    #     # Battery state
    #     flight_data.battery_percent = data.battery_percentage
    #     flight_data.estimated_flight_time_remaining = data.drone_fly_time_left / 10.

    #     # Flight mode
    #     flight_data.flight_mode = data.fly_mode

    #     # Flight time
    #     flight_data.flight_time = data.fly_time

    #     # Very coarse velocity data
    #     # TODO do east and north refer to the body frame?
    #     # TODO the / 10. conversion might be wrong, verify
    #     flight_data.east_speed = -1. if data.east_speed > 30000 else data.east_speed / 10.
    #     flight_data.north_speed = -1. if data.north_speed > 30000 else data.north_speed / 10.
    #     flight_data.ground_speed = -1. if data.ground_speed > 30000 else data.ground_speed / 10.

    #     # Altitude
    #     flight_data.altitude = -1. if data.height > 30000 else data.height / 10.

    #     # Equipment status
    #     flight_data.equipment = data.electrical_machinery_state
    #     flight_data.high_temperature = data.temperature_height

    #     # Some state indicators?
    #     flight_data.em_ground = data.em_ground
    #     flight_data.em_sky = data.em_sky
    #     flight_data.em_open = data.em_open

    #     # Publish what we have
    #     self._flight_data_pub.publish(flight_data)

    #     # Debugging: is there data here? Print nonzero values
    #     if data.battery_low:
    #         print('battery_low is nonzero: %d' % data.battery_low)
    #     if data.battery_lower:
    #         print('battery_lower is nonzero: %d' % data.battery_lower)
    #     if data.battery_state:
    #         print('battery_state is nonzero: %d' % data.battery_state)
    #     if data.drone_battery_left:
    #         print('drone_battery_left is nonzero: %d' % data.drone_battery_left)
    #     if data.camera_state:
    #         print('camera_state is nonzero: %d' % data.camera_state)
    #     if data.down_visual_state:
    #         print('down_visual_state is nonzero: %d' % data.down_visual_state)
    #     if data.drone_hover:
    #         print('drone_hover is nonzero: %d' % data.drone_hover)
    #     if data.factory_mode:
    #         print('factory_mode is nonzero: %d' % data.factory_mode)
    #     if data.front_in:
    #         print('front_in is nonzero: %d' % data.front_in)
    #     if data.front_lsc:
    #         print('front_lsc is nonzero: %d' % data.front_lsc)
    #     if data.front_out:
    #         print('front_out is nonzero: %d' % data.front_out)
    #     if data.gravity_state:
    #         print('gravity_state is nonzero: %d' % data.gravity_state)
    #     if data.imu_calibration_state:
    #         print('imu_calibration_state is nonzero: %d' % data.imu_calibration_state)
    #     if data.imu_state:
    #         print('imu_state is nonzero: %d' % data.imu_state)
    #     if data.outage_recording:
    #         print('outage_recording is nonzero: %d' % data.outage_recording)
    #     if data.power_state:
    #         print('power_state is nonzero: %d' % data.power_state)
    #     if data.pressure_state:
    #         print('pressure_state is nonzero: %d' % data.pressure_state)
    #     if data.throw_fly_timer:
    #         print('throw_fly_timer is nonzero: %d' % data.throw_fly_timer)
    #     if data.wind_state:
    #         print('wind_state is nonzero: %d' % data.wind_state)

    # def cmd_vel_callback(self, msg):
    #     self._drone.set_pitch(msg.linear.x)
    #     self._drone.set_roll(-msg.linear.y)     # Note sign flip
    #     self._drone.set_throttle(msg.linear.z)
    #     self._drone.set_yaw(-msg.angular.z)     # Note sign flip

    # def takeoff_callback(self, msg):
    #     self._drone.takeoff()

    # def land_callback(self, msg):
    #     self._drone.land()

    # def flip_callback(self, msg):
    #     if msg.flip_command == Flip.flip_forward:
    #         self._drone.flip_forward()
    #     elif msg.flip_command == Flip.flip_back:
    #         self._drone.flip_back()
    #     elif msg.flip_command == Flip.flip_left:
    #         self._drone.flip_left()
    #     elif msg.flip_command == Flip.flip_right:
    #         self._drone.flip_right()
    #     elif msg.flip_command == Flip.flip_forwardleft:
    #         self._drone.flip_forwardleft()
    #     elif msg.flip_command == Flip.flip_forwardright:
    #         self._drone.flip_forwardright()
    #     elif msg.flip_command == Flip.flip_backleft:
    #         self._drone.flip_backleft()
    #     elif msg.flip_command == Flip.flip_backright:
    #         self._drone.flip_backright()

    # def video_worker(self):
    #     # Get video stream, open in PyAV
    #     container = av.open(self._drone.get_video_stream())

    #     # Decode h264
    #     rospy.loginfo('starting video pipeline')
    #     for frame in container.decode(video=0):

    #         # Convert PyAV frame => PIL image => OpenCV Mat
    #         color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

    #         # Convert OpenCV Mat => ROS Image message and publish
    #         self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

    #         # Check for normal shutdown
    #         if self._stop_request.isSet():
    #             return


if __name__ == '__main__':
    driver = TelloDriver()
