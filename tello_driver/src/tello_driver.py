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
from sensor_msgs.msg import Image, Joy, CameraInfo, CompressedImage
from std_msgs.msg import Empty, Header
from tello_msgs.srv import TelloCommand
from tello_msgs.msg import TelloTelemetry
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
        if self.retries <= 0:
            return False

        self.start_time = time.time()
        self.end_time = None if self.timeout == None else self.start_time + self.timeout
        self.retries = self.retries - 1
        return True

    def remaining(self):
        if self.end_time == None:
            return None
        return max(0, self.end_time - time.time())

class TelloArgument(object):
    def __init__(self, name, minimum=None, maximum=None):
        self.name = name
        self.minimum = minimum
        self.maximum = maximum
    def check(self, value):
        if self.minimum != None and float(value) < self.minimum:
            return False
        if self.maximum != None and float(value) > self.maximum:
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

        # TODO move reading part to roslaunch file 
        command_description_file = rospy.get_param("command_description", "")

        if len(command_description_file):
            self.available_commands = TelloCommandProxy.parse(command_description_file)
        if self.available_commands == None:
            rospy.logerr("Unable to find/parse command description file")
            return

        # ROS publishers
        # TODO: uncomment here
        # self._flight_data_pub = rospy.Publisher('flight_data', FlightData, queue_size=10)
        self.bridge = CvBridge()

        self.s = rospy.Service('cmd', TelloCommand, self.cmd_callback)
        self.ip = rospy.get_param("tello_ip", "192.168.10.1")
        self.command_port = rospy.get_param("tello_command_port", 8889)
        self.command_timeout = 0.5

        self.video_enabled = rospy.get_param("video_enabled", True)
        camera_info_file = rospy.get_param("camera_info", "")
        with open(camera_info_file) as f:
            camera_info = [[float(x) for x in line.strip().split()] for line in f]
            height, width = camera_info[0]
            fx, fy = camera_info[1]
            cx, cy = camera_info[2]
            self.camera_info = CameraInfo()
            self.camera_info.header = Header()
            self.camera_info.header.frame_id = "camera_frame"
            self.camera_info.height = height
            self.camera_info.width = width
            self.camera_info.distortion_model = "plumb_bob"

            self.camera_info.D = camera_info[3]
            self.camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]# mb last is fx
            self.camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            rospy.logerr("%s" % self.camera_info)

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


        rospy.Subscriber('joy', Joy, self._cmd_joy_callback)
        self.telemetry_pub = rospy.Publisher("telemetry", TelloTelemetry, queue_size=10)
        # rospy.Subscriber('takeoff', Empty, self.takeoff_callback)
        # rospy.Subscriber('land', Empty, self.land_callback)
        # rospy.Subscriber('flip', Flip, self.flip_callback)

        self.process_command("command", False)
        
        if self.video_enabled and self.process_command("streamon", False) == "ok":
            self.image_pub = rospy.Publisher('camera/image/image_raw', Image, queue_size=10)
            self.image_com_pub = rospy.Publisher('camera/image/compressed', CompressedImage, queue_size=10)
            self.camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=10)
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
        self.process_command("land", False)

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
                data = [float(item.split(":")[1]) for item in telemetry.strip()[:-1].split(";")]


                msg = TelloTelemetry()
                msg.header.stamp = rospy.Time.now()
                msg.data = data
                self.telemetry_pub.publish(msg)

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
                            msg = CompressedImage()
                            msg.header.stamp = rospy.Time.now()
                            msg.format = "jpeg"
                            msg.data = numpy.array(cv2.imencode('.jpg', cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))[1]).tostring()
                            self.image_com_pub.publish(msg)
                            self.camera_info.header.stamp = rospy.Time.now()
                            self.camera_info_pub.publish(self.camera_info)

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







    def _cmd_joy_callback(self, msg):
        pitch = -msg.axes[0] * 100
        roll = -msg.axes[1] * 100
        throttle = msg.axes[4] * 100
        yaw = msg.axes[3] * 100
        self.process_command("rc", False, pitch, roll, throttle, yaw)


if __name__ == '__main__':
    driver = TelloDriver()
