import math
import time

import cv2
import numpy as np
import roslibpy
from django.http import HttpRequest
from roslibpy import ServiceRequest

from app0.util import get_user, failed_api_response, ErrorCode
from rosPrj.settings import ROS_PORT, ROS_HOST


class ROSClient(object):
    _instance = None
    _flag = False

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not ROSClient._flag:
            ROSClient._flag = True
            self.counter = 0
            self.failed_count = 0
            self.map_id = 0
            self.user_id = 0
            self.last_op_time = time.time()
            self.client = None
            self.map_service = None
            self.current_pose_service = None
            self.main_ctrl_service = None

    def reset(self, host, port, user_id):
        self.counter = 0
        self.failed_count = 0
        self.map_id = 0
        self.user_id = user_id
        self.client = roslibpy.Ros(host=host, port=port)
        self.map_service = roslibpy.Service(self.client, "/dynamic_map", "nav_msgs/GetMap")
        self.main_ctrl_service = roslibpy.Service(self.client, "/main_ctrl", "Tus_g5/MainCtrl")
        self.current_pose_service = roslibpy.Service(self.client, "/cur_pose", "Tus_g5/PoseSrv")
        self.client.run()

    def exit(self):
        self.client.terminate()
        self.counter = 0
        self.failed_count = 0
        self.map_id = 0
        self.user_id = 0
        self.last_op_time = time.time()
        self.client = None
        self.map_service = None
        self.main_ctrl_service = None

    @property
    def is_connect(self):
        if self.client is None:
            return False
        return self.client.is_connected

    def save_map_local(self, name):
        response = self.map_service.call(ServiceRequest({}))
        width = response["map"]["info"]["height"]
        height = response["map"]["info"]["width"]
        m = response["map"]["data"]
        m = np.array(m).reshape((width, height))
        tem = np.zeros((width, height))
        for i in range(width):
            for j in range(height):
                if m[i, j] == -1:
                    tem[width - 1 - i, j] = 127
                else:
                    tem[width - 1 - i, j] = 255 - (m[i, j] * 2)
        cv2.imwrite("./{}.png".format(name), tem)

    def get_current_pose(self):
        response = self.current_pose_service(ServiceRequest({}))
        return response["pose"]

    def send_ctrl_req(self, req: dict):
        """
        msg example:
        {
            "type": 1,
            "keyboard_ctrl_msg": {
                "direction": 1,
                "speed": 0.5,
            },
            "navigation_ctrl_msg": {
                "loop": 0,
                "pose_list": [
                    {
                        "position": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        },
                        "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 0.0
                        },
                    },
                    {
                        "position": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        },
                        "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 0.0
                        },
                    },
                ]
                "name_list": [
                    "航点1",
                    "航点2",
                ]
            },
            "command": "去xxx",
        }

        :param req: dict
        :return: None
        """
        req["id"] = self.counter
        self.counter += 1
        return self.main_ctrl_service.call(ServiceRequest(req))


def ctrl_template() -> dict:
    return {
        "type": 0,
        "keyboard_ctrl_msg": {
            "direction": 0,
            "speed": 0,
        },
        "navigation_ctrl_msg": {
            "loop": 0,
            "pose_list": [],
            "name_list": [],
        },
        "command": "",
    }


def pose_template() -> dict:
    return {
        "position": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        },
        "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 0.0
        },
    }


def ros_quaternion_to_theta(quaternion: dict):
    q0 = quaternion["w"]
    q1 = quaternion["x"]
    q2 = quaternion["y"]
    q3 = quaternion["z"]
    theta = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (math.pow(q2, 2) + math.pow(q3, 2)))
    return theta


def ros_theta_to_quaternion(theta: float):
    return {
        "w": math.cos(theta / 2),
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(theta / 2),
    }


def require_ros(func):
    def wrapper(request: HttpRequest, *args, **kwargs):
        user = get_user(request)
        ros_client = ROSClient()
        if ros_client.client is None:
            ros_client.reset(host=ROS_HOST, port=ROS_PORT, user_id=user.id)
        if ros_client.client is not None and ros_client.user_id != user.id \
                and time.time() - ros_client.last_op_time < 300:
            return failed_api_response(ErrorCode.BAD_REQUEST_ERROR, "暂时无机器人可用，请稍候")
        return func(request, *args, **kwargs)

    return wrapper


def require_map_selected(func):
    def wrapper(request: HttpRequest, *args, **kwargs):
        ros_client = ROSClient()
        if ros_client.map_id == 0:
            return failed_api_response(ErrorCode.BAD_REQUEST_ERROR, "请先选择地图")
        return func(request, *args, **kwargs)

    return wrapper


def ros_wrap_point(x, y, theta):
    return {
        "position": {
            "x": x,
            "y": y,
            "z": 0.0,
        },
        "orientation": ros_theta_to_quaternion(theta),
    }
