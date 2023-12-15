import os
import time
from enum import unique, Enum

from django.http import HttpRequest
from django.views.decorators.http import require_POST, require_GET, require_http_methods
from roslibpy.core import RosTimeoutError

from app0.file_util import get_oss_token, s3_upload_local_file, s3_download_url
from app0.models.File import File
from app0.models.Map import Map
from app0.models.Point import Point
from app0.models.User import User
from app0.ros_util import ROSClient, ctrl_template, require_ros, \
    ros_wrap_point, ros_quaternion_to_theta
from app0.util import response_wrapper, success_api_response, failed_api_response, ErrorCode, \
    require_jwt, get_user, require_keys, parse_data, filter_data, require_item_exist
from rosPrj.settings import ROS_PORT, ROS_HOST, DEBUG

DIR_MAP = {
    "r": 0,
    "w": 1,
    "s": 2,
    "a": 3,
    "d": 4,
    "q": 5,
    "e": 6
}


@unique
class CtrlType(Enum):
    STOP_FORCE = 0
    EXIT = 1

    # USER_CTRL_START = 10
    # USER_CTRL_END = 11
    USER_KEY_VEL_CMD = 12
    USER_VOICE_CMD = 13
    # USER_GOTO_POINT = 14

    MAPPING_START = 20
    MAPPING_END = 21
    MAPPING_KEY_VEL_CMD = 22
    MAPPING_SAVE_MAP = 23

    NAV_START = 30
    NAV_END = 31
    NAV_PATROL = 32
    # NAV_GOTO_POINT = 33
    NAV_STOP = 34
    # NAV_SET_CURR_POINT = 35
    # NAV_SET_POINT = 36
    # NAV_RENAME_POINT = 37
    # NAV_DELETE_POINT = 38
    # NAV_SELECT_MAP = 39


def check_connect(user: User):
    ros_client = ROSClient()
    # 如果没有 client 则创建
    if ros_client.client is None:
        try:
            ros_client.reset(host=ROS_HOST, port=ROS_PORT, user_id=user.id)
        except RosTimeoutError:
            return False
    # 如果连接失败，2s 内检查10次，roslibpy 会自动尝试重新连接
    count = 10
    while not ros_client.is_connect and count >= 0:
        time.sleep(0.2)
        count -= 1
    # 如果还是没连上，记连接失败
    if not ros_client.is_connect:
        ros_client.failed_count += 1
        # 如果3次都连接失败，则断开
        if ros_client.failed_count >= 3:
            ros_client.exit()
            return False
    ros_client.failed_count = 0
    return True


def call_ros_service(ctrl_msg, user):
    ros_client = ROSClient()
    if not check_connect(user):
        return failed_api_response(ErrorCode.ROS_CONNECT_FAILED, "与 ROS 连接失败，请检查网络")

    res = ros_client.send_ctrl_req(ctrl_msg)

    if res["code"] != 0:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, res["msg"])
    return success_api_response()


@response_wrapper
@require_jwt()
@require_GET
def ros_connect(request: HttpRequest):
    """
    [GET] /api/ros/connect
    """
    ros_client = ROSClient()
    user = get_user(request)
    if ros_client.client is None:
        ros_client.reset(host=ROS_HOST, port=ROS_PORT, user_id=user.id)
    else:
        if ros_client.user_id != user.id:
            if time.time() - ros_client.last_op_time < 300:
                return failed_api_response(ErrorCode.BAD_REQUEST_ERROR, "暂时无机器人可用，请稍候")
            else:
                try:
                    ros_client.reset(host=ROS_HOST, port=ROS_PORT, user_id=user.id)
                except RosTimeoutError:
                    failed_api_response(ErrorCode.ROS_CONNECT_FAILED, "ROS连接失败，请检查相关配置是否正确以及网络是否连通")
    flag = ros_client.user_id == user.id and check_connect(user)
    if not flag:
        return failed_api_response(ErrorCode.ROS_CONNECT_FAILED, "ROS连接失败，请检查相关配置是否正确以及网络是否连通")
    return success_api_response({"connect": True})


@response_wrapper
@require_jwt()
@require_GET
def get_ros_connect_status(request: HttpRequest):
    """
    [GET] /api/ros/connect_status
    """
    return success_api_response({"connect": ROSClient().is_connect, "user_id": ROSClient().user_id})


@response_wrapper
@require_jwt()
@require_GET
def ros_free(request: HttpRequest):
    """
    [GET] /api/ros/free
    """
    if ROSClient().client is None or ROSClient().user_id != get_user(request).id:
        return failed_api_response(ErrorCode.BAD_REQUEST_ERROR, "状态错误！")
    ROSClient().exit()
    return success_api_response()


"""==========================================  Mapping  ==============================================="""


@response_wrapper
@require_jwt(admin=True)
@require_GET
@require_ros
def mapping_start(request: HttpRequest):
    """
    [GET] /api/mapping/start
    """
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.MAPPING_START.value
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt(admin=True)
@require_GET
@require_ros
def mapping_end(request: HttpRequest):
    """
    [GET] /api/mapping/end
    """
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.MAPPING_END.value
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt(admin=True)
@require_POST
@require_keys({"direction", "speed"})
@require_ros
def mapping_move(request: HttpRequest):
    """
    [POST] /api/mapping/move
    """
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.MAPPING_KEY_VEL_CMD.value
    data = parse_data(request)
    filter_data(data, {"direction", "speed"})
    if DIR_MAP.get(data["direction"], None) is None:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "无效的direction")
    if not -1e-6 <= data["speed"] <= 0.3 + 1e-6:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "无效的speed")
    ctrl_msg["keyboard_ctrl_msg"]["direction"] = DIR_MAP[data["direction"]]
    ctrl_msg["keyboard_ctrl_msg"]["speed"] = data["speed"]
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt(admin=True)
@require_POST
@require_keys({"name"})
@require_ros
def mapping_save(request: HttpRequest):
    """
    [POST] /api/mapping/save
    """
    data = parse_data(request)
    name = data["name"]

    if Map.objects.filter(name=name).exists():
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "地图已存在")

    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.MAPPING_SAVE_MAP.value
    ctrl_msg["navigation_ctrl_msg"]["name_list"].append(name)

    res = call_ros_service(ctrl_msg, get_user(request))
    if not res["success"]:
        return res

    ros_client = ROSClient()
    ros_client.save_map_local(name=name)
    file_data = {
        "filename": "{}.png".format(name),
        "oss_token": get_oss_token(0, "{}.png".format(name)),
    }
    s3_upload_local_file("./{}.png".format(name), file_data["oss_token"])
    # do not remove for debugging
    if not DEBUG:
        os.remove("./{}.png".format(name))
    file = File.objects.create(**file_data)
    map_obj = Map.objects.create(name=name, file=file)
    return success_api_response({"id": map_obj.id, "url": s3_download_url(file.oss_token)})


"""==========================================  User Ctrl  ============================================="""


@response_wrapper
@require_jwt()
@require_POST
@require_keys({"direction", "speed"})
@require_ros
def user_ctrl_keyboard(request: HttpRequest):
    """
    [POST] /api/user_ctrl/keyboard
    """
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.USER_KEY_VEL_CMD.value
    data = parse_data(request)
    if DIR_MAP.get(data["direction"], None) is None:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "无效的direction")
    if not -1e-6 <= data["speed"] <= 0.3 + 1e-6:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "无效的speed")
    ctrl_msg["keyboard_ctrl_msg"]["direction"] = DIR_MAP[data["direction"]]
    ctrl_msg["keyboard_ctrl_msg"]["speed"] = data["speed"]
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt()
@require_POST
@require_keys({"command"})
@require_ros
def user_ctrl_command(request: HttpRequest):
    """
    [POST] /api/user_ctrl/command
    """
    data = parse_data(request)
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.USER_VOICE_CMD.value
    point_list = Point.objects.filter(mmap_id=ROSClient().map_id)
    ctrl_msg["navigation_ctrl_msg"]["pose_list"] = list(map(point2pose, point_list))
    ctrl_msg["navigation_ctrl_msg"]["name_list"] = list(map(lambda p: p.name, point_list))
    ctrl_msg["command"] = data["command"]
    return call_ros_service(ctrl_msg, get_user(request))


"""==========================================  Navigation  ============================================"""


@response_wrapper
@require_jwt()
@require_GET
@require_item_exist(Map, "id", "query_id")
@require_ros
def navigation_start(request: HttpRequest, query_id):
    """
    [GET] /api/navigation/start/<int:query_id>
    """
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.NAV_START.value
    mmap = Map.objects.get(id=query_id)
    ctrl_msg["navigation_ctrl_msg"]["name_list"].append(mmap.name)
    ros_client = ROSClient()
    ros_client.map_id = mmap.id
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt()
@require_GET
@require_ros
def navigation_end(request: HttpRequest):
    """
    [GET] /api/navigation/end
    """
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.NAV_END.value
    return call_ros_service(ctrl_msg, get_user(request))


def point_id2name(pid: int) -> str:
    return Point.objects.get(id=pid).name


def point_id2pose(pid: int) -> dict:
    point = Point.objects.get(id=pid)
    return {
        "position": {
            "x": point.px,
            "y": point.py,
            "z": point.pz,
        },
        "orientation": {
            "x": point.ox,
            "y": point.oy,
            "z": point.oz,
            "w": point.ow,
        },
    }


def point2pose(point: Point) -> dict:
    return {
        "position": {
            "x": point.px,
            "y": point.py,
            "z": point.pz,
        },
        "orientation": {
            "x": point.ox,
            "y": point.oy,
            "z": point.oz,
            "w": point.ow,
        },
    }


@response_wrapper
@require_jwt()
@require_POST
@require_keys({"id"})
@require_ros
def navigation_move(request: HttpRequest):
    """
    [POST] /api/navigation/move
    """
    data = parse_data(request)
    if not Point.objects.filter(id=data["id"]).exists():
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "航点不存在")
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.NAV_PATROL.value
    ctrl_msg["navigation_ctrl_msg"]["loop"] = 1
    ctrl_msg["navigation_ctrl_msg"]["pose_list"].append(point_id2pose(data["id"]))
    ctrl_msg["navigation_ctrl_msg"]["name_list"].append(point_id2name(data["id"]))
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt()
@require_POST
@require_keys({"path", "loop"})
@require_ros
def navigation_patrol(request: HttpRequest):
    """
    [POST] /api/navigation/patrol
    """
    data = parse_data(request)
    for pid in data["path"]:
        if not Point.objects.filter(id=pid).exists():
            return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "航点(ID:{})不存在".format(pid))
    path_name = list(map(point_id2name, data["path"]))
    path_pose = list(map(point_id2pose, data["path"]))
    loop = data["loop"]
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.NAV_PATROL.value
    ctrl_msg["navigation_ctrl_msg"]["loop"] = loop
    ctrl_msg["navigation_ctrl_msg"]["pose_list"] = path_pose
    ctrl_msg["navigation_ctrl_msg"]["name_list"] = path_name
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt()
@require_GET
@require_ros
def navigation_stop(request: HttpRequest):
    """
    [GET] /api/navigation/stop
    """
    ctrl_msg = ctrl_template()
    ctrl_msg["type"] = CtrlType.NAV_STOP.value
    return call_ros_service(ctrl_msg, get_user(request))


@response_wrapper
@require_jwt()
@require_POST
@require_keys({"name"})
@require_ros
def navigation_mark_current_point(request: HttpRequest):
    """
    [POST] /api/navigation/mark_curr
    """
    data = parse_data(request)
    ros_client = ROSClient()
    if Point.objects.filter(mmap_id=ros_client.map_id, name=data["name"]).exists():
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "该航点已存在")

    res = ros_client.get_current_pose()

    point = Point.objects.create(
        mmap_id=ros_client.map_id,
        name=data["name"],
        px=res["position"]["x"],
        py=res["position"]["y"],
        pz=res["position"]["z"],
        ox=res["orientation"]["x"],
        oy=res["orientation"]["y"],
        oz=res["orientation"]["z"],
        ow=res["orientation"]["w"],
    )
    return success_api_response({"id": point.id})


@response_wrapper
@require_jwt()
@require_POST
@require_keys({"name", "x", "y", "theta"})
@require_item_exist(Map, "id", "query_id")
def navigation_mark_point(request: HttpRequest, query_id):
    """
    [POST] /api/navigation/mark/<int:query_id>
    """
    data = parse_data(request)
    name = data["name"]
    if Point.objects.filter(mmap_id=query_id, name=name).exists():
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "该航点已存在")
    filter_data(data, {"x", "y", "theta"})
    pose = ros_wrap_point(**data)
    point = Point.objects.create(
        mmap_id=query_id,
        name=name,
        px=pose["position"]["x"],
        py=pose["position"]["y"],
        pz=pose["position"]["z"],
        ox=pose["orientation"]["x"],
        oz=pose["orientation"]["z"],
        oy=pose["orientation"]["y"],
        ow=pose["orientation"]["w"],
    )
    return success_api_response({"id": point.id})


@response_wrapper
@require_jwt()
@require_POST
@require_keys({"id", "name"})
def navigation_rename(request: HttpRequest):
    """
    [POST] /api/navigation/rename
    """
    data = parse_data(request)
    if not Point.objects.filter(id=data["id"]).exists():
        return failed_api_response(ErrorCode.ITEM_NOT_FOUND_ERROR, "航点id无效")
    if Point.objects.filter(name=data["name"]).exists():
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGUMENT_ERROR, "该航点已存在")
    point = Point.objects.filter(id=data["id"]).first()
    point.name = data["name"]
    point.save()
    return success_api_response()


@response_wrapper
@require_jwt()
@require_http_methods(["DELETE"])
@require_item_exist(Point, "id", "query_id")
def navigation_delete(request: HttpRequest, query_id):
    """
    [DELETE] /api/navigation/delete/<int:query_id>
    """
    point = Point.objects.get(id=query_id)
    point.delete()
    return success_api_response()


def map_to_dict(mmap: Map) -> dict:
    return {
        "id": mmap.id,
        "name": mmap.name,
        "url": s3_download_url(mmap.file.oss_token),
        "x": mmap.x,
        "y": mmap.y,
    }


@response_wrapper
@require_jwt()
@require_GET
def navigation_map_list(request: HttpRequest):
    """
    [GET] /api/navigation/map_list
    """
    data = {"maps": list(map(map_to_dict, Map.objects.all()))}
    return success_api_response(data)


@response_wrapper
@require_jwt()
@require_http_methods(["DELETE"])
@require_item_exist(Map, "id", "query_id")
def navigation_map_delete(request: HttpRequest, query_id):
    """
    [GET] /api/navigation/map/delete/<int:query_id>
    """
    Map.objects.get(id=query_id).delete()
    return success_api_response()


def point_to_dict(point: Point) -> dict:
    return {
        "id": point.id,
        "name": point.name,
        "x": point.px,
        "y": point.py,
        "theta": ros_quaternion_to_theta({
            "w": point.ow,
            "x": 0,
            "y": 0,
            "z": point.oz,
        })
    }


@response_wrapper
@require_jwt()
@require_GET
@require_item_exist(Map, "id", "query_id")
def navigation_point_list(request: HttpRequest, query_id):
    """
    [GET] /api/navigation/point_list/<query_id>
    """
    data = {"points": list(map(point_to_dict, Point.objects.filter(mmap_id=query_id)))}
    return success_api_response(data)


@response_wrapper
@require_POST
@require_keys({"x", "y", "name"})
def update_map_origin(request: HttpRequest):
    """
    [POST] /api/mapping/origin
    """
    data = parse_data(request)
    if not Map.objects.filter(name=data["name"]).exists():
        return failed_api_response(ErrorCode.ITEM_NOT_FOUND_ERROR, "对象不存在")
    mmap = Map.objects.filter(name=data["name"]).first()
    mmap.x = data["x"]
    mmap.y = data["y"]
    mmap.save()
    return success_api_response()
