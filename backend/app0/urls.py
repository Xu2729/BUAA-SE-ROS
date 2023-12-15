from django.urls import path

from app0.api.auth import login, register, update_password, get_user_detail, check_user_name_exist
from app0.api.face import upload_face, recognition_image, delete_face, get_face_list, get_log_list, test_file_upload, \
    test_post, get_latest_log
from app0.api.file import upload_user_image
from app0.api.ros import mapping_save, ros_connect, get_ros_connect_status, ros_free, \
    user_ctrl_keyboard, user_ctrl_command, mapping_start, mapping_end, mapping_move, navigation_start, \
    navigation_end, navigation_move, navigation_patrol, navigation_stop, navigation_mark_current_point, \
    navigation_mark_point, navigation_rename, navigation_delete, navigation_map_list, \
    navigation_point_list, navigation_map_delete, update_map_origin
from app0.views import test

urlpatterns = [
    # for test
    path("test", test),

    # auth and user
    path("auth/login", login),
    path("auth/register", register),
    path("auth/password/update", update_password),
    path("auth/check_username/<str:username>", check_user_name_exist),
    path("image/user", upload_user_image),
    path("user", get_user_detail),

    # ros connect
    path("ros/connect", ros_connect),
    path("ros/connect_status", get_ros_connect_status),
    path("ros/free", ros_free),

    # user ctrl
    path("user_ctrl/keyboard", user_ctrl_keyboard),
    path("user_ctrl/command", user_ctrl_command),

    # mapping
    path("mapping/start", mapping_start),
    path("mapping/end", mapping_end),
    path("mapping/move", mapping_move),
    path("mapping/save", mapping_save),

    # navigation
    path("navigation/start/<int:query_id>", navigation_start),
    path("navigation/end", navigation_end),
    path("navigation/move", navigation_move),
    path("navigation/patrol", navigation_patrol),
    path("navigation/stop", navigation_stop),
    path("navigation/mark_curr", navigation_mark_current_point),
    path("navigation/mark/<int:query_id>", navigation_mark_point),
    path("navigation/rename", navigation_rename),
    path("navigation/delete/<int:query_id>", navigation_delete),
    path("navigation/map_list", navigation_map_list),
    path("navigation/point_list/<int:query_id>", navigation_point_list),
    path("navigation/map/delete/<int:query_id>", navigation_map_delete),

    # face
    path("face/upload/<str:name>", upload_face),
    path("face/delete/<int:query_id>", delete_face),
    path("face/list", get_face_list),
    path("log/list", get_log_list),
    path("log/latest", get_latest_log),

    # detect
    path("detect/upload", recognition_image),

    # test
    path("test/file/upload", test_file_upload),
    path("test/post", test_post),

    # mapping origin
    path("mapping/origin", update_map_origin),
]
