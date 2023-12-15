from django.contrib.auth.hashers import check_password, make_password
from django.core.exceptions import ObjectDoesNotExist
from django.http import HttpRequest
from django.views.decorators.http import require_GET, require_POST

from app0.file_util import s3_download_url
from app0.models.User import User
from app0.util import require_jwt, response_wrapper, require_keys, parse_data, success_api_response, \
    failed_api_response, ErrorCode, filter_data, get_user, require_item_exist


@response_wrapper
@require_POST
@require_keys({"username", "password"})
def login(request: HttpRequest):
    """
    [POST] /api/auth/login
    """
    data = parse_data(request)
    username = data["username"]
    password = data["password"]
    try:
        user = User.objects.get(username=username)
        if not check_password(password, user.password):
            return failed_api_response(ErrorCode.BAD_REQUEST_ERROR, "密码错误")
        token = user.token
        return success_api_response({"token": token, "role": user.role, "id": user.id,
                                     "avatar": None if user.image is None else s3_download_url(user.image.oss_token)})
    except ObjectDoesNotExist:
        return failed_api_response(ErrorCode.ITEM_NOT_FOUND_ERROR, "用户不存在")


@response_wrapper
@require_POST
@require_keys({"username", "password"})
def register(request: HttpRequest):
    """
    [POST] /api/auth/register
    """
    data = parse_data(request)
    username = data["username"]
    if User.objects.filter(username=username).exists():
        return failed_api_response(ErrorCode.BAD_REQUEST_ERROR, "用户名已被注册")
    data["password"] = make_password(data["password"], None, 'pbkdf2_sha256')
    filter_data(data, {"username", "password"})
    user = User.objects.create(**data)
    return success_api_response({"id": user.id})


@response_wrapper
@require_POST
@require_jwt()
@require_keys({"origin_pwd", "new_pwd"})
def update_password(request: HttpRequest):
    """
    [POST] /api/auth/password/update
    """
    data = parse_data(request)
    origin_pwd = data["origin_pwd"]
    new_pwd = data["new_pwd"]
    user = get_user(request)
    if not check_password(origin_pwd, user.password):
        return failed_api_response(ErrorCode.BAD_REQUEST_ERROR, "原密码错误")
    user.password = make_password(new_pwd, None, 'pbkdf2_sha256')
    user.save()
    return success_api_response()


@response_wrapper
@require_GET
def check_user_name_exist(request: HttpRequest, username):
    """
    [GET] /api/auth/check_username/<str:username>
    """
    return success_api_response({"exist": User.objects.filter(username=username).exists()})


@response_wrapper
@require_jwt()
@require_GET
def get_user_detail(request: HttpRequest):
    """
    [GET] /api/user
    """
    user = get_user(request)
    return success_api_response({"username": user.username, "role": user.role, "id": user.id,
                                 "avatar": None if user.image is None else s3_download_url(user.image.oss_token)})
