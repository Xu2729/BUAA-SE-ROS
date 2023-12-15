from django.http import HttpRequest
from django.views.decorators.http import require_POST

from app0.file_util import s3_upload, _validate_upload_file, get_oss_token
from app0.models.File import File
from app0.util import response_wrapper, success_api_response, failed_api_response, ErrorCode, \
    require_jwt, validate_request, get_user


@response_wrapper
@require_jwt()
@require_POST
@validate_request(func=_validate_upload_file)
def upload_user_image(request: HttpRequest):
    """
    [POST] /api/image/user
    """
    user = get_user(request)
    try:
        filename = request.FILES.get("file").name
        data = {
            "filename": filename,
            "oss_token": get_oss_token(user.id, filename),
        }
        s3_upload(data["oss_token"], request)
    except Exception as exception:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "文件上传时出错: {}".format(str(exception)))
    file = File.objects.create(**data)
    user.image_id = file.id
    user.save()
    return success_api_response({"id": file.id})
