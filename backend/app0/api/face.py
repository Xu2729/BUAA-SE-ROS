import os
import time
from io import BytesIO

import cv2
import face_recognition
import numpy as np
from django.http import HttpRequest
from django.views.decorators.http import require_POST, require_http_methods, require_GET

from app0.api.fire import predict_fire_and_smoke, FIRE_DETECT, SMOKE_DETECT, NOTHING_DETECT
from app0.file_util import _validate_upload_file, get_oss_token, s3_upload_local_file, s3_download_url
from app0.models.Face import Face
from app0.models.File import File
from app0.models.Log import Log
from app0.util import response_wrapper, success_api_response, failed_api_response, ErrorCode, \
    require_jwt, validate_request, require_item_exist, parse_data
from rosPrj.settings import FACE_TOLERANCE, DEBUG, MAX_FACE_FILE_SIZE, IMAGE_HW, JPG_QUALITY, FIRE_SMOKE_CD, FACE_CD

stranger_cnt = 1
face_queue = []
last_fire = 0
last_smoke = 0


def compress_im(file, new_file, hw=IMAGE_HW, quality=JPG_QUALITY):
    img = cv2.imread(file)
    old_height = img.shape[0]
    old_width = img.shape[1]
    old_hw = old_height * old_width
    if old_hw <= hw:
        cv2.imwrite(new_file, img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return
    scale = hw / old_hw
    img = cv2.resize(img, (int(old_width * scale), int(old_height * scale)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(new_file, img, [cv2.IMWRITE_JPEG_QUALITY, quality])


def image_encoding(file: str) -> list:
    start_time = time.time()
    image = face_recognition.load_image_file(file)
    code = face_recognition.face_encodings(image, model="small")
    if DEBUG:
        print("Face encoding use %.2f ms" % (1000 * (time.time() - start_time)))
    return code


def face_cmp(faces: dict, unknown_face, tolerance=FACE_TOLERANCE):
    nt = time.time()
    while len(face_queue) > 0 and nt - face_queue[0][0] > FACE_CD:
        del face_queue[0]

    face_list = [None] * (len(faces) + len(face_queue))
    name_list = [None] * (len(faces) + len(face_queue))
    i = 0
    for k, v in faces.items():
        name_list[i] = k
        face_list[i] = v
        i += 1
    for it in face_queue:
        name_list[i] = it[1]
        face_list[i] = it[2]
        i += 1
    result = face_recognition.compare_faces(face_list, unknown_face, tolerance=tolerance)
    for i in range(len(name_list)):
        if result[i]:
            return name_list[i], i >= len(faces)
    return "", False


def load_faces_encoding() -> dict:
    faces = Face.objects.filter(strange=False)
    ret = {}
    for face in faces:
        ret[face.name] = np.load(BytesIO(face.encodings), allow_pickle=True)
    return ret


@response_wrapper
@require_jwt(admin=True)
@require_POST
@validate_request(func=_validate_upload_file)
def upload_face(request: HttpRequest, name: str):
    """
    [POST] /api/face/upload/<str:name>
    """
    file = request.FILES.get("file")
    if not file.name.endswith(".jpg"):
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "目前只支持jpg格式的图片")
    if file.size > MAX_FACE_FILE_SIZE:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "图片大小必须小于{}B".format(MAX_FACE_FILE_SIZE))

    with open("tmp.jpg", "wb") as f:
        for line in file.chunks():
            f.write(line)
    new_file = "{}.jpg".format(name)
    compress_im("tmp.jpg", "tmp_compressed.jpg")

    encoding = image_encoding("tmp_compressed.jpg")
    if len(encoding) != 1:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "图片中未检测到人脸或检测到多张人脸")

    file_data = {
        "filename": new_file,
        "oss_token": get_oss_token(0, new_file),
    }
    s3_upload_local_file("tmp_compressed.jpg", file_data["oss_token"])
    if not DEBUG:
        os.remove("tmp_compressed.jpg")

    file_obj = File.objects.create(**file_data)
    face_obj = Face.objects.create(name=name, file=file_obj, strange=False, encodings=encoding[0].dumps())
    return success_api_response({"id": face_obj.id})


@response_wrapper
@require_jwt(admin=True)
@require_http_methods(["DELETE"])
@require_item_exist(Face, "id", "query_id")
def delete_face(request: HttpRequest, query_id):
    """
    [DELETE] /api/face/delete/<int:query_id>
    """
    Face.objects.get(id=query_id).delete()
    return success_api_response()


def face_to_dict(face: Face) -> dict:
    return {
        "id": face.id,
        "name": face.name,
        "url": s3_download_url(face.file.oss_token),
    }


@response_wrapper
@require_jwt(admin=True)
@require_GET
def get_face_list(request: HttpRequest):
    """
    [GET] /api/face/list
    """
    faces = Face.objects.filter(strange=False)
    return success_api_response({"faces": list(map(face_to_dict, faces))})


@response_wrapper
@require_POST
@validate_request(func=_validate_upload_file)
def test_file_upload(request: HttpRequest):
    """
    [POST] /api/test/file/upload
    """
    file = request.FILES.get("file")
    if not file.name.endswith(".jpg"):
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "目前只支持jpg格式的图片")
    if file.size > MAX_FACE_FILE_SIZE:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "图片大小必须小于{}B".format(MAX_FACE_FILE_SIZE))
    with open("unknown.jpg", "wb") as f:
        for line in file.chunks():
            f.write(line)
    compress_im("unknown.jpg", "unknown_compressed.jpg")
    file_data = {
        "filename": "warning.jpg",
        "oss_token": get_oss_token(0, "warning.jpg"),
    }
    s3_upload_local_file("unknown_compressed.jpg", file_data["oss_token"])
    file_obj = File.objects.create(**file_data)
    return success_api_response({"url": s3_download_url(file_obj.oss_token)})


@response_wrapper
@require_POST
def test_post(request: HttpRequest):
    """
    [GET] /api/test/post
    """
    data = parse_data(request)
    return success_api_response(data)


@response_wrapper
@require_POST
@validate_request(func=_validate_upload_file)
def recognition_image(request: HttpRequest):
    """
    [POST] /api/detect/upload
    """
    global last_fire, stranger_cnt, last_smoke
    file = request.FILES.get("file")
    if not file.name.endswith(".jpg"):
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "目前只支持jpg格式的图片")
    if file.size > MAX_FACE_FILE_SIZE:
        return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "图片大小必须小于{}B".format(MAX_FACE_FILE_SIZE))

    result_list = []
    with open("unknown.jpg", "wb") as f:
        for line in file.chunks():
            f.write(line)
    compress_im("unknown.jpg", "unknown_compressed.jpg")

    flag = True
    log_id = None
    fsr = predict_fire_and_smoke("unknown_compressed.jpg", DEBUG)
    if fsr != NOTHING_DETECT:
        # 超过冷却写日志
        if (fsr == FIRE_DETECT and (last_fire == 0 or time.time() - last_fire > FIRE_SMOKE_CD)) or \
                (fsr == SMOKE_DETECT and (last_smoke == 0 or time.time() - last_smoke > FIRE_SMOKE_CD)):
            if fsr == FIRE_DETECT:
                last_fire = time.time()
            else:
                last_smoke = time.time()
            file_data = {
                "filename": "warning.jpg",
                "oss_token": get_oss_token(0, "warning.jpg"),
            }
            s3_upload_local_file("unknown_compressed.jpg", file_data["oss_token"])
            file_obj = File.objects.create(**file_data)
            log = Log.objects.create(detail="火灾" if fsr == FIRE_DETECT else "烟雾", file=file_obj)
            log_id = log.id
        # 如果检测到火灾烟雾，则不检测人脸
    else:
        codes = image_encoding("unknown_compressed.jpg")
        face_dict = load_faces_encoding()
        log_flag = False
        for code in codes:
            result, sf = face_cmp(face_dict, code)
            if len(result) == 0:
                # 检测到新的陌生人，写日志，维护队列
                flag = False
                log_flag = True
                stranger_name = "stranger_{}".format(stranger_cnt)
                result_list.append(stranger_name)
                face_queue.append((time.time(), stranger_name, code))
                stranger_cnt += 1
            elif sf:
                # 检测到最近的陌生人，不写日志，不维护队列
                flag = False
                result_list.append(result)
            else:
                # 熟人
                result_list.append(result)

        # 存在新的陌生人
        if log_flag:
            file_data = {
                "filename": "warning.jpg",
                "oss_token": get_oss_token(0, "warning.jpg"),
            }
            s3_upload_local_file("unknown_compressed.jpg", file_data["oss_token"])
            file_obj = File.objects.create(**file_data)
            log = Log.objects.create(detail="陌生人入侵，检测到：{}".format("、".join(result_list)), file=file_obj)
            log_id = log.id

    return success_api_response({"face_results": result_list, "stranger": not flag, "log_id": log_id,
                                 "fire": fsr == FIRE_DETECT, "smoke": fsr == SMOKE_DETECT})


# deprecated
# @response_wrapper
# @require_POST
# @validate_request(func=_validate_upload_file)
# def upload_fire_file(request: HttpRequest):
#     """
#     [POST] /api/fire/upload
#     """
#     if request.FILES.get("file").size > MAX_FACE_FILE_SIZE:
#         return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "图片大小必须小于{}B".format(MAX_FACE_FILE_SIZE))
#     try:
#         filename = request.FILES.get("file").name
#         data = {
#             "filename": filename,
#             "oss_token": get_oss_token(1, filename),
#         }
#         s3_upload(data["oss_token"], request)
#     except Exception as exception:
#         return failed_api_response(ErrorCode.INVALID_REQUEST_ARGS, "文件上传时出错: {}".format(str(exception)))
#     file_obj = File.objects.create(**data)
#     Log.objects.create(detail="火灾", file=file_obj)
#     return success_api_response()


def log_to_dict(log: Log) -> dict:
    return {
        "id": log.id,
        "time": log.time,
        "detail": log.detail,
        "url": s3_download_url(log.file.oss_token),
    }


@response_wrapper
@require_jwt()
@require_GET
def get_log_list(request: HttpRequest):
    """
    [GET] /api/log/list
    """
    logs = Log.objects.all().order_by("-id")
    return success_api_response({"logs": list(map(log_to_dict, logs))})


@response_wrapper
@require_jwt()
@require_GET
def get_latest_log(request: HttpRequest):
    """
    [GET] /api/log/latest
    """
    log = Log.objects.all().order_by("-id").first()
    return success_api_response({"log": log_to_dict(log)})
