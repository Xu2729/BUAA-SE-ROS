from datetime import timedelta, datetime

from django.http import HttpRequest, HttpResponse
from django.utils.encoding import escape_uri_path
from minio import Minio

from rosPrj.settings import S3_SSL, S3_SECRET_ID, S3_SECRET_KEY, S3_ADDRESS, S3_BUCKET_NAME

minio_client = Minio(
    S3_ADDRESS,
    access_key=S3_SECRET_ID,
    secret_key=S3_SECRET_KEY,
    secure=S3_SSL
)


def _validate_upload_file(request: HttpRequest) -> bool:
    """
    check upload file request
    :param request: HttpRequest
    :return: True if valid else False
    """
    if request.FILES.get("file", None) is None:
        return False
    return True


def s3_download(oss_token: str, filename: str) -> HttpResponse:
    """
    download file from object storage
    :param oss_token: oss_token of file
    :param filename: filename for set response
    :return: response
    """
    response = minio_client.get_object(S3_BUCKET_NAME, oss_token)
    response = HttpResponse(response.read())
    response["Content-Type"] = "application/octet-stream"
    response["Content-Disposition"] = "attachment;filename*=utf-8''{}".format(escape_uri_path(filename))
    return response


def s3_download_url(oss_token: str) -> str:
    """
    get download url for front-end, expire time: 1h
    :param oss_token: oss_token of file
    :return: download_url
    """
    return minio_client.presigned_get_object(S3_BUCKET_NAME, oss_token, expires=timedelta(hours=1))


def s3_upload(oss_token: str, request: HttpRequest) -> HttpResponse:
    """
    upload file to object storage
    :param oss_token: oss_token of file
    :param request: upload file request
    :return: response
    """
    response = minio_client.put_object(S3_BUCKET_NAME, oss_token, request.FILES["file"], request.FILES["file"].size)
    return response


def get_oss_token(query_id: int, filename: str) -> str:
    """
    generate oss token for a file
    :param query_id: query id
    :param filename: filename
    :return: oss_token
    """
    special_symbols = {" ", "(", ")", "/"}
    for symbol in special_symbols:
        filename = filename.replace(symbol, "_")
    return "{}/{}/{}".format(query_id, str(int(datetime.utcnow().timestamp() * 1000)), filename)


def s3_upload_local_file(path: str, oss_token: str):
    """
    upload local file to minio
    :param path: path of this file
    :param oss_token: oss_token
    :return: None
    """
    minio_client.fput_object(S3_BUCKET_NAME, oss_token, path)
