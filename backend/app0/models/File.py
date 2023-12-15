from django.db import models
from django.utils import timezone


class File(models.Model):
    """
    文件模型：
    filename: 图片文件名
    oss_token: 用于在 s3 存储桶中定位
    upload_time: 上传时间
    """
    filename = models.CharField(max_length=100)
    oss_token = models.CharField(max_length=300)
    upload_time = models.DateTimeField(default=timezone.now)
