from django.db import models

from app0.models.File import File


class Face(models.Model):
    """
    人脸模型：
    name: 名字
    encoding: 二进制形式的编码后的人脸数据
    strange: 是否为陌生人
    file: 对应的文件
    """
    name = models.CharField(max_length=20)
    encodings = models.BinaryField()
    strange = models.BooleanField(default=False)
    file = models.ForeignKey(to=File, on_delete=models.PROTECT)
