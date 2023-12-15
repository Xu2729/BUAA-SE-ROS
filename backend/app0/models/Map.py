from django.db import models

from app0.models.File import File


class Map(models.Model):
    """
    地图模型：
    file: 对应文件
    name: 地图名
    """
    name = models.CharField(max_length=100)
    x = models.FloatField(default=0.0)
    y = models.FloatField(default=0.0)
    file = models.ForeignKey(to=File, on_delete=models.PROTECT)
