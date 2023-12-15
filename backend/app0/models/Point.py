from django.db import models

from app0.models.Map import Map


class Point(models.Model):
    """
    航点模型：
    mmap: 对应地图
    name: 航点名称
    px, py, pz: 位置信息
    ox, oy, oz, ow: 方向信息
    """
    mmap = models.ForeignKey(to=Map, on_delete=models.CASCADE)
    name = models.CharField(max_length=20)
    px = models.FloatField()
    py = models.FloatField()
    pz = models.FloatField()
    ox = models.FloatField()
    oy = models.FloatField()
    oz = models.FloatField()
    ow = models.FloatField()

    @property
    def position(self) -> dict:
        return {
            "x": self.px,
            "y": self.py,
            "z": self.pz,
        }

    @property
    def orientation(self) -> dict:
        return {
            "x": self.ox,
            "y": self.oy,
            "z": self.oz,
            "w": self.ow,
        }

    @property
    def pose(self):
        return {
            "position": self.position,
            "orientation": self.orientation,
        }
