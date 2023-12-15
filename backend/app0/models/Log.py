from django.db import models
from django.utils import timezone

from app0.models.File import File


class Log(models.Model):
    """
    日志模型：
    user: 操作用户
    op_time: 操作时间
    detail: 操作详情，如：用户登录成功，管理员登录成功，新建商品123 等
    """
    time = models.DateTimeField(default=timezone.now)
    detail = models.CharField(max_length=100)
    file = models.ForeignKey(to=File, null=True, on_delete=models.SET_NULL)
