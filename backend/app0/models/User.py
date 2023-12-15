from datetime import timedelta

import jwt
from django.db import models
from django.utils import timezone

from rosPrj import settings
from app0.models.File import File

ROLE_NORMAL_USER = 0
ROLE_ADMIN = 1


class User(models.Model):
    """
    用户模型：
    username: 用户名，其实就是账号
    password: 密码，这里存储的是加密后的密文
    reg_time: 注册时间
    role: 角色，分为管理员和普通用户
    image: 头像
    """
    ROLE_TYPE = [
        (ROLE_ADMIN, "管理员"),
        (ROLE_NORMAL_USER, "普通用户"),
    ]

    username = models.CharField(max_length=20, unique=True)
    password = models.CharField(max_length=128)
    reg_time = models.DateTimeField(default=timezone.now)
    role = models.IntegerField(choices=ROLE_TYPE, default=ROLE_NORMAL_USER)
    image = models.ForeignKey(to=File, on_delete=models.SET_NULL, null=True)

    @property
    def token(self):
        token = jwt.encode({
            # timedelta(hours=8) 是为了解决时差问题
            'exp': timezone.now() - timedelta(hours=8) + timedelta(hours=12),
            'iat': timezone.now() - timedelta(hours=8),
            'username': self.username,
            'role': self.role,
        }, settings.SECRET_KEY, algorithm='HS256')
        return token
