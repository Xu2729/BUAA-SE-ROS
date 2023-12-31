# Generated by Django 4.2 on 2023-04-25 23:00

from django.db import migrations, models
import django.db.models.deletion
import django.utils.timezone


class Migration(migrations.Migration):

    initial = True

    dependencies = []

    operations = [
        migrations.CreateModel(
            name="File",
            fields=[
                (
                    "id",
                    models.BigAutoField(
                        auto_created=True,
                        primary_key=True,
                        serialize=False,
                        verbose_name="ID",
                    ),
                ),
                ("filename", models.CharField(max_length=100)),
                ("oss_token", models.CharField(max_length=300)),
                (
                    "upload_time",
                    models.DateTimeField(default=django.utils.timezone.now),
                ),
            ],
        ),
        migrations.CreateModel(
            name="User",
            fields=[
                (
                    "id",
                    models.BigAutoField(
                        auto_created=True,
                        primary_key=True,
                        serialize=False,
                        verbose_name="ID",
                    ),
                ),
                ("username", models.CharField(max_length=20, unique=True)),
                ("password", models.CharField(max_length=128)),
                ("reg_time", models.DateTimeField(default=django.utils.timezone.now)),
                (
                    "role",
                    models.IntegerField(choices=[(1, "管理员"), (0, "普通用户")], default=0),
                ),
                (
                    "image",
                    models.ForeignKey(
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        to="app0.file",
                    ),
                ),
            ],
        ),
    ]
