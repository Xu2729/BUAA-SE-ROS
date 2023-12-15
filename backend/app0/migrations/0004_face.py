# Generated by Django 4.2 on 2023-05-21 20:50

from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ("app0", "0003_point"),
    ]

    operations = [
        migrations.CreateModel(
            name="Face",
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
                ("name", models.CharField(max_length=20, unique=True)),
                ("encodings", models.BinaryField()),
                ("strange", models.BooleanField(default=False)),
                (
                    "file",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.PROTECT, to="app0.file"
                    ),
                ),
            ],
        ),
    ]
