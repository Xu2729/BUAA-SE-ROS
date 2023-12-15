# BUAA-SE-ROS-backend

## 运行方式

先根据 config_example.yaml 编写配置文件 config.yaml

### Windows

建立虚拟环境

```shell
virtualenv venv
```

激活虚拟环境

```shell
cd venv/Scripts
activate.bat
cd ../../
```

安装依赖

```shell
pip install -r requirements.txt
```

运行服务

```shell
python manage.py runserver
```

### Linux

建立虚拟环境

```shell
virtualenv venv
```

激活虚拟环境

```shell
source venv/bin/activate
```

安装依赖

```shell
pip3 install -r requirements.txt
```

运行服务

```shell
python3 manage.py runserver
```
