## 运行方式

首先开一个终端，运行 rosbridge

```shell
roslaunch rosbridge_server rosbridge_websocket.launch _port:=9090 websocket_external_port:=80
```

然后再开一个终端，反向代理到云服务器，参考命令如下

```shell
ssh -NR 0.0.0.0:<服务器代理端口>:localhost:9090 <服务器用户名>@<服务器公网ip>
```

然后根据提示输出密码，如无回显则应该没问题，服务器上还是用 nginx 进行了一次反向代理

最后运行 ROS 进程

```shell
roslaunch Tus_g5 final.launch
```

