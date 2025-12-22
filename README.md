# 兼容传统电控代码的自瞄部署仓库

## 依赖
```
 操作系统: Ubuntu22.04 
 ROS: Rolling (以及camera-info-manager、generate-parameters-library、serial-driver)
 OpenCV: 4.5.4 or later
 CMake: 3.22.1 or later
 Ninja: 1.11.1 or later
 Clang: 15 or later
 Ceres
 Eigen
```
## 快速部署

### 手动部署(需要自己手动在主机上配置ssh-key)
```bash
    # cd 到指定目录下然后执行以下命令拉取代码
    mkdir autoaim_ws && cd autoaim_ws && \
    git clone git@e.coding.net:swjtuhelios/cv/autoaim_top_module.git --recursive && \
    mv autoaim_top_module src && cd src && \
    git submodule update --init --recursive
    # 然后回到工作空间进行编译（可以修改src目录下的build.sh文件来自定义编译参数）
    cd ../ && sh ./src/build.sh release 
```
### 配置相关环境以及脚本自启动

#### 配置识别器和预测器的日志文本输出

```bash
cd ~ && mkdir logs && \
cd logs && mkdir detector_logs armor_predictor_logs energy_predictor_logs
```

#### 配置脚本部署方式

详情查看仓库： [**auto_startup**](https://swjtuhelios.coding.net/p/cv/d/auto_startup/git)

### docker部署
#### 容器构建

注意：如果需要自己构建容器，执行以下命令：
    
    cd ~/autoaim_ws/src && \
    docker build --build-arg ROS_DOAMIN_ID=<ID> -t autoaim_deploy .

#### 直接部署命令

    # 构建开发容器(调试的时候使用)
    docker run -it --name autoaim_devel \
        --privileged --network host \
        -v /dev:/dev -v $HOME/.ros:/root/.ros \
        autoaim_deploy:latest \
    # 构建运行容器(部署上场的时候用)
    docker run -it --name autoaim_runtime \
        --privileged --network host --restart always \
        -v /dev:/dev -v $HOME/.ros:/root/.ros -v /home/helios/Videos:/home/helios/Videos \
        autoaim_deploy:latest \
        ros2 launch autoaim_bring_up autoaim.launch.py

## 快速拉取远程更改
    cd autoaim_ws && \
    sudo chmod +x ./auto_pull.sh && \
    ./auto_pull.sh

## 快速推送本地更改
    cd autoaim_ws && \
    sudo chmod +x ./auto_push.sh && \
    ./auto_push.sh