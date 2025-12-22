docker build --build-arg ROS_DOAMIN_ID=<ID> -t autoaim_deploy:latest .

docker run -it --name autoaim_runtime \
    --privileged --network host --restart always \
    -v /dev:/dev -v $HOME/.ros:/root/.ros \
    swjtuhelios-docker.pkg.coding.net/cv/rm_dev_docker/autoaim_deploy:latest \
    ros2 launch autoaim_bring_up autoaim.launch.py
