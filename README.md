#### ROS wrapper for dense_flow.

Extracting dense flow field given a video and publish it as a topic.

work in progress..

#### TODO:

- make same modifications to warp flow, cpu flow
- remove/ fix saving of images (not functional with the many videos stream)
- fix the change video image transition error
- fix the normalization function (the CAST compiler def was not working and you changed for something else that does not make the same kinds of images - they used to look much grayer so the trained network might not work)
- fix bug during execution when we don't have output for quite a while (I believe it is a jit compiler issue, but I am not sure). At least have it should error until the publisher actually starts doing its thing

#### running

you need to setup ros networking well with ros and docker. easier said than done; make sure hosts are visible to each other and present with their correct hostname on their respective /etc/hosts file (on the docker pc and roscore pc). I am not using docker compose, so you will need to set the right addresses in the entrypoint scripts of each docker container as well.

Also, the roscore (in my case hostname:SATELLITE-S50-B; ip4:10.0.0.239) needs to forward packages to the docker network (which runs somewhere). In my case the docker pc has ip 10.0.0.7 and is connected to my physical network via interface enp8s0, so this network rule becomes:

    (SATELLITE-S50-B)$ sudo ip route add 172.0.0.0/8 via 10.0.0.7 dev enp8s0
    
By default docker does not forward packages outside of the pc in which it is running (probably security vulnerability?). In my case hostname:poop; ip4:10.0.0.7, we need this to be done, or ros will not be able to read topic outputs from the docker nodes. This can be set up with the rules (on docker pc):

    (poop)$ sudo sysctl net.ipv4.conf.all.forwarding=1 && sudo iptables -P FORWARD ACCEPT
    
After this initial setup is all done, you need to run this package from the respective docker container (see https://github.com/mysablehats/dt.git). It should have been compiled by catkin_make already, so nothing to do here. 

Source the devel/setup.bash file from the catkin_ws directory and run:

    roslaunch dense_flow df.launch
    
Now, there is a slight bug here still of this date in which the correct cuda code was not generated. I have no idea how to fix this yet, so until the jit compiler fixes that we won't have any output. You can fork the roslaunch to the background and use rostopic hz /extract_gpu/image to check when it will start to publish. 
