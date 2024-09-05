# Human-Robot Interaction Using Hololens 2



https://github.com/user-attachments/assets/a245e35d-a538-4267-8ce4-e3fc691adfee

The repository containing the code for Human-Robot Interaction utilizes Hololens 2 in conjunction with the paper titled "AR-enhanced digital twin for human–robot interaction in manufacturing systems" by Liao, Zhongyuan, and Yi Cai (2024). The project involves a series of steps for execution:

1. Utilize Docker to run the ROS image and Moba.exe for visualization.
2. Run the project in Unity.
3. Perform manipulation tasks using Hololens 2.



![image](https://github.com/user-attachments/assets/3fbe1c8d-6d87-4f69-b7b3-8e07f155b05a)

Within the Unity environment, various robots are available, including the elephant robot, CR5, and a robot equipped with a gripper.

To customize the project, users can modify the IP address in the RosConnector, set up joint names, and configure the subscriber and publisher components.

Several ROS system commands are essential for the successful operation of the project, including launching specific nodes and establishing communication between ROS and Unity.


![image](https://github.com/user-attachments/assets/1066b760-a012-42b1-a558-e7b09be7a34b)

In the RosConnector, you can change the IP of ROS.
![image](https://github.com/user-attachments/assets/dcbbc92d-dd29-44b0-9c20-2e84dc59be7c)

And you need to set up the joint name:

![image](https://github.com/user-attachments/assets/5d5139c7-506d-446f-8847-b1e43f520bb7)

and subscriber:

![image](https://github.com/user-attachments/assets/c0d0c704-0eaa-4188-8df8-f1ebe34e0921)

and Publisher:

![image](https://github.com/user-attachments/assets/0651041e-45d5-4477-a5e2-2b4651e4999b)


Some commands in ROS system:

docker run -d -it --network host -p 9090:9090 -p 5005:5005 ed9 /bin/bash

export DISPLAY=192.168.1.103:0.0

roslaunch mycobot_280pi test.launch

docker start 4e1

docker exec -it 4e1 /bin/bash

sudo apt-get install ros-noetic-rosbridge-server

roslaunch rosbridge_server rosbridge_websocket.launch port:=9090

docker run -d -it -p 11311:11311 270 /bin/bash

rostopic echo /joint_states


For additional information on the ROS setup for the robot arm, users can refer to the following repositories:

- [CR_ROS by Dobot-Arm](https://github.com/Dobot-Arm/CR_ROS)
- [Elephant Robotics](https://github.com/elephantrobotics)

Moreover, for communication between ROS and Unity, the [rosbridge_suite repository](https://github.com/RobotWebTools/rosbridge_suite) provides valuable resources.

For a mixed reality demonstration, users can explore the [mixed-reality-robot-interaction-demo repository by Microsoft](https://github.com/microsoft/mixed-reality-robot-interaction-demo).

# Cite this article

Liao, Z., Cai, Y. AR-enhanced digital twin for human–robot interaction in manufacturing systems. Energ. Ecol. Environ. (2024). https://doi.org/10.1007/s40974-024-00327-7


