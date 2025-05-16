# pickandplacescaraarm

This project aims to simulate a SCARA pick and place robot with a conveyor belt in gazebo.

Workflow: 
- SCARA robot, conveyor belt are spawned in the gazebo enviroment, followed by the objects being spawned on the belt.
- The robot then locates the objects based on thier spawn position, the gripper picks the object and places it on the floor.

Progress:
- Simulation of SCARA and conveyor belt successful in gazebo.
- Movement of conveyor belt with certain velocity.

Things to work on:
- Connection to node controller.
- Spawning of objects on the belt.
- Detection of objects on the belt.
- Gripper action. 
