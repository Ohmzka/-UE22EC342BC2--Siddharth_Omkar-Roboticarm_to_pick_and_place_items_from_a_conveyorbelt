#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('spawn_objects')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_entity service...')
        self.timer = self.create_timer(15.0, self.spawn_cylinder)
        self.count = 0

    def spawn_cylinder(self):
        try:
            request = SpawnEntity.Request()
            request.name = f"cylinder_{self.count}"
            request.xml = """
            <sdf version='1.6'>
              <model name='cylinder'>
                <pose>0 0 0.5 0 0 0</pose>
                <link name='link'>
                  <visual name='visual'>
                    <geometry>
                      <cylinder>
                        <radius>0.05</radius>
                        <length>0.1</length>
                      </cylinder>
                    </geometry>
                  </visual>
                  <collision name='collision'>
                    <geometry>
                      <cylinder>
                        <radius>0.05</radius>
                        <length>0.1</length>
                      </cylinder>
                    </geometry>
                  </collision>
                </link>
              </model>
            </sdf>
            """
            pose = Pose()
            pose.position.x = 0.7     # Adjust to conveyor's center
            pose.position.y = 0.0
            pose.position.z = 0.4     # Ensure it's just above belt
            request.initial_pose = pose
            request.robot_namespace = ""

            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"Spawned cylinder_{self.count}")
            else:
                self.get_logger().error(f"Failed to spawn cylinder_{self.count}")
            self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in spawn_cylinder: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

