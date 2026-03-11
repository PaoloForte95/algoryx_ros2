import rclpy
from rclpy.node import Node, ReentrantCallbackGroup
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
import agxROS2
import time
from rclpy.duration import Duration

class PandaControllerBridge(Node):
    def __init__(self):
        super().__init__("panda_controller_bridge")

        self.joint_speeds_pub = agxROS2.PublisherStdMsgsFloat32MultiArray("joint_speeds")
        self.gripper_pos_pub = agxROS2.PublisherStdMsgsFloat32("gripper_pos")

        
        cb_group = ReentrantCallbackGroup()
        ActionServer(self, FollowJointTrajectory, "fr3_arm_controller/follow_joint_trajectory", self.execute_arm, callback_group=cb_group)
        ActionServer(self, FollowJointTrajectory, "fr3_hand_controller/follow_joint_trajectory", self.execute_hand, callback_group=cb_group)

    def send_float32_data(self, topic, data):
        msg = agxROS2.StdMsgsFloat32()
        msg.data = data

        if topic == "gripper_pos":
            self.gripper_pos_pub.sendMessage(msg)
        elif topic == "reset_pos":
            self.reset_pub.sendMessage(msg)
        else:
            print("Unknown topic: ", topic)

    def send_float32_multiarray_data(self, topic, data):
        msg = agxROS2.StdMsgsFloat32MultiArray()

        msg.data = data

        if topic == "joint_speeds":
            self.get_logger().info(f"Publishing joint speeds: {data}")
            self.joint_speeds_pub.sendMessage(msg)
        else:
            print("Unknown topic: ", topic)

    
    def execute_arm(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        
        t0 = trajectory.points[0].time_from_start.sec + trajectory.points[0].time_from_start.nanosec * 1e-9
        t1 = trajectory.points[1].time_from_start.sec + trajectory.points[1].time_from_start.nanosec * 1e-9
        dt = t1 - t0

        real_start = time.time()
        sim_start = self.get_clock().now().nanoseconds / 1e9

        for point in trajectory.points:
            velocities = list(point.velocities)
            self.send_float32_multiarray_data("joint_speeds", velocities)

            # Recalculate ratio dynamically every point
            real_elapsed = time.time() - real_start
            sim_elapsed = self.get_clock().now().nanoseconds / 1e9 - sim_start
            sim_to_real = real_elapsed / sim_elapsed if sim_elapsed > 0.001 else 1.0

            time.sleep(dt * sim_to_real)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def execute_hand(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        for point in trajectory.points:
            self.send_float32_data("gripper_pos",point.positions[0])
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(PandaControllerBridge())
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()