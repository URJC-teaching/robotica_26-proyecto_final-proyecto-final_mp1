# Copyright 2025 - Final Project ROS2 Jazzy
# Nodo de prueba independiente para Nav2
# Estilo: simple_navigation_app de los ejemplos base

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class NavTestNode(Node):
    """
    Nodo de prueba standalone para Nav2.
    Envía una secuencia de goals predeterminados y espera cada resultado.
    Útil para verificar la navegación antes de integrar HRI y YOLO.
    """

    def __init__(self):
        super().__init__('nav_test_node')

        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_theta', 0.0)

        goal_x     = self.get_parameter('goal_x').value
        goal_y     = self.get_parameter('goal_y').value
        goal_theta = self.get_parameter('goal_theta').value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.target_pose = self._make_pose(goal_x, goal_y, goal_theta)
        self.server_ready = False
        self.goal_sent    = False

        self.timer = self.create_timer(0.5, self._control_cycle)
        self.get_logger().info(
            f'NavTest: goal = ({goal_x}, {goal_y}, theta={goal_theta} rad)'
        )

    def _make_pose(self, x: float, y: float, theta: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def _control_cycle(self):
        if not self.server_ready:
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Servidor Nav2 disponible')
                self.server_ready = True
            return

        if not self.goal_sent:
            self.get_logger().info('Enviando goal de navegación...')
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal = NavigateToPose.Goal()
            goal.pose = self.target_pose
            future = self.nav_client.send_goal_async(
                goal, feedback_callback=self._feedback_cb
            )
            future.add_done_callback(self._goal_response_cb)
            self.goal_sent = True

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        t = fb.navigation_time.sec + fb.navigation_time.nanosec / 1e9
        self.get_logger().info(
            f'  Distancia restante: {fb.distance_remaining:.2f} m | '
            f'Tiempo: {t:.1f} s'
        )

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rechazado por Nav2')
            self.timer.cancel()
            return
        self.get_logger().info('Goal aceptado, navegando...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        from action_msgs.msg import GoalStatus
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navegacion completada con exito')
        else:
            self.get_logger().warn(f'Navegacion fallida (status={status})')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = NavTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
