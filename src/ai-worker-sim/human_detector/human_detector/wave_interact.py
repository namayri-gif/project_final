import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

from moveit.planning import MoveItPy


class WaveInteraction(Node):
    def __init__(self):
        super().__init__('wave_interaction')

        cb_group = ReentrantCallbackGroup()
        # ---
        # --- Nav2 client ---
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=cb_group
        )
        self.goal_handle = None
        self.current_goal_pose = None
        self.waving = False

        # --- Detection subscriber ---
        self.create_subscription(
            Bool, 'person_detected', self.on_person_detected, 10,
            callback_group=cb_group
        )

        # --- MoveIt ---
        self.moveit = MoveItPy(node_name='wave_interaction_moveit')
        self.arm = self.moveit.get_planning_component('arm_r')
        self.hand = self.moveit.get_planning_component('hand_r')

        self.get_logger().info('wave_interaction node ready')

    # ---------------- Nav2 goal handling ----------------

    def send_nav_goal(self, pose):
        self.current_goal_pose = pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')

    # ---------------- Detection callback ----------------

    def on_person_detected(self, msg: Bool):
        if msg.data and not self.waving and self.goal_handle is not None:
            self.waving = True
            self.get_logger().info('Person detected — cancelling current goal')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._after_cancel)

    def _after_cancel(self, future):
        self.get_logger().info('Goal cancelled — starting wave')
        self.do_wave()
        self.get_logger().info('Wave done — resuming original goal')
        self.waving = False
        self.send_nav_goal(self.current_goal_pose)

    # ---------------- Wave motion ----------------

    def do_wave(self):
        self._go_to_named(self.hand, 'open')
        self._go_to_named(self.arm, 'ready')

        for _ in range(3):
            self._go_to_named(self.arm, 'wave_left')
            self._go_to_named(self.arm, 'wave_right')

        self._go_to_named(self.arm, 'ready')

    def _go_to_named(self, component, state_name):
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=state_name)
        plan_result = component.plan()
        if plan_result:
            component.execute(plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error(f'Planning failed for -> {state_name}')


def main(args=None):
    rclpy.init(args=args)
    node = WaveInteraction()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()