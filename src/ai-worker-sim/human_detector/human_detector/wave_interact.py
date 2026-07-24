import copy
import threading

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


ARM_R_JOINT_NAMES = [
    'arm_r_joint1',
    'arm_r_joint2',
    'arm_r_joint3',
    'arm_r_joint4',
    'arm_r_joint5',
    'arm_r_joint6',
    'arm_r_joint7',
]


# Same values as the existing "ready" state in ffw.srdf.
READY_POSITIONS = [
    -2.0005,
    -0.7693,
    1.9266,
    -1.8409,
    2.9385,
    0.5668,
    -1.4242,
]


# Wave positions based on the ready pose.
# Only joint 7 moves.
#
# These values replace the missing SRDF states:
#   wave_left
#   wave_right
#
# Both targets remain close to the valid ready value of -1.4242.
WAVE_LEFT_POSITIONS = [
    -2.0005,
    -0.7693,
    1.9266,
    -1.8409,
    2.9385,
    0.5668,
    -1.10,
]

WAVE_RIGHT_POSITIONS = [
    -2.0005,
    -0.7693,
    1.9266,
    -1.8409,
    2.9385,
    0.5668,
    -1.75,
]


class WaveInteraction(Node):

    STATE_IDLE = 'idle'
    STATE_SENDING = 'sending'
    STATE_NAVIGATING = 'navigating'
    STATE_CANCELLING = 'cancelling'
    STATE_WAVING = 'waving'
    STATE_RESUMING = 'resuming'
    STATE_SUCCEEDED = 'succeeded'
    STATE_CANCELLED = 'cancelled'
    STATE_ABORTED = 'aborted'
    STATE_REJECTED = 'rejected'
    STATE_FAILED = 'failed'

    def __init__(self):
        super().__init__('wave_interaction')

        self.declare_parameter('use_sim_time', True)

        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()

        # ---------------- Nav2 client ----------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group,
        )

        # This node must own the Nav2 goal so it can store, cancel, and resume it.
        # Publish PoseStamped goals to /interaction_goal_pose instead of sending
        # them directly to Nav2 from another node.
        self.create_subscription(
            PoseStamped,
            '/interaction_goal_pose',
            self._goal_pose_cb,
            10,
            callback_group=self.callback_group,
        )

        self.goal_handle = None
        self.current_goal_pose = None

        self.interaction_state = self.STATE_IDLE

        # Each sent goal gets a sequence number. Delayed callbacks belonging
        # to an older cancelled goal are ignored after the goal is resumed.
        self.goal_sequence = 0
        self.active_goal_sequence = None

        # ---------------- Person detection ----------------

        # The latch remains true while the detector continuously publishes True.
        # Navigation can resume immediately after waving; additional True
        # messages are ignored until the detector eventually publishes False.
        self.person_latched = False

        self.create_subscription(
            Bool,
            '/person_detected',
            self.on_person_detected,
            10,
            callback_group=self.callback_group,
        )

        self.wave_thread = None

        # ---------------- MoveIt ----------------

        self.get_logger().info('Creating MoveItPy instance...')

        self.moveit = MoveItPy(
            node_name='wave_interaction_moveit',
        )

        self.robot_model = self.moveit.get_robot_model()

        # Keep the arm and hand as separate planning components.
        self.arm = self.moveit.get_planning_component('arm_r')
        self.hand = self.moveit.get_planning_component('hand_r')

        self.get_logger().info(
            'Wave interaction node ready. '
            'Publish PoseStamped goals on /interaction_goal_pose.'
        )

    # ============================================================
    # State helpers
    # ============================================================

    def _set_state(self, new_state):
        with self.state_lock:
            previous_state = self.interaction_state
            self.interaction_state = new_state

        if previous_state != new_state:
            self.get_logger().info(
                f'Interaction state: {previous_state} -> {new_state}'
            )

    def _goal_pose_cb(self, msg: PoseStamped):
        with self.state_lock:
            busy_states = {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RESUMING,
            }

            if self.interaction_state in busy_states:
                self.get_logger().warning(
                    'Ignoring new navigation goal because the interaction '
                    f'node is currently "{self.interaction_state}"'
                )
                return

        self.send_nav_goal(msg, is_resume=False)

    # ============================================================
    # Nav2 goal handling
    # ============================================================

    def send_nav_goal(self, pose, is_resume=False):
        if pose is None:
            self.get_logger().error(
                'Cannot send Nav2 goal because the pose is None'
            )
            return False

        if not pose.header.frame_id:
            self.get_logger().error(
                'Cannot send Nav2 goal because frame_id is empty'
            )
            return False

        with self.state_lock:
            blocked_states = {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_WAVING,
            }

            if self.interaction_state in blocked_states:
                self.get_logger().warning(
                    'Cannot send Nav2 goal while interaction state is '
                    f'"{self.interaction_state}"'
                )
                return False

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self._set_state(self.STATE_FAILED)
            self.get_logger().error(
                'NavigateToPose action server is unavailable'
            )
            return False

        saved_pose = copy.deepcopy(pose)
        saved_pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = copy.deepcopy(saved_pose)

        with self.state_lock:
            self.current_goal_pose = saved_pose
            self.goal_handle = None

            self.goal_sequence += 1
            goal_sequence = self.goal_sequence
            self.active_goal_sequence = goal_sequence

            self.interaction_state = (
                self.STATE_RESUMING if is_resume else self.STATE_SENDING
            )

        if is_resume:
            self.get_logger().info(
                'Resending the interrupted Nav2 goal'
            )
        else:
            self.get_logger().info('Sending Nav2 goal')

        try:
            future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback:
                    self._nav_feedback_cb(
                        feedback,
                        goal_sequence,
                    ),
            )
            future.add_done_callback(
                lambda completed_future:
                    self._goal_response_cb(
                        completed_future,
                        goal_sequence,
                        is_resume,
                    )
            )
        except Exception as error:
            with self.state_lock:
                if goal_sequence == self.active_goal_sequence:
                    self.goal_handle = None
                    self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                f'Failed to request Nav2 goal: {error}'
            )
            return False

        return True

    def _goal_response_cb(
        self,
        future,
        goal_sequence,
        is_resume,
    ):
        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

        try:
            returned_goal_handle = future.result()
        except Exception as error:
            with self.state_lock:
                if goal_sequence != self.active_goal_sequence:
                    return

                self.goal_handle = None
                self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                f'Failed to send Nav2 goal: {error}'
            )
            return

        if returned_goal_handle is None:
            with self.state_lock:
                self.goal_handle = None
                self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                'Nav2 returned an empty goal handle'
            )
            return

        if not returned_goal_handle.accepted:
            with self.state_lock:
                self.goal_handle = None
                self.interaction_state = self.STATE_REJECTED

            self.get_logger().warning('Nav2 rejected the goal')
            return

        with self.state_lock:
            self.goal_handle = returned_goal_handle
            self.interaction_state = self.STATE_NAVIGATING

        if is_resume:
            self.get_logger().info(
                'Resumed Nav2 goal accepted'
            )
        else:
            self.get_logger().info('Nav2 goal accepted')

        try:
            result_future = returned_goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed_future:
                    self._nav_result_cb(
                        completed_future,
                        goal_sequence,
                    )
            )
        except Exception as error:
            with self.state_lock:
                if goal_sequence == self.active_goal_sequence:
                    self.goal_handle = None
                    self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                f'Failed to request Nav2 result: {error}'
            )

    def _nav_feedback_cb(self, feedback_msg, goal_sequence):
        # Nav2 feedback is frequent. It is deliberately not logged.
        # The sequence check prevents stale feedback from being treated as
        # feedback for a resumed goal.
        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

    def _nav_result_cb(self, future, goal_sequence):
        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

        try:
            wrapped_result = future.result()
        except Exception as error:
            with self.state_lock:
                if goal_sequence != self.active_goal_sequence:
                    return

                self.goal_handle = None
                self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                f'Failed to retrieve Nav2 result: {error}'
            )
            return

        status = wrapped_result.status

        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

            self.goal_handle = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.interaction_state = self.STATE_SUCCEEDED
                message = 'Navigation goal succeeded'
                log_method = self.get_logger().info

            elif status == GoalStatus.STATUS_CANCELED:
                # During an interaction, cancellation is expected. Keep the
                # interaction state if the wave worker is still running.
                if self.interaction_state not in {
                    self.STATE_CANCELLING,
                    self.STATE_WAVING,
                    self.STATE_RESUMING,
                }:
                    self.interaction_state = self.STATE_CANCELLED

                message = 'Navigation goal finished as cancelled'
                log_method = self.get_logger().info

            elif status == GoalStatus.STATUS_ABORTED:
                self.interaction_state = self.STATE_ABORTED
                message = 'Navigation goal was aborted'
                log_method = self.get_logger().error

            else:
                self.interaction_state = self.STATE_FAILED
                message = (
                    'Navigation ended with unexpected action status '
                    f'{status}'
                )
                log_method = self.get_logger().warning

        log_method(message)

    # ============================================================
    # Person detection
    # ============================================================

    def on_person_detected(self, msg: Bool):
        if not msg.data:
            # False rearms the latch, but navigation does not wait for False.
            with self.state_lock:
                self.person_latched = False
            return

        with self.state_lock:
            if self.person_latched:
                self.get_logger().debug(
                    'Ignoring continuous True detection because it is latched'
                )
                return

            if self.interaction_state in {
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RESUMING,
                self.STATE_SENDING,
            }:
                self.get_logger().debug(
                    'Ignoring detection because an interaction is already '
                    f'in progress: "{self.interaction_state}"'
                )
                return

            if self.interaction_state != self.STATE_NAVIGATING:
                self.get_logger().warning(
                    'Person detected, but navigation is not active. '
                    f'Current state: "{self.interaction_state}"'
                )
                return

            if self.goal_handle is None:
                self.interaction_state = self.STATE_FAILED
                self.get_logger().error(
                    'Navigation state is active, but no goal handle exists'
                )
                return

            self.person_latched = True
            self.interaction_state = self.STATE_CANCELLING

            goal_handle = self.goal_handle
            goal_sequence = self.active_goal_sequence

        self.get_logger().info(
            'Person detected — requesting cancellation of Nav2 goal'
        )

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda completed_future:
                    self._after_cancel(
                        completed_future,
                        goal_sequence,
                    )
            )
        except Exception as error:
            with self.state_lock:
                if goal_sequence == self.active_goal_sequence:
                    self.interaction_state = self.STATE_NAVIGATING

            self.get_logger().error(
                f'Failed to request Nav2 cancellation: {error}'
            )

    def _after_cancel(self, future, goal_sequence):
        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

        try:
            cancel_response = future.result()
        except Exception as error:
            with self.state_lock:
                if goal_sequence == self.active_goal_sequence:
                    self.interaction_state = self.STATE_NAVIGATING

            self.get_logger().error(
                f'Nav2 goal cancellation failed: {error}'
            )
            return

        if cancel_response is None:
            with self.state_lock:
                self.interaction_state = self.STATE_NAVIGATING

            self.get_logger().error(
                'Nav2 returned an empty cancellation response'
            )
            return

        if not cancel_response.goals_canceling:
            # The goal may have completed before the cancellation request was
            # processed. Do not wave and do not resend a completed goal.
            with self.state_lock:
                if self.interaction_state == self.STATE_CANCELLING:
                    self.interaction_state = self.STATE_NAVIGATING

            self.get_logger().warning(
                'Nav2 did not confirm cancellation. '
                'Wave will not start and the goal will not be resent.'
            )
            return

        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

            if self.interaction_state != self.STATE_CANCELLING:
                return

            self.interaction_state = self.STATE_WAVING

        self.get_logger().info(
            'Nav2 confirmed cancellation — starting wave worker'
        )

        # MoveIt planning and execution are blocking. Run the complete wave
        # outside the ROS action callback so subscriptions and action callbacks
        # remain responsive.
        self.wave_thread = threading.Thread(
            target=self._wave_worker,
            args=(goal_sequence,),
            daemon=True,
        )
        self.wave_thread.start()

    def _wave_worker(self, goal_sequence):
        wave_success = self.do_wave()

        if not wave_success:
            with self.state_lock:
                if goal_sequence == self.active_goal_sequence:
                    self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                'Wave sequence failed; navigation will not resume'
            )
            return

        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                self.get_logger().warning(
                    'Goal changed during wave; navigation will not resume'
                )
                return

            if self.interaction_state != self.STATE_WAVING:
                self.get_logger().warning(
                    'Interaction state changed during wave; '
                    'navigation will not resume'
                )
                return

            if self.current_goal_pose is None:
                self.interaction_state = self.STATE_FAILED
                self.get_logger().error(
                    'No saved Nav2 goal exists to resume'
                )
                return

            pose_to_resume = copy.deepcopy(self.current_goal_pose)
            self.goal_handle = None
            self.interaction_state = self.STATE_RESUMING

        self.get_logger().info(
            'Wave completed successfully — resuming navigation'
        )

        if not self.send_nav_goal(
            pose_to_resume,
            is_resume=True,
        ):
            with self.state_lock:
                self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                'The original Nav2 goal could not be resumed'
            )

    # ============================================================
    # Wave sequence
    # ============================================================

    def do_wave(self):
        # The named states "open" and "ready" exist in ffw.srdf.
        if not self._go_to_named(
            component=self.hand,
            state_name='open',
            description='open right hand',
        ):
            return False

        if not self._go_to_named(
            component=self.arm,
            state_name='ready',
            description='move right arm to ready',
        ):
            return False

        for wave_number in range(3):
            self.get_logger().info(
                f'Wave cycle {wave_number + 1} of 3'
            )

            if not self._go_to_arm_positions(
                positions=WAVE_LEFT_POSITIONS,
                description='wave left',
            ):
                return False

            if not self._go_to_arm_positions(
                positions=WAVE_RIGHT_POSITIONS,
                description='wave right',
            ):
                return False

        if not self._go_to_named(
            component=self.arm,
            state_name='ready',
            description='return right arm to ready',
        ):
            return False

        return True

    # ============================================================
    # Named-state motion
    # ============================================================

    def _go_to_named(
        self,
        component,
        state_name,
        description,
    ):
        self.get_logger().info(
            f'Planning motion: {description}'
        )

        try:
            # This requires valid data on /joint_states.
            component.set_start_state_to_current_state()

            component.set_goal_state(
                configuration_name=state_name,
            )

        except Exception as error:
            self.get_logger().error(
                f'Failed to configure motion "{description}": {error}'
            )
            return False

        return self._plan_and_execute(
            component=component,
            description=description,
        )

    # ============================================================
    # Direct joint-value motion
    # ============================================================

    def _go_to_arm_positions(
        self,
        positions,
        description,
    ):
        if len(positions) != len(ARM_R_JOINT_NAMES):
            self.get_logger().error(
                f'Invalid arm target for "{description}": '
                f'expected {len(ARM_R_JOINT_NAMES)} values, '
                f'got {len(positions)}'
            )
            return False

        self.get_logger().info(
            f'Planning motion: {description}'
        )

        try:
            # This requires /joint_states to be available.
            self.arm.set_start_state_to_current_state()

            goal_state = RobotState(self.robot_model)

            # Assign all seven joints in the exact arm_r group order.
            goal_state.set_joint_group_positions(
                'arm_r',
                positions,
            )

            # Update transforms after modifying joint values.
            goal_state.update()

            self.arm.set_goal_state(
                robot_state=goal_state,
            )

        except Exception as error:
            self.get_logger().error(
                f'Failed to configure motion "{description}": {error}'
            )
            return False

        return self._plan_and_execute(
            component=self.arm,
            description=description,
        )

    # ============================================================
    # Plan and execute
    # ============================================================

    def _plan_and_execute(
        self,
        component,
        description,
    ):
        try:
            plan_result = component.plan()
        except Exception as error:
            self.get_logger().error(
                f'Planning raised an exception for '
                f'"{description}": {error}'
            )
            return False

        if not plan_result:
            self.get_logger().error(
                f'Planning failed for "{description}"'
            )
            return False

        if not hasattr(plan_result, 'trajectory'):
            self.get_logger().error(
                f'Planning result for "{description}" '
                'does not contain a trajectory'
            )
            return False

        self.get_logger().info(
            f'Executing motion: {description}'
        )

        try:
            # Execution must be performed through the main MoveItPy
            # instance, not through the PlanningComponent.
            execution_result = self.moveit.execute(
                plan_result.trajectory,
                controllers=[],
            )
        except Exception as error:
            self.get_logger().error(
                f'Execution raised an exception for '
                f'"{description}": {error}'
            )
            return False

        if not self._execution_succeeded(execution_result):
            self.get_logger().error(
                f'Execution failed or was aborted for "{description}". '
                f'Result: {execution_result}'
            )
            return False

        self.get_logger().info(
            f'Motion completed successfully: {description}'
        )

        return True

    @staticmethod
    def _execution_succeeded(execution_result):
        """
        Handle the result formats used by different MoveItPy versions.

        Possible formats include:
        - bool
        - MoveItErrorCode-like object with a .val field
        - object exposing success
        """

        if execution_result is None:
            return False

        if isinstance(execution_result, bool):
            return execution_result

        if hasattr(execution_result, 'val'):
            # MoveItErrorCodes.SUCCESS is 1.
            return execution_result.val == 1

        if hasattr(execution_result, 'success'):
            return bool(execution_result.success)

        # Do not silently treat an unknown result type as success.
        return False


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = MultiThreadedExecutor()

    try:
        node = WaveInteraction()
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().fatal(
                f'Wave interaction node crashed: {error}'
            )
        else:
            print(f'Wave interaction node failed to start: {error}')

    finally:
        executor.shutdown()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()