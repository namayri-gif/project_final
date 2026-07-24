import copy
import math
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


def radians(values_degrees):
    return [math.radians(value) for value in values_degrees]


# The robot rejected the earlier +122-degree shoulder target because it moved
# the upper arm into the lift/body. These candidates keep shoulder pitch near
# zero and use negative shoulder roll so the arm moves outward from the side.
#
# MoveIt tests each candidate for joint limits and self-collision. The first
# candidate that produces a valid plan is executed.
SIDE_ARM_CANDIDATES = [
    radians([0.0, -60.0, 0.0, -20.0, 0.0, 0.0, 0.0]),
    radians([0.0, -70.0, 0.0, -25.0, 0.0, 0.0, 0.0]),
    radians([-10.0, -60.0, 0.0, -25.0, 0.0, 0.0, 0.0]),
    radians([10.0, -60.0, 0.0, -25.0, 0.0, 0.0, 0.0]),
]


# After the arm is raised laterally, rotate the elbow plane upward and bend
# the elbow. Again, MoveIt selects the first collision-free candidate.
ELBOW_UP_CANDIDATES = [
    radians([0.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0]),
    radians([0.0, -65.0, -75.0, -85.0, 0.0, 0.0, 0.0]),
    radians([0.0, -55.0, -90.0, -80.0, 0.0, 0.0, 0.0]),
    radians([-10.0, -60.0, -80.0, -85.0, 0.0, 0.0, 0.0]),
]


# Wrist wave amplitude. Joint 6 is changed while the shoulder and elbow remain
# fixed. Two complete left/right cycles are executed.
WRIST_WAVE_DEGREES = 20.0


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

        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()

        self.interaction_state = self.STATE_IDLE
        self.goal_handle = None
        self.current_goal_pose = None
        self.goal_sequence = 0
        self.active_goal_sequence = None
        self.wave_thread = None

        # Only one detector-triggered wave is allowed for each external goal.
        # A temporary detector dropout cannot start a second wave after resume.
        self.interaction_done_for_goal = False

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group,
        )

        # Send PoseStamped goals here instead of directly through RViz/Nav2.
        # This node must own the goal handle to cancel and resume the goal.
        self.create_subscription(
            PoseStamped,
            '/interaction_goal_pose',
            self._goal_pose_cb,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Bool,
            '/person_detected',
            self.on_person_detected,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Bool,
            '/wave_command',
            self._wave_command_cb,
            10,
            callback_group=self.callback_group,
        )

        self.get_logger().info('Creating MoveItPy instance...')

        # The launch file supplies the complete MoveIt configuration and
        # use_sim_time. Do not declare use_sim_time here and do not pass a
        # config_dict to MoveItPy.
        self.moveit = MoveItPy(
            node_name='wave_interaction_moveit',
        )

        self.robot_model = self.moveit.get_robot_model()
        self.arm = self.moveit.get_planning_component('arm_r')

        self.get_logger().info(
            'Wave interaction node ready. Goals: /interaction_goal_pose. '
            'Standalone wave: /wave_command.'
        )

    # ============================================================
    # State helpers
    # ============================================================

    def _set_state(self, new_state):
        with self.state_lock:
            old_state = self.interaction_state
            self.interaction_state = new_state

        if old_state != new_state:
            self.get_logger().info(
                f'Interaction state: {old_state} -> {new_state}'
            )

    def _wave_is_running(self):
        return self.wave_thread is not None and self.wave_thread.is_alive()

    # ============================================================
    # Standalone wave test
    # ============================================================

    def _wave_command_cb(self, msg: Bool):
        if not msg.data:
            return

        with self.state_lock:
            if self._wave_is_running():
                self.get_logger().warning(
                    'Ignoring wave command because a wave is already running'
                )
                return

            if self.interaction_state in {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RESUMING,
            }:
                self.get_logger().warning(
                    'Ignoring standalone wave command while state is '
                    f'"{self.interaction_state}"'
                )
                return

            self.interaction_state = self.STATE_WAVING

        self.get_logger().info('Starting standalone wave')

        self.wave_thread = threading.Thread(
            target=self._standalone_wave_worker,
            daemon=True,
        )
        self.wave_thread.start()

    def _standalone_wave_worker(self):
        success = self.do_wave()

        with self.state_lock:
            self.interaction_state = (
                self.STATE_IDLE if success else self.STATE_FAILED
            )

        if success:
            self.get_logger().info('Standalone wave completed successfully')
        else:
            self.get_logger().error('Standalone wave failed')

    # ============================================================
    # Nav2 goal ownership
    # ============================================================

    def _goal_pose_cb(self, msg: PoseStamped):
        with self.state_lock:
            if self.interaction_state in {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RESUMING,
            }:
                self.get_logger().warning(
                    'Ignoring new goal while state is '
                    f'"{self.interaction_state}"'
                )
                return

        self.send_nav_goal(msg, is_resume=False)

    def send_nav_goal(self, pose: PoseStamped, is_resume=False):
        if pose is None:
            self.get_logger().error('Cannot send a None Nav2 goal')
            return False

        if not pose.header.frame_id:
            self.get_logger().error(
                'Cannot send Nav2 goal because frame_id is empty'
            )
            return False

        with self.state_lock:
            if self.interaction_state in {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_WAVING,
            }:
                self.get_logger().warning(
                    'Cannot send Nav2 goal while state is '
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
            if not is_resume:
                self.interaction_done_for_goal = False

            self.current_goal_pose = saved_pose
            self.goal_handle = None
            self.goal_sequence += 1
            goal_sequence = self.goal_sequence
            self.active_goal_sequence = goal_sequence
            self.interaction_state = (
                self.STATE_RESUMING if is_resume else self.STATE_SENDING
            )

        self.get_logger().info(
            'Resending interrupted Nav2 goal'
            if is_resume
            else 'Sending Nav2 goal'
        )

        try:
            future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self._nav_feedback_cb(
                    feedback,
                    goal_sequence,
                ),
            )

            future.add_done_callback(
                lambda completed_future: self._goal_response_cb(
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

            self.get_logger().error('Nav2 returned an empty goal handle')
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

        self.get_logger().info(
            'Resumed Nav2 goal accepted'
            if is_resume
            else 'Nav2 goal accepted'
        )

        try:
            result_future = returned_goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed_future: self._nav_result_cb(
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

    def _nav_feedback_cb(self, _feedback_msg, goal_sequence):
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
        start_wave = False

        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

            self.goal_handle = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.interaction_state = self.STATE_SUCCEEDED
                message = 'Navigation goal succeeded'
                log_method = self.get_logger().info

            elif status == GoalStatus.STATUS_CANCELED:
                if self.interaction_state == self.STATE_CANCELLING:
                    self.interaction_state = self.STATE_WAVING
                    start_wave = True
                    message = (
                        'Navigation goal is fully canceled; starting wave'
                    )
                    log_method = self.get_logger().info
                elif self.interaction_state not in {
                    self.STATE_WAVING,
                    self.STATE_RESUMING,
                }:
                    self.interaction_state = self.STATE_CANCELLED
                    message = 'Navigation goal finished as canceled'
                    log_method = self.get_logger().info
                else:
                    message = 'Canceled result received during interaction'
                    log_method = self.get_logger().debug

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

        if start_wave:
            self._start_navigation_wave_worker(goal_sequence)

    # ============================================================
    # Detection and cancellation
    # ============================================================

    def on_person_detected(self, msg: Bool):
        if not msg.data:
            return

        with self.state_lock:
            if self.interaction_done_for_goal:
                return

            if self.interaction_state != self.STATE_NAVIGATING:
                return

            if self.goal_handle is None:
                self.interaction_state = self.STATE_FAILED
                self.get_logger().error(
                    'Navigation is active, but no goal handle exists'
                )
                return

            # Set this before cancellation. Any temporary detector dropout and
            # re-detection is ignored for the remainder of this goal.
            self.interaction_done_for_goal = True
            self.interaction_state = self.STATE_CANCELLING

            goal_handle = self.goal_handle
            goal_sequence = self.active_goal_sequence

        self.get_logger().info(
            'Person detected; requesting Nav2 goal cancellation'
        )

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda completed_future: self._after_cancel_request(
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

    def _after_cancel_request(self, future, goal_sequence):
        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

        try:
            cancel_response = future.result()
        except Exception as error:
            with self.state_lock:
                if (
                    goal_sequence == self.active_goal_sequence
                    and self.interaction_state == self.STATE_CANCELLING
                ):
                    self.interaction_state = self.STATE_NAVIGATING

            self.get_logger().error(
                f'Nav2 cancellation request failed: {error}'
            )
            return

        if cancel_response is None or not cancel_response.goals_canceling:
            with self.state_lock:
                if (
                    goal_sequence == self.active_goal_sequence
                    and self.interaction_state == self.STATE_CANCELLING
                ):
                    self.interaction_state = self.STATE_NAVIGATING

            self.get_logger().warning(
                'Nav2 did not accept the cancellation request'
            )
            return

        self.get_logger().info(
            'Nav2 accepted the cancellation request; waiting for the '
            'terminal canceled result'
        )

    def _start_navigation_wave_worker(self, goal_sequence):
        with self.state_lock:
            if goal_sequence != self.active_goal_sequence:
                return

            if self._wave_is_running():
                self.get_logger().warning(
                    'Wave worker is already running'
                )
                return

        self.wave_thread = threading.Thread(
            target=self._navigation_wave_worker,
            args=(goal_sequence,),
            daemon=True,
        )
        self.wave_thread.start()

    def _navigation_wave_worker(self, goal_sequence):
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
            'Wave completed successfully; resuming navigation'
        )

        if not self.send_nav_goal(pose_to_resume, is_resume=True):
            with self.state_lock:
                self.interaction_state = self.STATE_FAILED

            self.get_logger().error(
                'The interrupted Nav2 goal could not be resumed'
            )

    # ============================================================
    # Side raise, elbow up, wrist wave
    # ============================================================

    def do_wave(self):
        selected_side_pose = self._plan_and_execute_first_valid(
            candidates=SIDE_ARM_CANDIDATES,
            description='raise right arm outward from the side',
        )

        if selected_side_pose is None:
            return False

        selected_elbow_pose = self._plan_and_execute_first_valid(
            candidates=ELBOW_UP_CANDIDATES,
            description='bend right elbow upward',
        )

        if selected_elbow_pose is None:
            return False

        wrist_left = list(selected_elbow_pose)
        wrist_right = list(selected_elbow_pose)

        wrist_left[5] = math.radians(-WRIST_WAVE_DEGREES)
        wrist_right[5] = math.radians(WRIST_WAVE_DEGREES)

        for cycle in range(2):
            self.get_logger().info(
                f'Wrist wave cycle {cycle + 1} of 2'
            )

            if not self._go_to_arm_positions(
                wrist_left,
                'move right hand left',
            ):
                return False

            if not self._go_to_arm_positions(
                wrist_right,
                'move right hand right',
            ):
                return False

        return True

    def _plan_and_execute_first_valid(self, candidates, description):
        for index, positions in enumerate(candidates, start=1):
            self.get_logger().info(
                f'Trying candidate {index} of {len(candidates)} for '
                f'"{description}"'
            )

            plan_result = self._plan_arm_positions(
                positions=positions,
                description=f'{description}, candidate {index}',
                log_failure=False,
            )

            if plan_result is None:
                self.get_logger().warning(
                    f'Candidate {index} is invalid or collision-prone'
                )
                continue

            if not self._execute_plan(
                plan_result=plan_result,
                description=f'{description}, candidate {index}',
            ):
                return None

            self.get_logger().info(
                f'Selected candidate {index} for "{description}"'
            )
            return list(positions)

        self.get_logger().error(
            f'No valid collision-free candidate was found for "{description}"'
        )
        return None

    def _go_to_arm_positions(self, positions, description):
        plan_result = self._plan_arm_positions(
            positions=positions,
            description=description,
            log_failure=True,
        )

        if plan_result is None:
            return False

        return self._execute_plan(
            plan_result=plan_result,
            description=description,
        )

    def _plan_arm_positions(
        self,
        positions,
        description,
        log_failure,
    ):
        if len(positions) != len(ARM_R_JOINT_NAMES):
            self.get_logger().error(
                f'Invalid target for "{description}": expected '
                f'{len(ARM_R_JOINT_NAMES)} values, got {len(positions)}'
            )
            return None

        self.get_logger().info(f'Planning motion: {description}')

        try:
            self.arm.set_start_state_to_current_state()

            goal_state = RobotState(self.robot_model)
            goal_state.set_joint_group_positions(
                'arm_r',
                positions,
            )
            goal_state.update()

            self.arm.set_goal_state(
                robot_state=goal_state,
            )

            plan_result = self.arm.plan()

        except Exception as error:
            if log_failure:
                self.get_logger().error(
                    f'Planning raised an exception for "{description}": '
                    f'{error}'
                )
            return None

        if not plan_result or not hasattr(plan_result, 'trajectory'):
            if log_failure:
                self.get_logger().error(
                    f'Planning failed for "{description}"'
                )
            return None

        return plan_result

    def _execute_plan(self, plan_result, description):
        self.get_logger().info(f'Executing motion: {description}')

        try:
            execution_result = self.moveit.execute(
                plan_result.trajectory,
                controllers=[],
            )
        except Exception as error:
            self.get_logger().error(
                f'Execution raised an exception for "{description}": {error}'
            )
            return False

        if not self._execution_succeeded(execution_result):
            self.get_logger().error(
                f'Execution failed for "{description}". '
                f'Result: {execution_result}'
            )
            return False

        self.get_logger().info(
            f'Motion completed successfully: {description}'
        )
        return True

    @staticmethod
    def _execution_succeeded(execution_result):
        if execution_result is None:
            return False

        status = getattr(execution_result, 'status', None)
        if status is not None:
            return str(status).strip().upper() == 'SUCCEEDED'

        result_text = str(execution_result).strip().upper()
        if 'SUCCEEDED' in result_text:
            return True

        if isinstance(execution_result, bool):
            return execution_result

        if hasattr(execution_result, 'val'):
            return execution_result.val == 1

        if hasattr(execution_result, 'success'):
            return bool(execution_result.success)

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
