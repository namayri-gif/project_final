import copy
import os
import threading

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
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


# Wave targets based on the existing arm_r "ready" SRDF state.
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

<<<<<<< HEAD


=======
>>>>>>> 63cbe71 (Merge remote changes and update project)
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()

        self.interaction_state = self.STATE_IDLE
        self.goal_handle = None
        self.current_goal_pose = None
        self.goal_sequence = 0
        self.active_goal_sequence = None
        self.person_latched = False
        self.wave_thread = None

        # ---------------- Nav2 client ----------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group,
        )

        # Goals must enter through this topic so this node owns the Nav2
        # goal handle and can safely cancel and resume it.
        self.create_subscription(
            PoseStamped,
            '/interaction_goal_pose',
            self._goal_pose_cb,
            10,
            callback_group=self.callback_group,
        )

        # ---------------- Detection and standalone test ----------------

        self.create_subscription(
            Bool,
            '/person_detected',
            self.on_person_detected,
            10,
            callback_group=self.callback_group,
        )

        # Publish True here to test the complete wave without Nav2.
        self.create_subscription(
            Bool,
            '/wave_command',
            self._wave_command_cb,
            10,
            callback_group=self.callback_group,
        )

        # ---------------- MoveItPy ----------------

        self.get_logger().info('Building MoveItPy configuration...')
        moveit_config = self._build_moveit_config()

        self.get_logger().info('Creating MoveItPy instance...')
        self.moveit = MoveItPy(
            node_name='wave_interaction_moveit',
            config_dict=moveit_config,
        )

        self.robot_model = self.moveit.get_robot_model()
        self.arm = self.moveit.get_planning_component('arm_r')
        self.hand = self.moveit.get_planning_component('hand_r')

        self.get_logger().info(
            'Wave interaction node ready. Goals: /interaction_goal_pose. '
            'Standalone wave: /wave_command.'
        )

    def _build_moveit_config(self):
        ffw_description_share = get_package_share_directory(
            'ffw_description'
        )

        ffw_moveit_config_share = get_package_share_directory(
            'ffw_moveit_config'
        )

        robot_xacro_path = os.path.join(
            ffw_description_share,
            'urdf',
            'ffw_sh5_rev1_follower',
            'ffw_sh5_follower.urdf.xacro',
        )

        moveit_cpp_path = os.path.join(
            ffw_moveit_config_share,
            'config',
            'moveit_cpp.yaml',
        )

        required_files = [
            robot_xacro_path,
            moveit_cpp_path,
        ]

        for file_path in required_files:
            if not os.path.exists(file_path):
                raise FileNotFoundError(
                    f'Required MoveIt file was not found: {file_path}'
                )

        return (
            MoveItConfigsBuilder(
                robot_name='ffw',
                package_name='ffw_moveit_config',
            )
            .robot_description(
                file_path=robot_xacro_path,
            )
            .robot_description_semantic(
                file_path='config/ffw.srdf',
            )
            .robot_description_kinematics(
                file_path='config/kinematics.yaml',
            )
            .joint_limits(
                file_path='config/joint_limits.yaml',
            )
            .planning_pipelines(
                pipelines=['ompl'],
                default_planning_pipeline='ompl',
            )
            .trajectory_execution(
                file_path='config/moveit_controllers.yaml',
            )
            .moveit_cpp(
                file_path=moveit_cpp_path,
            )
            .to_moveit_configs()
            .to_dict()
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
                    'Ignoring standalone wave command: wave already running'
                )
                return

            if self.interaction_state in {
                self.STATE_SENDING,
                self.STATE_NAVIGATING,
                self.STATE_CANCELLING,
                self.STATE_RESUMING,
                self.STATE_WAVING,
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
        wave_success = self.do_wave()

        with self.state_lock:
            self.interaction_state = (
                self.STATE_IDLE if wave_success else self.STATE_FAILED
            )

        if wave_success:
            self.get_logger().info('Standalone wave completed successfully')
        else:
            self.get_logger().error('Standalone wave failed')

    # ============================================================
    # Nav2 goal handling
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
                    'Ignoring new navigation goal because state is '
                    f'"{self.interaction_state}"'
                )
                return

        self.send_nav_goal(msg, is_resume=False)

    def send_nav_goal(self, pose, is_resume=False):
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
                    # The goal is now terminally canceled. It is safe to move
                    # the arm and later resend the saved navigation goal.
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
    # Person detection and cancellation
    # ============================================================

    def on_person_detected(self, msg: Bool):
        if not msg.data:
            with self.state_lock:
                self.person_latched = False
            return

        with self.state_lock:
            if self.person_latched:
                return

            if self.interaction_state in {
                self.STATE_CANCELLING,
                self.STATE_WAVING,
                self.STATE_RESUMING,
                self.STATE_SENDING,
            }:
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
                    'Navigation is active, but no goal handle exists'
                )
                return

            self.person_latched = True
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
                'Nav2 did not accept the cancellation request. '
                'Wave will not start.'
            )
            return

        # Do not start the wave here. A cancellation response only means the
        # request was accepted. _nav_result_cb starts the wave after Nav2
        # reports the goal's terminal CANCELED result.
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
    # Wave sequence
    # ============================================================

    def do_wave(self):
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

        return self._go_to_named(
            component=self.arm,
            state_name='ready',
            description='return right arm to ready',
        )

    def _go_to_named(
        self,
        component,
        state_name,
        description,
    ):
        self.get_logger().info(f'Planning motion: {description}')

        try:
            component.set_start_state_to_current_state()
            component.set_goal_state(configuration_name=state_name)
        except Exception as error:
            self.get_logger().error(
                f'Failed to configure motion "{description}": {error}'
            )
            return False

        return self._plan_and_execute(component, description)

    def _go_to_arm_positions(self, positions, description):
        if len(positions) != len(ARM_R_JOINT_NAMES):
            self.get_logger().error(
                f'Invalid target for "{description}": expected '
                f'{len(ARM_R_JOINT_NAMES)} values, got {len(positions)}'
            )
            return False

        self.get_logger().info(f'Planning motion: {description}')

        try:
            self.arm.set_start_state_to_current_state()
            goal_state = RobotState(self.robot_model)
            goal_state.set_joint_group_positions('arm_r', positions)
            goal_state.update()
            self.arm.set_goal_state(robot_state=goal_state)
        except Exception as error:
            self.get_logger().error(
                f'Failed to configure motion "{description}": {error}'
            )
            return False

        return self._plan_and_execute(self.arm, description)

    def _plan_and_execute(self, component, description):
        try:
            plan_result = component.plan()
        except Exception as error:
            self.get_logger().error(
                f'Planning raised an exception for "{description}": '
                f'{error}'
            )
            return False

        if not plan_result or not hasattr(plan_result, 'trajectory'):
            self.get_logger().error(
                f'Planning failed for "{description}"'
            )
            return False

        self.get_logger().info(f'Executing motion: {description}')

        try:
            execution_result = self.moveit.execute(
                plan_result.trajectory,
                controllers=[],
            )
        except Exception as error:
            self.get_logger().error(
                f'Execution raised an exception for "{description}": '
                f'{error}'
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
