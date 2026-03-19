import os
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from autopatrol_interfaces.srv import SpeechText


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)

        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.declare_parameter('image_save_path', '.')

        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.image_save_path = self.get_parameter('image_save_path').value

        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10
        )

        # 语音服务 client
        self.speech_client = self.create_client(SpeechText, '/speech_text')
        self._speech_service_waited = False

        self._worker_thread = None
        self._stop_event = threading.Event()

        self.get_logger().info('patrol_node started')

    def image_callback(self, msg):
        self.latest_image = msg

    def speach_text(self, text):
        # 为了兼容你原来的函数名，我保留 speach_text 这个名字
        # 实际上这里做的是：调用 /speech_text 服务
        if not self._speech_service_waited:
            self.get_logger().info('等待语音服务 /speech_text ...')
            while not self.speech_client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok() or self._stop_event.is_set():
                    self.get_logger().warn('语音服务等待被中断')
                    return False
                self.get_logger().warn('/speech_text 服务未就绪，继续等待...')
            self._speech_service_waited = True
            self.get_logger().info('/speech_text 服务已连接')

        req = SpeechText.Request()
        req.text = text

        future = self.speech_client.call_async(req)

        # 在工作线程里等待 future 完成
        while rclpy.ok() and not future.done():
            if self._stop_event.is_set():
                self.get_logger().warn('语音播报请求被停止')
                return False
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error('语音服务调用未完成')
            return False

        try:
            response = future.result()
            if response is None:
                self.get_logger().error('语音服务返回为空')
                return False

            if response.result:
                self.get_logger().info(f'语音播报成功: {text}')
                return True
            else:
                self.get_logger().warn(f'语音播报失败: {text}')
                return False

        except Exception as e:
            self.get_logger().error(f'语音服务调用异常: {str(e)}')
            return False

    def start_patrol(self):
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self.get_logger().warn('巡逻线程已经在运行')
            return

        self._worker_thread = threading.Thread(
            target=self.patrol_loop,
            daemon=True
        )
        self._worker_thread.start()
        self.get_logger().info('巡逻线程已启动')

    def stop_patrol(self):
        self._stop_event.set()

    def record_image(self):
        if self.latest_image is None:
            self.get_logger().warn('当前没有图像可保存')
            return

        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn('当前没有位姿，无法保存图像')
            return

        os.makedirs(self.image_save_path, exist_ok=True)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='bgr8'
            )
            filename = os.path.join(
                self.image_save_path,
                f'image_{pose.translation.x:.2f}_{pose.translation.y:.2f}.png'
            )
            ok = cv2.imwrite(filename, cv_image)
            if ok:
                self.get_logger().info(f'图像已保存: {filename}')
            else:
                self.get_logger().error(f'图像保存失败: {filename}')
        except Exception as e:
            self.get_logger().error(f'图像转换或保存失败: {str(e)}')

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        q = quaternion_from_euler(0, 0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def init_robot_pose(self):
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(
            self.initial_point_[0],
            self.initial_point_[1],
            self.initial_point_[2]
        )
        self.setInitialPose(init_pose)
        self.get_logger().info('已设置初始位姿，等待 Nav2 激活...')
        self.waitUntilNav2Active()
        self.get_logger().info('Nav2 已激活')

    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value

        if len(self.target_points_) % 3 != 0:
            self.get_logger().error('target_points 参数长度不是 3 的倍数')
            return points

        for index in range(len(self.target_points_) // 3):
            x = self.target_points_[index * 3]
            y = self.target_points_[index * 3 + 1]
            yaw = self.target_points_[index * 3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取到目标点: {index} -> ({x}, {y}, {yaw})')

        return points

    def nav_to_pose(self, target_pose):
        self.goToPose(target_pose)
        self.get_logger().info('已发送导航目标')

        while rclpy.ok() and not self._stop_event.is_set():
            if self.isTaskComplete():
                break

            feedback = self.getFeedback()
            if feedback:
                try:
                    eta = (
                        Duration.from_msg(
                            feedback.estimated_time_remaining
                        ).nanoseconds / 1e9
                    )
                    self.get_logger().info(f'预计 {eta:.1f} s 后到达')
                except Exception:
                    pass

            time.sleep(0.2)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
            return False
        else:
            self.get_logger().error('导航结果：返回状态无效')
            return False

    def get_current_pose(self):
        for _ in range(10):
            if self._stop_event.is_set():
                return None
            try:
                tf = self.buffer_.lookup_transform(
                    'map',
                    'base_footprint',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2)
                )
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(
                    f'平移: {transform.translation}, 旋转欧拉角: {rotation_euler}'
                )
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能获取坐标变换，原因: {str(e)}')
                time.sleep(0.1)

        return None

    def patrol_loop(self):
        try:
            self.init_robot_pose()
            points = self.get_target_points()

            if not points:
                self.get_logger().warn('没有可用的目标点，巡逻线程退出')
                return

            while rclpy.ok() and not self._stop_event.is_set():
                for point in points:
                    if self._stop_event.is_set():
                        break

                    x, y, yaw = point
                    target_pose = self.get_pose_by_xyyaw(x, y, yaw)

                    self.speach_text(f'准备前往目标点，坐标是 {x:.1f}, {y:.1f}')
                    success = self.nav_to_pose(target_pose)

                    if success:
                        time.sleep(0.3)
                        self.speach_text('已到达目标点，开始拍照')
                        self.record_image()
                    else:
                        self.get_logger().warn(f'目标点 ({x}, {y}, {yaw}) 未成功到达')
                        self.speach_text('目标点到达失败')

                    time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f'巡逻线程异常退出: {str(e)}')


def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.start_patrol()

    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        pass
    finally:
        patrol.stop_patrol()
        patrol.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()