

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations
import math
import json
from nav_msgs.msg import Path

class StatusWzPublisher(Node):
    def __init__(self):
        super().__init__('status_wz_publisher', namespace='red_standard_robot1')

        # 加载多边形 JSON 参数
        default_polygons = json.dumps([
            [[10.43, 10], [10.78, -10.35], [-12.98, -10.85], [-12.68, 10.4]]
        ])
        self.declare_parameter('polygons_json', default_polygons)
        polygons_str = self.get_parameter('polygons_json').get_parameter_value().string_value
        try:
            self.polygons = json.loads(polygons_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'解析 polygons_json 失败: {e}')
            self.polygons = []

        # TF 监听
        qos_tf = QoSProfile(depth=10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos_tf)

        # 参数：角速度限幅
        self.declare_parameter('wz_limit', 3.14)
        self.wz_limit = self.get_parameter('wz_limit').get_parameter_value().double_value

        # 状态记录
        self.robot_position = None
        self.inside_polygon_id = 0
        self.last_plan_time = self.get_clock().now()
        self.plan_timeout_sec = 0.5

        # 订阅 local_plan，发布 status_wz
        self.sub_plan = self.create_subscription(Path, 'local_plan', self.plan_callback, 10)
        self.pub_status = self.create_publisher(Twist, 'status_wz', 0)

        # 定时器：检查是否超时清零
        self.create_timer(0.2, self.check_plan_timeout)

        self.get_logger().info(f'节点启动，已加载 {len(self.polygons)} 个多边形')

    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'chassis', rclpy.time.Time())
            x, y = trans.transform.translation.x, trans.transform.translation.y
            self.robot_position = [x, y]
            self.inside_polygon_id = self.is_inside_any_polygon(self.robot_position)
        except TransformException as e:
            self.get_logger().warn(f'TF 获取失败: {e}')
            self.robot_position = None
            self.inside_polygon_id = 0

    def plan_callback(self, msg: Path):
        # 更新收到路径时间
        self.last_plan_time = self.get_clock().now()
        self.get_robot_position()

        # 获取底盘 yaw
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'chassis', rclpy.time.Time())
            q = trans.transform.rotation
            _, _, chassis_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        except TransformException:
            chassis_yaw = 0.0

        # 提取路径最后两点，计算原始路径方向
        poses = msg.poses
        if len(poses) < 2 or self.robot_position is None:
            return
        x1, y1 = poses[-2].pose.position.x, poses[-2].pose.position.y
        x2, y2 = poses[-1].pose.position.x, poses[-1].pose.position.y
        raw_yaw = math.atan2(y2 - y1, x2 - x1)

        # 对齐到最近边方向，并选最小旋转候选
        edge_raw = round(raw_yaw / (math.pi/2)) * (math.pi/2)
        cand1 = edge_raw
        cand2 = (edge_raw + math.pi) % (2*math.pi)
        cand2 = (cand2 + math.pi) % (2*math.pi) - math.pi
        err1 = abs(self._angle_diff(cand1, chassis_yaw))
        err2 = abs(self._angle_diff(cand2, chassis_yaw))
        path_yaw = cand1 if err1 < err2 else cand2

        # 计算误差并限幅（限制旋转到 ±π/4）
        yaw_err = self._angle_diff(path_yaw, chassis_yaw)
        max_rot = math.pi/4
        yaw_err = max(-max_rot, min(max_rot, yaw_err))
        wz = max(-self.wz_limit, min(self.wz_limit, yaw_err))

        # # 接近目标清零
        # dist = math.hypot(x2 - x1, y2 - y1)
        # if dist < 0.05:
        #     wz = 0.0
        #     self.get_logger().info('接近目标点，清零 wz')

        # 发布 Twist
        out = Twist()
        out.linear.x = float(bool(self.inside_polygon_id))
        out.angular.z = wz
        self.pub_status.publish(out)
        self.get_logger().info(
            f'raw_yaw={raw_yaw:.2f} edge_raw={edge_raw:.2f} chassis_yaw={chassis_yaw:.2f} wz={wz:.2f}'
        )

    def check_plan_timeout(self):
        # 超时未收到路径，则清零 wz
        dt = (self.get_clock().now() - self.last_plan_time).nanoseconds * 1e-9
        if dt > self.plan_timeout_sec:
            out = Twist()
            out.linear.x = float(bool(self.inside_polygon_id))
            out.angular.z = 0.0
            self.pub_status.publish(out)
            self.get_logger().info(f'plan超时 {dt:.2f}s，清零 wz')

    def is_inside_any_polygon(self, point):
        for idx, verts in enumerate(self.polygons, start=1):
            if self._point_in_quad(verts[0], verts[1], verts[2], verts[3], point):
                return idx
        return 0

    @staticmethod
    def _point_in_quad(p1, p2, p3, p4, p):
        def cross(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        return (cross(p1, p2, p) * cross(p3, p4, p) >= 0 and
                cross(p2, p3, p) * cross(p4, p1, p) >= 0)

    @staticmethod
    def _angle_diff(target, current):
        a = target - current
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = StatusWzPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()











# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from rclpy.qos import QoSProfile
# from tf2_ros import Buffer, TransformListener, TransformException
# import tf_transformations
# import math
# import json
# from nav_msgs.msg import Path

# class StatusWzPublisher(Node):
#     def __init__(self):
#         super().__init__('status_wz_publisher', namespace='red_standard_robot1')

#         # 加载多边形 JSON 参数
#         default_polygons = json.dumps([
#             [[-5.43, 5], [5, 10], [10, -10], [-10, -10]]
#         ])
#         self.declare_parameter('polygons_json', default_polygons)
#         polygons_str = self.get_parameter('polygons_json').get_parameter_value().string_value
#         try:
#             self.polygons = json.loads(polygons_str)
#         except json.JSONDecodeError as e:
#             self.get_logger().error(f'解析 polygons_json 失败: {e}')
#             self.polygons = []

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self, qos=QoSProfile(depth=10))

#         # 参数
#         self.declare_parameter('wz_limit', 3.14)
#         self.wz_limit = self.get_parameter('wz_limit').get_parameter_value().double_value

#         self.robot_position = None
#         self.inside_polygon_id = 0

#         # 超时检测
#         self.last_plan_time = self.get_clock().now()
#         self.plan_timeout_sec = 0.5

#         # 订阅与发布
#         self.sub_plan = self.create_subscription(Path, 'local_plan', self.plan_callback, 10)
#         self.pub_status = self.create_publisher(Twist, 'status_wz', 10)

#         # 定时器：检查是否超时清零
#         self.create_timer(0.2, self.check_plan_timeout)

#         self.get_logger().info(f'节点启动，已加载 {len(self.polygons)} 个多边形')

#     def get_robot_position(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'chassis', rclpy.time.Time())
#             x = trans.transform.translation.x
#             y = trans.transform.translation.y
#             self.robot_position = [x, y]
#             self.inside_polygon_id = self.is_inside_any_polygon(self.robot_position)
#             self.get_logger().info(
#                 f'TF 位置: ({x:.2f}, {y:.2f})，在多边形 #{self.inside_polygon_id}'
#             )
#         except TransformException as e:
#             self.get_logger().warn(f'TF 获取失败: {e}')
#             self.robot_position = None
#             self.inside_polygon_id = 0






#     # def plan_callback(self, msg: Path):
#     #     self.last_plan_time = self.get_clock().now()
#     #     self.get_robot_position()

#     #     # 获取底盘当前 yaw
#     #     try:
#     #         tf_odom = self.tf_buffer.lookup_transform('odom', 'chassis', rclpy.time.Time())
#     #         q = tf_odom.transform.rotation
#     #         _, _, chassis_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
#     #     except TransformException:
#     #         chassis_yaw = 0.0

#     #     # 提取 local_plan 最后两点
#     #     poses = msg.poses
#     #     if len(poses) < 2 or self.robot_position is None:
#     #         return
#     #     x1, y1 = poses[-2].pose.position.x, poses[-2].pose.position.y
#     #     x2, y2 = poses[-1].pose.position.x, poses[-1].pose.position.y

#     #     # 计算原始路径朝向
#     #     raw_yaw = math.atan2(y2 - y1, x2 - x1)

#     #     # 对齐到最近的边（0, 90°, 180°, 270°）
#     #     edge_yaw = round(raw_yaw / (math.pi/2)) * (math.pi/2)
#     #     path_yaw = edge_yaw

#     #     # 计算误差并限幅
#     #     yaw_err = self._angle_diff(path_yaw, chassis_yaw)
#     #     wz = max(-self.wz_limit, min(self.wz_limit, yaw_err))

#     #     # 接近目标清零
#     #     dist = math.hypot(x2 - x1, y2 - y1)
#     #     if dist < 0.05:
#     #         wz = 0.0

#     #     # 发布
#     #     out = Twist()
#     #     out.linear.x = float(bool(self.inside_polygon_id))
#     #     out.angular.z = wz
#     #     self.pub_status.publish(out)
#     #     self.get_logger().info(f'raw_yaw={raw_yaw:.2f} edge_yaw={edge_yaw:.2f} chassis_yaw={chassis_yaw:.2f} wz={wz:.2f}')



# def plan_callback(self, msg: Path):
#     self.last_plan_time = self.get_clock().now()
#     self.get_robot_position()

#     # 获取底盘当前 yaw
#     try:
#         tf_odom = self.tf_buffer.lookup_transform('odom', 'chassis', rclpy.time.Time())
#         q = tf_odom.transform.rotation
#         _, _, chassis_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
#     except TransformException:
#         chassis_yaw = 0.0

#     poses = msg.poses
#     if len(poses) < 2 or self.robot_position is None:
#         return

#     # 1. 计算路径原始方向
#     x1, y1 = poses[-2].pose.position.x, poses[-2].pose.position.y
#     x2, y2 = poses[-1].pose.position.x, poses[-1].pose.position.y
#     raw_yaw = math.atan2(y2 - y1, x2 - x1)

#     # 2. 对齐到最近的“边”方向：先量化到 0, π/2, π, -π/2
#     edge_raw = round(raw_yaw / (math.pi/2)) * (math.pi/2)

#     # 3. 生成两个候选（正反向），因为对边翻转180°仍是同一条边
#     cand1 = edge_raw
#     cand2 = (edge_raw + math.pi) % (2*math.pi)
#     # 归一化到 [-π, π]
#     cand2 = (cand2 + math.pi) % (2*math.pi) - math.pi

#     # 4. 选择使旋转最小的候选
#     err1 = abs(self._angle_diff(cand1, chassis_yaw))
#     err2 = abs(self._angle_diff(cand2, chassis_yaw))
#     path_yaw = cand1 if err1 < err2 else cand2

#     # 5. 计算误差并限幅
#     yaw_err = self._angle_diff(path_yaw, chassis_yaw)
#     wz = max(-self.wz_limit, min(self.wz_limit, yaw_err))

#     # # 6. 接近目标清零
#     # dist = math.hypot(x2 - x1, y2 - y1)
#     # if dist < 0.05:
#     #     wz = 0.0

#     # 发布
#     out = Twist()
#     out.linear.x = float(bool(self.inside_polygon_id))
#     out.angular.z = wz
#     self.pub_status.publish(out)

#     self.get_logger().info(
#         f'raw_yaw={raw_yaw:.2f} edge={edge_raw:.2f} '
#         f'cand1={cand1:.2f} err1={err1:.2f} cand2={cand2:.2f} err2={err2:.2f} '
#         f'ch_yaw={chassis_yaw:.2f} wz={wz:.2f}'
#     )


















    # def plan_callback(self, msg: Path):
    #     self.last_plan_time = self.get_clock().now()
    #     self.get_robot_position()

    #     # 获取当前底盘角度
    #     try:
    #         trans = self.tf_buffer.lookup_transform('odom', 'chassis', rclpy.time.Time())
    #         q = trans.transform.rotation
    #         _, _, chassis_yaw = tf_transformations.euler_from_quaternion(
    #             [q.x, q.y, q.z, q.w]
    #         )
    #     except TransformException as e:
    #         self.get_logger().warn(f'TF 获取失败: {e}')
    #         chassis_yaw = 0.0

    #     # 提取路径最后一个点和当前位置，计算目标方向角
    #     poses = msg.poses
    #     if len(poses) < 1 or self.robot_position is None:
    #         self.get_logger().warn('路径或机器人位置无效，跳过计算')
    #         return

    #     x2 = poses[-1].pose.position.x
    #     y2 = poses[-1].pose.position.y
    #     x1 = self.robot_position[0]
    #     y1 = self.robot_position[1]

    #     path_yaw = math.atan2(y2 - y1, x2 - x1)
    #     yaw_err = self._angle_diff(path_yaw, chassis_yaw)
    #     wz = max(-self.wz_limit, min(self.wz_limit, yaw_err))

    #     # 判断是否已接近目标点并清零
    #     dist_to_target = math.hypot(x2 - x1, y2 - y1)
    #     if dist_to_target < 0.05:
    #         wz = 0.0
    #         self.get_logger().info('接近目标点，清零 wz')

    #     # 发布
    #     out = Twist()
    #     out.linear.x = float(1 if self.inside_polygon_id else 0)
    #     out.angular.z = wz
    #     self.pub_status.publish(out)

    #     self.get_logger().info(
    #         f'ch_yaw={chassis_yaw:.2f} path_yaw={path_yaw:.2f} wz={wz:.2f}'
#     #     )

#     def check_plan_timeout(self):
#         now = self.get_clock().now()
#         dt = (now - self.last_plan_time).nanoseconds * 1e-9
#         if dt > self.plan_timeout_sec:
#             out = Twist()
#             out.linear.x = float(1 if self.inside_polygon_id else 0)
#             out.angular.z = 0.0
#             self.pub_status.publish(out)
#             self.get_logger().info(f'plan超时 {dt:.2f}s，清零 wz')

#     def is_inside_any_polygon(self, point):
#         for idx, verts in enumerate(self.polygons, start=1):
#             if self._point_in_quad(verts[0], verts[1], verts[2], verts[3], point):
#                 return idx
#         return 0

#     @staticmethod
#     def _point_in_quad(p1, p2, p3, p4, p):
#         def cross(a, b, c):
#             return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
#         return (cross(p1, p2, p) * cross(p3, p4, p) >= 0 and
#                 cross(p2, p3, p) * cross(p4, p1, p) >= 0)

#     @staticmethod
#     def _angle_diff(target, current):
#         a = target - current
#         return (a + math.pi) % (2 * math.pi) - math.pi

# def main(args=None):
#     rclpy.init(args=args)
#     node = StatusWzPublisher()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import open3d as o3d
# import math
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import PointCloud2
# from geometry_msgs.msg import Twist
# import sensor_msgs.msg
# import tf_transformations
# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener



# class RobotHandler(Node):
#     def __init__(self):
#         super().__init__('pass_hole')

#    # 创建一个订阅者来接收 local_plan 消息    
        
#         # self.subscription = self.create_subscription(Twist, 'local_plan', self.twist_callback, 10)
#         # self.events_publisher = self.create_publisher(String, 'events', 10)

#         self.target_frame = 'map'
#         self.source_frame = 'chassis'
#         self.robot_position = None  # 机器人当前位置


#         self.declare_parameter('source_frame', 'chassis')             
#         self.source_frame = self.get_parameter(                     
#             'source_frame').get_parameter_value().string_value
#         # Create a parameter for the target coordinate system name
#         self.declare_parameter('target_frame', 'map')             
#         self.target_frame = self.get_parameter(                    
#             'target_frame').get_parameter_value().string_value
        

#         # Create a buffer to save coordinate transformation information
#         self.tf_buffer = Buffer()         
#         # Create a listener for coordinate transformation
#         self.tf_listener = TransformListener(self.tf_buffer, self)  
#         # Create a fixed cycle timer to process coordinate information
#         self.tf_timer = self.create_timer(0.1, self.tf_timer)   


#         self.init_x = 0.0
#         self.init_y = 0.0
#         self.init_yaw = 0.0
#         self.trans_init = np.eye(4)
        
#         self.delete_polygons_vertices = [
#       #逆时针选点
#         # #     [
#         #         [-3.43,0.66，1.57],
#         #         [-1.13,0.66 ，1.57],
#         #         [-0.28,2.26，1.57],
#         #         [-3.18,2.61，1.57]
#         #     ],
  

#         ]






#     def cmd_vel_callback(self, msg):
#         # 获取机器人当前位置
#         self.get_robot_position()

#         if self.robot_position:
#                 # 初始化加速状态
#                 should_accelerate = False
#                 id=self.is_inside_any_polygon(self.robot_position)
#                 # 检查机器人是否在特定的多边形内
#                 if(id!=0):
#                     # 如果在多边形内，检查姿态是否在特定方向范围内
#                     if self.is_yaw_within_range(self.init_yaw, self.delete_polygons_vertices[id-1]):  # 只使用当前多边形的方向
#                         # 如果满足两个条件，则设置加速标志为True
#                         should_accelerate = True

#                 # 如果机器人在指定的多边形内且姿态在特定方向范围内，则加速
#                 if should_accelerate:
#                     scaled_twist = Twist()
#                     scaled_twist.linear.x = 2.5
#                     scaled_twist.linear.y = msg.linear.y
#                     scaled_twist.linear.z = msg.linear.z
#                     scaled_twist.angular = msg.angular  # 保持角速度不变
#                     self.cmd_vel_publisher.publish(scaled_twist)
#                     print("1111111111111111111")
#                     self.get_logger().info("Accelerating within specified polygon and yaw range")
                    
#                 elif id != 0 and not self.is_yaw_within_range(self.init_yaw, self.delete_polygons_vertices[id-1]):  # 只使用当前多边形的方向
#                     scaled_twist = Twist()
#                     scaled_twist.linear.x = 0.6
#                     scaled_twist.linear.y = msg.linear.y
#                     scaled_twist.linear.z = msg.linear.z
#                     scaled_twist.angular = msg.angular  # 保持角速度不变
#                     self.cmd_vel_publisher.publish(scaled_twist)
#                 else:
#                     # 否则，保持原速度不变
#                     self.cmd_vel_publisher.publish(msg)
#                     self.get_logger().info("Not accelerating within specified polygon or yaw range")
#                     print("2222222222222")



#     def get_robot_position(self):
#         try:
#             # 获取当前机器人位置
#             trans = self.tf_buffer.lookup_transform(
#                 self.target_frame,  # 目标坐标系
#                 'chassis',  # 源坐标系
#                 rclpy.time.Time())  # 当前时间
#             self.robot_position = [trans.transform.translation.x, trans.transform.translation.y]
#         except TransformException as ex:
#             # 获取失败时，打印错误信息
#             self.get_logger().info(f'Failed to get robot position: {ex}')
#             self.robot_position = None



#     def remove_points_in_polygon(self, ranges, msg):
#         filtered_ranges = []
#         for i, range_value in enumerate(ranges):
#             angle = msg.angle_min + i * msg.angle_increment

#             # 将激光点变换到指定位置
#             transformed_point = self.transform_point(range_value, angle)

#             id=self.is_inside_any_polygon(transformed_point)
#             # 检查变换后的激光点是否在四边形内
#             if(id==0):
#                 filtered_ranges.append(range_value)
#             else:
#                 filtered_ranges.append(float('nan'))  # 不在区域内的点设置为 NaN

#             # # 检查变换后的激光点是否在四边形内
#             # if not self.is_inside_any_polygon(transformed_point):
#             #     filtered_ranges.append(range_value)
#             # else:
#             #     filtered_ranges.append(float('nan'))  # 不在区域内的点设置为 NaN
#         return filtered_ranges

#     def transform_point(self, range_value, angle):
#         # 将激光点表示为齐次坐标
#         x = range_value * np.cos(angle)
#         y = range_value * np.sin(angle)
#         homogeneous_point = np.array([x, y, 0, 1])

#         # 应用四阶变换矩阵
#         transformed_point = np.dot(self.trans_init, homogeneous_point)

#         # 返回变换后的点的二维坐标
#         return transformed_point[:2]
   
    






#     def is_inside_any_polygon(self, point):
#         index=0
#         add=0
#         # 检查点是否在任何一个四边形内部
#         for polygon_vertices in self.delete_polygons_vertices:
#             add+=1
#             # 从四边形的顶点列表中提取顶点坐标
#             p1, p2, p3, p4 = polygon_vertices
#             # 检查点是否在当前四边形内部
#             if self.IsPointInMatrix(p1, p2, p3, p4, point):
#                 index=add
#                 break
#         return index





#     def is_yaw_within_range(self, yaw,polygon_vertices):
#         # 检查姿态是否在指定范围内
#             # 获取多边形的方向
#         polygon_direction = polygon_vertices[0][2]  # 假设每个多边形的方向都一样，取第一个顶点的方向
#             # 根据多边形的方向确定最小和最大偏航角
#          # 计算小车方向与多边形方向之间的角度差
#         # angle_diff = abs(yaw - polygon_direction)

#         # 规范化角度
#         normalized_yaw = self.normalize_angle(yaw)
#         normalized_polygon_direction = self.normalize_angle(polygon_direction)
#         # 计算小车方向与多边形方向之间的角度差
#         angle_diff = abs(normalized_yaw - normalized_polygon_direction)


#         self.get_logger().info(f"{angle_diff}")
#             # 如果角度差不超过90度且不超过180度，返回True
#         if angle_diff <= 1.57:
#             return True
            
#         # 如果姿态不在任何一个多边形的方向范围内，返回False
#         return False
    


#     def normalize_angle(self, angle):
#         # 规范化角度到 [-pi, pi] 范围内
#         while angle > math.pi:
#             angle -= 2 * math.pi
#         while angle < -math.pi:
#             angle += 2 * math.pi
#         return angle



#     # Tf listening callback(between map and base_link)
#     def tf_timer(self):
#         try:
#             # Obtain the current time of the ROS system
#             now = rclpy.time.Time()   
#             # Monitor the coordinate transformation from the source coordinate system to the target coordinate system at the current time                          
#             trans = self.tf_buffer.lookup_transform(                
#                 self.target_frame,
#                 self.source_frame,
#                 now)
#         # If coordinate transformation acquisition fails, enter an exception report
#         except TransformException as ex:                            
#             self.get_logger().info(
#                 f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
#             return
        
#         # Obtain location information
#         pos  = trans.transform.translation           
#         # Obtain posture information (quaternion)
#         quat = trans.transform.rotation                             
#         euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
#         self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
#           % (self.source_frame, self.target_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))

#         # Update initial position and yaw
#         self.init_x = pos.x
#         self.init_y = pos.y
#         self.init_yaw = euler[2]

#         self.trans_init = self.transformation_matrix(self.init_x, self.init_y, self.init_yaw)


#     def transformation_matrix(self, x, y, yaw):
#         Ro = self.euler_to_rotation_matrix(yaw)
#         Tr = np.eye(4)
#         Tr[:3, :3] = Ro
#         Tr[0, 3] = x
#         Tr[1, 3] = y
#         return Tr

#     def euler_to_rotation_matrix(self, yaw):
#         Ro = np.eye(3)
#         Ro[0, 0] = math.cos(yaw)
#         Ro[0, 1] = -math.sin(yaw)
#         Ro[1, 0] = math.sin(yaw)
#         Ro[1, 1] = math.cos(yaw)
#         return Ro
    
#     def GetCross(self, p1, p2, p):
#         return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p[0] - p1[0]) * (p2[1] - p1[1])

#     def IsPointInMatrix(self, p1, p2, p3, p4, p):
#         isPointIn = self.GetCross(p1, p2, p) * self.GetCross(p3, p4, p) >= 0 and self.GetCross(p2, p3, p) * self.GetCross(p4, p1, p) >= 0
#         return isPointIn
    
# def main(args=None):
#     rclpy.init(args=args)
#     robot_handler = RobotHandler()
#     rclpy.spin(robot_handler)
#     robot_handler.destroy_node()
#     rclpy.shutdown()



# if __name__ == '__main__':
#     main()