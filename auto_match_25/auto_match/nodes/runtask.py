#!/usr/bin/python3

import sys
import os
import yaml
import _thread
import threading
import pickle
import json
import time

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseResult
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
# import common.action
import swiftpro.msg
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray
import tf
import tf.transformations as tf_transformations

# catkin 的 runtask 可执行包装器位于 devel/lib，手写辅助模块仍在 nodes/。
# 显式优先加入源文件同目录，避免上机时报 No module named nav_manager。
NODE_DIR = os.path.dirname(os.path.realpath(__file__))
if NODE_DIR not in sys.path:
    sys.path.insert(0, NODE_DIR)

from nav_manager import NavigationManager
from task_rules import choose_drop_zone, remaining_seconds, should_stop

class SwiftProInterface:
    def __init__(self):
        # 创建控制机械臂的topic发布者
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1)    # 机械臂开关状态发布者


    def set_pose(self, x, y, z):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        # rospy.loginfo(f"set pose {x},{y},{z}")
        self.arm_position_pub.publish(pos)
        rospy.sleep(1)

    def set_pump(self, enable:bool):
        '''
        吸取或释放，设定机械臂气泵状态
        '''
        rospy.loginfo(f" 设定机械臂气泵状态为：{enable}")
        if enable:
            self.arm_pump_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_pump_pub.publish(swiftpro.msg.status(0))

    def set_status(self, lock:bool):
        '''
        设定机械臂开关状态
        '''
        rospy.loginfo(f"set arm status {lock}")
        if lock:
            self.arm_status_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_status_pub.publish(swiftpro.msg.status(0))

class CamAction:
    def __init__(self):
        # 获取标定文件相关信息
        rospack = rospkg.RosPack()
        package_path = os.path.join(rospack.get_path('auto_match'))          # 获取功能包路径
        items_path = os.path.join(package_path, 'config', 'items_config.yaml')  # 获取物体标签路径
        try:
            with open(items_path, "r", encoding="utf8") as f:
                items_content = yaml.load(f.read(), Loader=yaml.FullLoader)
        except Exception:
            rospy.logerr("can't not open file")
            sys.exit(1)
        if isinstance(items_content, type(None)):
            rospy.logerr("items file empty")
            sys.exit(1)

        # 根据yaml文件，确定抓取物品的id号
        self.search_id = [
            items_content["items"][items_content["objects"]["objects_a"]],
            items_content["items"][items_content["objects"]["objects_b"]],
            items_content["items"][items_content["objects"]["objects_c"]],
            items_content["items"][items_content["objects"]["objects_d"]]
        ]

    def detector(self, allowed_ids=None):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][1]:代表第i个物体的x方向上的位置;     cube_list[i][1][2:代表第i个物体的y方向上的位置
        '''
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
        except Exception:
            cube_list.clear()
            return cube_list
        
        # 提取
        allowed = set(self.search_id if allowed_ids is None else allowed_ids)
        for obj in obj_array.detections:
            if not obj.results:
                continue
            obj_id = obj.results[0].id
            if obj_id in allowed:
                x = obj.bbox.center.x
                y = obj.bbox.center.y
                cube_list.append([obj_id, [x, y, 0]])  # ֱ�����ӵ��б�
          
            
        # 筛选出需要的物品 cube_list中的key代表识别物体的ID，value代表位置信息
        # for key, value in obj_dist.items():
        #     if key in self.search_id:
        #         cube_list.append([key, value])

        return cube_list

class ArmAction:
    def __init__(self):

        self.cam = CamAction()

        # 获取标定文件数据/一层
        filename = os.environ['HOME'] + "/thefile.txt"
        try:
            with open(filename, 'r') as f:
                s = f.read()
            arr = s.split()
            self.x_kb = [float(arr[0]), float(arr[1])]
            self.y_kb = [float(arr[2]), float(arr[3])]
        except Exception as e:
            rospy.logerr(f"读取一层标定文件失败: {filename} ({e})")
            raise

        # 获取标定文件数据/二层
        filename1 = os.environ['HOME'] + "/thefile2.txt"
        try:
            with open(filename1, 'r') as f1:
                s1 = f1.read()
            arr1 = s1.split()
            self.x_kb1 = [float(arr1[0]), float(arr1[1])]
            self.y_kb1 = [float(arr1[2]), float(arr1[3])]
        except Exception as e:
            rospy.logerr(f"读取二层标定文件失败: {filename1} ({e})")
            raise
    
        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()

        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)


    #抓取一层代码
    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = -55

        # 寻找物品
        cube_list = self.cam.detector()
        rospy.loginfo(f"待抓取物体: {cube_list}") 
        rospy.sleep(1)
        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0
        
        cube_list.sort(key=lambda item: item[1][1], reverse=True)  

        rospy.loginfo(f"排序后列表: {cube_list}")

        target = cube_list[0]
        x = self.x_kb[0] * target[1][1] + self.x_kb[1]  # ͼ��x �� ��е��x
        y = self.y_kb[0] * target[1][0] + self.y_kb[1]  # ͼ��y �� ��е��y
        z = -55  # �̶�ץȡ�߶�

        print(f"找到物品了！它在: {x}, {y}, {z}")

        # 打开气泵，进行吸取


        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(1)

        self.interface.set_pump(True)
        rospy.sleep(1)
        # 机械臂移动到目标位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(1)
        
        r2.sleep()

        # 抬起目标方块
        print(f"我把物品抬起来了")
        self.interface.set_pose(x, y, z + 120)
        # rospy.sleep(2)
        # r1.sleep()

        self.grasp_status_pub.publish(String("0"))

        return cube_list[0][0]

    #抓取二层香蕉代码
    def grasp1(self, allowed_ids=None):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = -55

        # 寻找物品
        cube_list = self.cam.detector(allowed_ids=allowed_ids)
        rospy.loginfo(f"待抓取物体: {cube_list}") 
        rospy.sleep(1)
        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0
        
        cube_list.sort(key=lambda item: item[1][0]+5*item[1][1], reverse=True)  

        rospy.loginfo(f"排序后列表: {cube_list}")

        target = cube_list[0]
        x = self.x_kb1[0] * target[1][1] + self.x_kb1[1]  # ͼ��x �� ��е��x
        y = self.y_kb1[0] * target[1][0] + self.y_kb1[1]  # ͼ��y �� ��е��y
        z = 45  # �̶�ץȡ�߶�

        print(f"找到物品了！它在: {x}, {y}, {z}")




        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(1)

                # 打开气泵，进行吸取
        self.interface.set_pump(True)
        rospy.sleep(1)


        # 机械臂移动到目标位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(1)

        r2.sleep()

        # 抬起目标方块
        print(f"我把物品抬起来了")
        self.interface.set_pose(x, y, z + 20)
        # rospy.sleep(2)
        r1.sleep()

        self.grasp_status_pub.publish(String("0"))



        return cube_list[0][0]
    
    # def grasp(self):
    #     '''
    #     使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
    #     @return: 抓取到物品的id, 0为未识别到需要的物品
    #     '''
    #     x = 235
    #     y = 0
    #     z = -55

    #     cube_list = self.cam.detector()
    #     rospy.loginfo(f"待抓取: {cube_list}") 

    #     if len(cube_list) == 0:
    #         rospy.logwarn("没有找到物品啊。。。去下一个地方")
    #         self.grasp_status_pub.publish(String("1"))
    #         return 0
        
    #     cube_list.sort(key=lambda item: item[1][1], reverse=True)  
    #     rospy.loginfo(f"排序后列表:{cube_list}")

    #     target = cube_list[0]
    #     x = self.x_kb[0] * target[1][1] + self.x_kb[1]  
    #     y = self.y_kb[0] * target[1][0] + self.y_kb[1]  
    #     z = -55

    #     print(f"找到物品了！它在: {x}, {y}, {z}")

    #     self.interface.set_pose(x, y, z + 20)

    #     self.interface.set_pump(True)
    #     rospy.sleep(1)

    #     def lift_object():
    #         print("我把物品抬起来了")
    #         self.interface.set_pose(x, y, z + 120)

    #     def robot_step_back():
    #         print("=======向后退一点===== ")
    #         self.robot.step_back()

    #     lift_thread = threading.Thread(target=lift_object)
    #     back_thread = threading.Thread(target=robot_step_back)

    #     lift_thread.start()
    #     back_thread.start()

    #     lift_thread.join()
    #     back_thread.join()

    #     self.grasp_status_pub.publish(String("0"))

    #     return cube_list[0][0]
    
    # def grasp1(self):
    #     '''
    #     使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
    #     @return: 抓取到物品的id, 0为未识别到需要的物品
    #     '''
    #     r1 = rospy.Rate(0.25)
    #     r2 = rospy.Rate(10)
    #     x = 235
    #     y = 0
    #     z = -55

    #     # 寻找物品
    #     cube_list = self.cam.detector()
    #     rospy.loginfo(f"待抓取物体: {cube_list}") 
    #     # rospy.sleep(2)
    #     if len(cube_list) == 0:
    #         rospy.logwarn("没有找到物品啊。。。去下一个地方")
    #         self.grasp_status_pub.publish(String("1"))
    #         return 0
        
    #     cube_list.sort(key=lambda item: item[1][1], reverse=True)  

    #     rospy.loginfo(f"排序后列表: {cube_list}")

    #     target = cube_list[0]
    #     x = self.x_kb1[0] * target[1][1] + self.x_kb1[1]  # ͼ��x �� ��е��x
    #     y = self.y_kb1[0] * target[1][0] + self.y_kb1[1]  # ͼ��y �� ��е��y
    #     z = 45  # �̶�ץȡ�߶�

    #     print(f"找到物品了！它在: {x}, {y}, {z}")

    #     # 机械臂移动到目标位置上方
    #     self.interface.set_pose(x, y, z + 20)
    #     # rospy.sleep(1)

    #     # 打开气泵，进行吸取
    #     self.interface.set_pump(True)
    #     rospy.sleep(1)

    #     def lift_object():
    #         print("我把物品抬起来了")
    #         self.interface.set_pose(x, y, z + 120)

    #     def robot_step_back():
    #         print("=======向后退一点===== ")
    #         self.robot.step_back()

    #     lift_thread = threading.Thread(target=lift_object)
    #     back_thread = threading.Thread(target=robot_step_back)

    #     lift_thread.start()
    #     back_thread.start()

    #     lift_thread.join()
    #     back_thread.join()

    #     self.grasp_status_pub.publish(String("0"))

    #     return cube_list[0][0]
    
    def drop(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = -55

        # 默认放置位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(2)

        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        rospy.sleep(1)

        self.arm_grasp_ready()  # 移动机械臂到其他地方
        # rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True

    def drop1(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = 60

        # 默认放置位置
        cube_list = self.cam.detector()
        x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
        y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
        z = 60
        # 默认放置位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(2)

        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        rospy.sleep(1)

        self.arm_grasp_ready()  # 移动机械臂到其他地方
        # rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True
    
    def drop2(self, check=False):

        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = 160
        
        # 默认放置位置
        cube_list = self.cam.detector()
        x = self.x_kb1[0] * cube_list[0][1][1] + self.x_kb1[1]
        y = self.y_kb1[0] * cube_list[0][1][0] + self.y_kb1[1]
        z = 160

        # 默认放置位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(2)

        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        rospy.sleep(1)

        self.arm_grasp_ready()  # 移动机械臂到其他地方
        # rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True

    def arm_position_reset(self):
        '''
        校准机械臂的坐标系, 机械臂因碰撞导致坐标计算出问题时使用
        '''
        r1 = rospy.Rate(10)
        self.interface.set_status(False)
        r1.sleep()
        self.interface.set_status(True)
        r1.sleep()

    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        self.interface.set_pose(130, 0, 35)
        if block:
            rospy.sleep(1)

    def arm_grasp_ready(self, block=False):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(50, 180, 160)  
        if block:
            rospy.sleep(1)

    def arm_grasp_laser(self, block=False):
        self.interface.set_pose(160, 0, 20)
        if block:
            rospy.sleep(1)

class RobotMoveAction:
    def __init__(self):
        # 创建控制spark直走的action客户端
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # 创建控制spark旋转的action客户端
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))


        # 创建获取spark前后距离的service客户端
        try:
            rospy.wait_for_service('/get_distance', timeout=5.0)
        except rospy.ROSException:
            rospy.logwarn("/get_distance 服务未就绪，step_back 可能受影响")
        self.distance_srv = rospy.ServiceProxy(
            'get_distance', common.srv.GetFrontBackDistance)

        # 创建导航地点的话题发布者
        self.goto_local_pub = rospy.Publisher(
            "mark_nav", String, queue_size=1)


    def goto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("go "+name)
        rospy.loginfo(f"已发送go指令至: {name}")

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True

    #不记录方向信息   
    def arriveto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("arrive "+name)
        rospy.loginfo(f"已发送arrivex指令至: {name}")

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True
        
    def step_back(self):
        '''
        后退, 用于抓取或放置后使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.1,
                move_distance=0.1,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    
    def step_go(self,dis):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.15,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True


class AutoAction:
    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('spark_auto_match_node', anonymous=True)

        print("========ready to task===== ")

        # 实例化Cam
        self.cam = CamAction()
        print("========实例化Cam===== ")
        # 实例化Arm
        self.arm = ArmAction()
        print("========实例化Arm===== ")
        # 实例化导航模块（NavigationManager：直接调 move_base action，带超时/卡死/重试）
        # 原 RobotMoveAction 类保留在下方，需要时可切回
        self.robot = NavigationManager()
        print("========实例化Robot===== ")
        
        # 物品 ID 映射 (COCO 标准)
        self.BANANA_ID = 4      # banana
        self.TEDDY_ID  = 1      # teddy bear
        self.CLOCK_ID  = 3      # clock
        self.CUP_ID    = 2      # wine glass

        # ==== 规则：蛋糕（二层中间那块）必须先放 A 区，否则本局蛋糕不计分 ====
        # 《赛项规则拟定版》：赛队进行金砖抓取时，需首先将蛋糕金砖放入得分区 A 区。
        # CAKE_FIRST = False 可以退回旧顺序（第 9 次才去二层），方便对比/回滚。
        self.CAKE_ID = 5        # items_config.yaml 里 cake: 5
        self.CAKE_FIRST = True

        required_marks = [
            "Classification_area", "Collection_A", "Collection_B", "Collection_C", "Collection_D",
            "Sorting_W", "Sorting_S", "Sorting_E", "Sorting_N", "Sorting_M",
            "Step_NW", "Step_NE", "Step_SW", "Step_SE",
        ]
        missing = self.robot.validate_marks(required_marks)
        if missing:
            raise RuntimeError("缺少必需导航点位: %s；请先标定，禁止启动自动任务" % ", ".join(missing))

        # ==== 分类板映射（由 A 组 board_classifier.py 发布到 /board_mapping）====
        # D 组只订阅、不改识别逻辑；识别失败时按物品分散兜底，避免全塞一个区
        self.mapping = {}
        self.board_timeout = 10.0
        task_cfg = self._load_task_config()
        self.default_zone = task_cfg['default_zone']
        self.fallback_zones = task_cfg['fallback_zones']
        self.time_budget_s = task_cfg['time_budget_s']
        self.min_remaining_s = task_cfg['min_remaining_s']
        self.task_deadline = None
        rospy.loginfo("调度参数: 兜底区=%s 按物品兜底=%s 时间预算=%.0fs 预留=%.0fs",
                      self.default_zone, self.fallback_zones,
                      self.time_budget_s, self.min_remaining_s)

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
        self.stop_flag = False  # 任务的启停标志

        # 订阅机械臂手动控制的话题
        self.grasp_sub = rospy.Subscriber("grasp", String, self.grasp_cb)

        rospy.loginfo("spark_auto_match_node is ready")

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        ret = False # 是否导航成功标志
        item_type = 0 
        # self.arm.arm_home()  # 移动机械臂到其他地方

        # ===== 现在开始执行任务 =====
        rospy.loginfo("start task now.")
        # 规则：正式比赛 10 分钟，超时放置不得分且每秒扣 2 分 —— 到点必须停止新抓取
        self.task_deadline = time.time() + self.time_budget_s
        rospy.loginfo("时间预算 %.0fs，截止时刻约 %.0fs（剩余不足 %.0fs 时停止抓取）",
                      self.time_budget_s, self.task_deadline, self.min_remaining_s)
        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
        # ==== 离开起始区,避免在膨胀区域中，导致导航失败 =====
        if not self.robot.step_go(0.3):
            rospy.logerr("起步失败，终止任务")
            return

        if self.stop_flag: return

        # ==== 移动机械臂 =====
        self.arm.arm_position_reset()  # 重置机械臂坐标系

        # robot_thread = threading.Thread(target=lambda: self.robot.step_go(0.4))
        # robot_thread.start()

        # arm_thread = threading.Thread(target=lambda: [self.arm.arm_position_reset(), self.arm.arm_grasp_ready()])
        # arm_thread.start()

        # robot_thread.join()
        # arm_thread.join()

        # # ===== 导航到分类区 =====
        if not self.robot.goto_local("Classification_area"):
            rospy.logerr("无法到达分类区，终止任务")
            return
        rospy.sleep(2)
        self.mapping = self.wait_board_mapping()
        if not self.robot.goto_local("Collection_A"):
            rospy.logerr("无法到达 Collection_A，终止任务")
            return
        rospy.sleep(2)
      
        # 创建任务安排字典，设定前往的抓取地点与次数
        sorting_status_times = {
            "Sorting_W":2,
            "Sorting_S":2,
            "Sorting_E":2,
            "Sorting_N":2,
            "Sorting_M":2,
        }

        drop_times = {
            "Collection_B":0,
            "Collection_C":0,
            "Collection_D":0,
            "Collection_A":0,
        }

        sorting_name = "Sorting_W"
        # pre_sorting = "Sorting_W"
        at_sorting = False
        drop_zone = 'Collection_A'
        total_drop_times = 0
        # =======开始循环运行前往中心区域抓取与放置任务======
        while True:
            # 根据任务安排逐步执行
            print("readying to sort_areas")

            if total_drop_times == 10:
                break
            if not sorting_status_times:
                rospy.logwarn("所有分拣点已耗尽，提前结束任务")
                break

            # 时间预算：剩余时间不够再走一轮（导航+抓取+放置）时就停手，把时间留给收尾
            if should_stop(time.time(), self.task_deadline, self.min_remaining_s):
                rospy.logwarn("时间预算将尽（剩余 %.0fs < 预留 %.0fs），停止继续抓取，共完成 %d 块",
                              remaining_seconds(self.task_deadline, time.time()),
                              self.min_remaining_s, total_drop_times)
                break
            else:
                # 就近选点：从上一个放置区(drop_zone)出发，选距离最近的未抓分拣点，减少空跑
                _candidates = list(sorting_status_times.keys())
                _nearest = self.robot.nearest_mark(_candidates, from_name=drop_zone)
                sorting_name = _nearest if _nearest in _candidates else _candidates[0]
                rospy.loginfo(f"就近选择分拣点: {sorting_name} (候选={_candidates}, 起点={drop_zone})")
                # 规则：第一块(蛋糕)固定去中心 Sorting_M（A 组安全加固保留）
                if self.CAKE_FIRST and total_drop_times == 0:
                    sorting_name = "Sorting_M"

            # =====导航到目标地点====
            if(at_sorting == False):
                if(drop_zone == "Collection_B"):
                    if "Sorting_E" in sorting_status_times:
                        if(sorting_status_times["Sorting_E"] > 0):
                            sorting_name = "Sorting_E"
                if(drop_zone == "Collection_C"):
                    if "Sorting_E" in sorting_status_times:
                        if(sorting_status_times["Sorting_S"] > 0):
                            sorting_name = "Sorting_S"
                if(drop_zone == "Collection_D"):
                    if "Sorting_E" in sorting_status_times:
                        if(sorting_status_times["Sorting_W"] > 0):
                            sorting_name = "Sorting_W"


            if(at_sorting == False):
                if(drop_zone == "Collection_B"):
                    if(sorting_name == 'Sorting_W'):
                        ret = self.robot.arriveto_local("Step_NW")
                    if(sorting_name == 'Sorting_S'):
                        ret = self.robot.arriveto_local('Step_SE')
                if(drop_zone == "Collection_C"):
                    if(sorting_name == 'Sorting_W'):
                        ret = self.robot.arriveto_local("Step_SW")
                    if(sorting_name == 'Sorting_N' or sorting_name == 'Sorting_M'):
                        ret = self.robot.arriveto_local('Step_NE')
                if(drop_zone == "Collection_D"):
                    if(sorting_name == 'Sorting_N' or sorting_name == 'Sorting_M'):
                        ret = self.robot.arriveto_local("Step_NW")
                    if(sorting_name == 'Sorting_E'):
                        ret = self.robot.arriveto_local('Step_SE')

                        
            ret = self.robot.goto_local(sorting_name)  # 导航到分拣点抓取（原误写为 Collection_A）
            at_sorting =True
            rospy.sleep(1) # 停稳

            if self.stop_flag: return

            # =====识别并抓取物体====
            item_type = 0
            if ret: # 判断是否成功到达目标点
                print("========扫描中，准备抓取===== ")
                if(self.CAKE_FIRST and total_drop_times == 0):
                    # 规则要求第一块就放蛋糕：先上二层（grasp1 是二层位姿），再去 A 区
                    rospy.loginfo("按规则先抓二层蛋糕")
                    item_type = self.arm.grasp1(allowed_ids=[self.CAKE_ID])
                else: item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型

                if self.stop_flag: return

            if item_type == 0: # 如果没有识别到物体，将该地点的抓取次数归0
                sorting_status_times.pop(sorting_name, None)
                at_sorting = False
                if self.CAKE_FIRST and total_drop_times == 0:
                    rospy.logerr("Sorting_M 未识别到蛋糕；为避免误抓误放，终止自动任务")
                    return
                continue

            
            # ====放置物品====
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")

            if self.CAKE_FIRST and total_drop_times == 0 and item_type != self.CAKE_ID:
                drop_zone = "Collection_A"   # 第一块没认成蛋糕（模型缺 cake 类）也先放 A，避免占用 B/C/D
                rospy.logwarn(f"第一块未识别为蛋糕(id={item_type})，仍按规则放 A 区")
            else:
                # 蛋糕固定 A -> 分类板映射 -> 按物品兜底（不会把 10 块全塞进一个区）
                drop_zone = choose_drop_zone(
                    item_type, self.mapping, self.fallback_zones,
                    cake_id=self.CAKE_ID, cake_zone="Collection_A",
                    default_zone=self.default_zone)
                if item_type != self.CAKE_ID and not (self.mapping or {}).get(item_type):
                    rospy.logwarn("分类板映射里没有物品 %s，使用兜底区 %s（本场分类板可能没识别出来）",
                                  item_type, drop_zone)
            
            if total_drop_times != 9:
                print("========向后退一点===== ")
                self.robot.step_back()  # 后退


            rospy.loginfo(f"本次物品ID: {item_type} → 放置区: {drop_zone}")

            # === 用动态放置区导航 ===
            if(drop_zone == 'Collection_B'):
                if(sorting_name == 'Sorting_S'):
                    ret = self.robot.arriveto_local("Step_SE")
                if(sorting_name == 'Sorting_W'):
                    ret = self.robot.arriveto_local("Step_NW")
            if(drop_zone == 'Collection_C'):
                if(sorting_name == 'Sorting_N'):
                    ret = self.robot.arriveto_local("Step_NE")
                if(sorting_name == 'Sorting_W'):
                    ret = self.robot.arriveto_local("Step_SW")
            if(drop_zone == 'Collection_D'):
                if(sorting_name == 'Sorting_N'):
                    ret = self.robot.arriveto_local("Step_NW")
                if(sorting_name == 'Sorting_E'):
                    ret = self.robot.arriveto_local("Step_SE")


            ret = self.robot.goto_local(drop_zone)
            rospy.sleep(1)  # 停稳
            if self.stop_flag: return

            if ret:
                if drop_times[drop_zone] == 0:
                    self.arm.drop()  # 放下物品
                    drop_times[drop_zone] += 1

                elif drop_times[drop_zone] == 1:
                    self.arm.drop1()  # 放下物品
                    drop_times[drop_zone] += 1

                elif drop_times[drop_zone] >= 2:
                    if drop_times[drop_zone] > 2:
                        rospy.logwarn(f"{drop_zone} 已堆 {drop_times[drop_zone]} 层，继续用二层位姿放置，注意别撞")
                    self.arm.drop2()  # 放下物品
                    drop_times[drop_zone] += 1
                    
                self.robot.step_back()  # 后退
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                rospy.logerr("仍吸持物体，禁止原地释放；请人工接管并发送 stop")
                self.robot.cancel_all()
                return
            if self.stop_flag: return

            # 下一步
            sorting_status_times[sorting_name] = sorting_status_times[sorting_name] - 1
            if "Sorting_E" in sorting_status_times:
                if(sorting_status_times["Sorting_E"] == 0):
                    sorting_status_times.pop("Sorting_E")
                    # if "Sorting_N" in sorting_status_times:
                    #     sorting_status_times["Sorting_N"] -=1
                    # if "Sorting_S" in sorting_status_times:
                    #     sorting_status_times["Sorting_S"] -=1
            if "Sorting_W" in sorting_status_times:
                if(sorting_status_times["Sorting_W"] == 0):
                    sorting_status_times.pop("Sorting_W")
                    # if "Sorting_N" in sorting_status_times:
                    #     sorting_status_times["Sorting_N"] -=1
                    # if "Sorting_S" in sorting_status_times:
                    #     sorting_status_times["Sorting_S"] -=1
            if "Sorting_N" in sorting_status_times:
                if(sorting_status_times["Sorting_N"] == 0):
                    sorting_status_times.pop("Sorting_N")
                    # if "Sorting_W" in sorting_status_times:
                    #     sorting_status_times["Sorting_W"] -=1
                    # if "Sorting_E" in sorting_status_times:
                    #     sorting_status_times["Sorting_E"] -=1
            if "Sorting_S" in sorting_status_times:
                if(sorting_status_times["Sorting_S"] == 0):
                    sorting_status_times.pop("Sorting_S")
                    # if "Sorting_W" in sorting_status_times:
                    #     sorting_status_times["Sorting_W"] -=1
                    # if "Sorting_E" in sorting_status_times:
                    #     sorting_status_times["Sorting_E"] -=1
            at_sorting = False
            total_drop_times += 1


        self.arm.arm_home()
        # act.goto_local("sp")

        rospy.logwarn("***** task finished *****")
        rospy.logwarn("if you want to run task again. ")
        rospy.logwarn("Re-send a message to hm_task_cmd topic. ")
        rospy.logwarn("Or press Ctrl+C to exit the program")
        
    def _load_task_config(self):
        '''
        读取调度参数：兜底放置区、按物品的兜底分区、时间预算。
        这些都放在 items_config.yaml 的 task 段，改参数不用改代码。
        '''
        cfg = {
            'default_zone': 'Collection_B',
            'fallback_zones': {},
            'time_budget_s': 540.0,
            'min_remaining_s': 90.0,
        }
        try:
            path = os.path.join(rospkg.RosPack().get_path('auto_match'), 'config', 'items_config.yaml')
            with open(path, 'r', encoding='utf8') as f:
                content = yaml.load(f.read(), Loader=yaml.FullLoader) or {}
        except Exception as exc:
            rospy.logwarn(f"读取 task 配置失败({exc})，使用默认值")
            return cfg

        cfg['default_zone'] = str(content.get('board_fallback_zone', 'Collection_B'))
        task = content.get('task') or {}
        cfg['time_budget_s'] = float(task.get('time_budget_s', 540.0))
        cfg['min_remaining_s'] = float(task.get('min_remaining_s', 90.0))

        # 按物品名配置的兜底区 -> 转成 ID
        items = content.get('items') or {}
        by_name = task.get('fallback_by_name') or {}
        for name, zone in by_name.items():
            if name in items:
                cfg['fallback_zones'][int(items[name])] = str(zone)
            else:
                rospy.logwarn(f"task.fallback_by_name 里的类别 {name!r} 不在 items 段，已忽略")
        return cfg

    def wait_board_mapping(self):
        '''
        等分类板识别结果（A 组 board_classifier.py 发布在 /board_mapping）。
        @return {item_id: 'Collection_X'}；超时或解析失败返回 {}，调用处会退回 default_zone
        '''
        started = time.time()
        try:
            rospy.wait_for_service("/board_classifier/capture", timeout=min(3.0, self.board_timeout))
            response = rospy.ServiceProxy("/board_classifier/capture", Trigger)()
            if not response.success:
                raise RuntimeError(response.message)
            capture_id = int(response.message)
        except Exception as exc:
            rospy.logwarn(f"无法启动分类板采集({exc})，本场使用兜底放置区 {self.default_zone}")
            return {}

        # latched 话题在新采集完成前会先返回上一局结果，因此必须校验 capture_id。
        while not rospy.is_shutdown():
            remaining = self.board_timeout - (time.time() - started)
            if remaining <= 0:
                break
            try:
                msg = rospy.wait_for_message(
                    "/board_mapping", String, timeout=min(1.0, remaining))
                payload = json.loads(msg.data)
                if int(payload.get("capture_id", -1)) != capture_id:
                    rospy.sleep(0.05)
                    continue
                if not payload.get("stable", False):
                    rospy.sleep(0.05)
                    continue
                mapping = {int(k): str(v) for k, v in payload.get("mapping", {}).items()}
                expected_ids = {self.TEDDY_ID, self.CLOCK_ID, self.BANANA_ID}
                board_zones = {mapping.get(item_id) for item_id in expected_ids}
                if board_zones != {"Collection_B", "Collection_C", "Collection_D"}:
                    rospy.logwarn("分类板映射不完整或区域重复: %s", mapping)
                    rospy.sleep(0.05)
                    continue
                rospy.loginfo(f"分类板映射(capture_id={capture_id}): {mapping}")
                return mapping
            except rospy.ROSException:
                continue
            except Exception as exc:
                rospy.logwarn(f"分类板消息解析失败: {exc}")
                rospy.sleep(0.05)

        rospy.logwarn(f"分类板识别超时，本场使用兜底放置区 {self.default_zone}")
        return {}

    def detect_classification_board(self): 
        # 已废弃：现由 wait_board_mapping() 从 /board_mapping 读取（A 组 board_classifier 发布）。
        # 保留仅为兼容/参考，run_task 不再调用。
        rospy.loginfo("开始检测分类版图案...")

        zones = ["Collection_B", "Collection_C", "Collection_D"]
        mapping = {}

        while not rospy.is_shutdown():
            try:
                # 获取分类版信息
                obj_array = rospy.wait_for_message("/objects", Detection2DArray, timeout=5)
            except Exception:
                rospy.logwarn("未检测到分类版物体，继续等待...")
                continue  # 循环

            objs = []
            for obj in obj_array.detections:
                item_id = obj.results[0].id
                if item_id in (10, 20, 30, 40, 99):  # 场地标记/底板，不参与分类板映射  
                    continue
                item_x = obj.bbox.center.x
                objs.append((item_id, item_x))

            if len(objs) < 3:
                rospy.logwarn(f"当前识别到的物体数量: {len(objs)}继续等待...")
                continue  # 未识别到三个，继续等待

            # 排序物体从左到右
            objs.sort(key=lambda x: x[1])

            # 填充映射
            for i, (item_id, _) in enumerate(objs[:3]):
                mapping[item_id] = zones[i]

            rospy.loginfo(f"分类版检测: {mapping}")
            return mapping  # 检测到三个物体后返回

    def task_cmd_cb(self,flag):
        command = str(flag.data).strip().lower()
        if command in ("1", "true", "start", "run"):
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.task_run_th = threading.Thread(target=self._run_task_guarded, args=())
                self.task_run_th.daemon = True
                self.task_run_th.start()
                rospy.loginfo("start task!!!")
            else:
                rospy.logwarn("任务已经在运行")
        elif command in ("0", "false", "stop", "cancel"):
            self.stop_flag = True
            self.robot.cancel_all()
            rospy.logwarn("已请求停止任务并取消导航目标")
        else:
            rospy.logwarn("无法识别 task_start_flag=%r；使用 start/stop 或 1/0", flag.data)

    def _run_task_guarded(self):
        try:
            self.run_task()
        except Exception:
            rospy.logerr("自动任务异常退出", exc_info=True)
            self.robot.cancel_all()

    def grasp_cb(self, msg):
        if not self.task_run_th.is_alive():
            if msg.data == "1":
                self.arm.grasp()
            elif msg.data == "0":
                self.arm.drop()
                self.arm.arm_grasp_ready()
            else:
                rospy.logwarn("grasp msg error")


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
