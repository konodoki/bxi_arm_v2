import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
class PicoHand:
    # --- 类级别共享变量 ---
    _subscriber_node = None
    _shared_data = {
        'head':  {'pos': [0.0]*3, 'quat': [1.0, 0.0, 0.0, 0.0]},
        'left':  {'pos': [0.0]*3, 'quat': [1.0, 0.0, 0.0, 0.0], 'trigger': 0.0, 'grip': 0.0},
        'right': {'pos': [0.0]*3, 'quat': [1.0, 0.0, 0.0, 0.0], 'trigger': 0.0, 'grip': 0.0}
    }
    _struct_format = "<4s29fI"
    _expected_size = struct.calcsize(_struct_format)

    def __init__(self, part):
        self.part = part
        # 如果还没创建底层订阅节点，则创建一个
        if PicoHand._subscriber_node is None:
            PicoHand._subscriber_node = Node("pico_shared_subscriber")
            PicoHand._subscriber_node.create_subscription(
                UInt8MultiArray,
                '/pico/data',
                PicoHand._static_callback,
                10)

    @staticmethod
    def _static_callback(msg):
        """全局唯一的解析回调"""
        data_bytes = bytes(msg.data)
        if len(data_bytes) != PicoHand._expected_size:
            return

        unpacked = struct.unpack(PicoHand._struct_format, data_bytes)
        if unpacked[0] != b'\xa1\xa2\xa3\xa4':
            return

        floats = unpacked[1:30]
        d = PicoHand._shared_data
        
        # 一次性解析所有数据
        d['head']['pos'],  d['head']['quat']  = list(floats[0:3]),   list(floats[3:7])
        d['left']['pos'],  d['left']['quat']  = list(floats[7:10]),  list(floats[10:14])
        d['left']['trigger'], d['left']['grip'] = floats[14], floats[15]
        d['right']['pos'], d['right']['quat'] = list(floats[16:19]), list(floats[19:23])
        d['right']['trigger'], d['right']['grip'] = floats[23], floats[24]
        d['left']['joy'], d['right']['joy'] = floats[25:27], floats[27:29]
    def get_joy(self):
        return PicoHand._shared_data.get(self.part, {}).get('joy')
    # --- 实例方法：根据各自的 self.part 获取数据 ---
    def get_pos(self):
        """坐标转换: (x,y,z) -> (-z, -x, y)"""
        p = PicoHand._shared_data.get(self.part, {}).get('pos')
        return -p[2], -p[0], p[1]

    def get_quaternion(self, to_ros_format=False):
        """四元数转换: 根据 (w,x,y,z) 顺序进行坐标轴重映射"""
        q = PicoHand._shared_data.get(self.part, {}).get('quat')
        x, y, z,w = q
        # 根据你的 get_pos 逻辑 (-z, -x, y)，对应的四元数分量映射如下：
        if to_ros_format:
            # 返回 ROS 顺序 [x', y', z', w] -> [-z, -x, y, w]
            return [-z, -x, y, w]
        # 返回 wxyz 顺序 [w, x', y', z']
        return [w, -z, -x, y]

    def get_button_values(self):
        target = PicoHand._shared_data.get(self.part, {})
        return (target['trigger'], target['grip']) if 'trigger' in target else (0.0, 0.0)

    @classmethod
    def get_node(cls):
        """暴露底层的 Node 给执行器使用"""
        return cls._subscriber_node