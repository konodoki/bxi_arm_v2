"""Thread-safe Pico packet adapter."""

import copy
import struct
import threading

from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


class PicoHand:
    """Provides per-device accessors for the shared Pico data packet."""

    _subscriber_node = None
    _lock = threading.RLock()
    _struct_format = '<4s29fI'
    _expected_size = struct.calcsize(_struct_format)
    _packet_header = b'\xa1\xa2\xa3\xa4'
    _shared_data = {
        'head': {
            'pos': [0.0, 0.0, 0.0],
            'quat': [1.0, 0.0, 0.0, 0.0],
            'joy': [0.0, 0.0],
        },
        'left': {
            'pos': [0.0, 0.0, 0.0],
            'quat': [1.0, 0.0, 0.0, 0.0],
            'trigger': 0.0,
            'grip': 0.0,
            'joy': [0.0, 0.0],
        },
        'right': {
            'pos': [0.0, 0.0, 0.0],
            'quat': [1.0, 0.0, 0.0, 0.0],
            'trigger': 0.0,
            'grip': 0.0,
            'joy': [0.0, 0.0],
        },
    }

    def __init__(self, part: str):
        if part not in self._shared_data:
            raise ValueError(f'unknown Pico part: {part}')
        self.part = part
        self.get_node()

    @classmethod
    def get_node(cls) -> Node:
        """Return the shared ROS subscriber node, creating it if needed."""
        with cls._lock:
            if cls._subscriber_node is None:
                cls._subscriber_node = Node('pico_shared_subscriber')
                cls._subscriber_node.create_subscription(
                    UInt8MultiArray,
                    '/pico/data',
                    cls._static_callback,
                    10,
                )
            return cls._subscriber_node

    @classmethod
    def destroy_shared_node(cls) -> None:
        """Destroy the shared subscriber node."""
        with cls._lock:
            if cls._subscriber_node is not None:
                cls._subscriber_node.destroy_node()
                cls._subscriber_node = None

    @classmethod
    def _static_callback(cls, msg: UInt8MultiArray) -> None:
        data_bytes = bytes(msg.data)
        if len(data_bytes) != cls._expected_size:
            return

        unpacked = struct.unpack(cls._struct_format, data_bytes)
        if unpacked[0] != cls._packet_header:
            return

        floats = unpacked[1:30]
        next_data = {
            'head': {
                'pos': list(floats[0:3]),
                'quat': list(floats[3:7]),
                'joy': [0.0, 0.0],
            },
            'left': {
                'pos': list(floats[7:10]),
                'quat': list(floats[10:14]),
                'trigger': floats[14],
                'grip': floats[15],
                'joy': list(floats[25:27]),
            },
            'right': {
                'pos': list(floats[16:19]),
                'quat': list(floats[19:23]),
                'trigger': floats[23],
                'grip': floats[24],
                'joy': list(floats[27:29]),
            },
        }
        with cls._lock:
            cls._shared_data = next_data

    def _state(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._shared_data[self.part])

    def get_joy(self) -> tuple[float, float]:
        joy = self._state().get('joy', [0.0, 0.0])
        return float(joy[0]), float(joy[1])

    def get_pos(self) -> tuple[float, float, float]:
        """Convert Pico position from (x, y, z) to (-z, -x, y)."""
        pos = self._state()['pos']
        return -pos[2], -pos[0], pos[1]

    def get_quaternion(self, to_ros_format: bool = False) -> list[float]:
        """Remap Pico quaternion axes while preserving the old API."""
        quat = self._state()['quat']
        x, y, z, w = quat
        if to_ros_format:
            return [-z, -x, y, w]
        return [w, -z, -x, y]

    def get_button_values(self) -> tuple[float, float]:
        state = self._state()
        return float(state.get('trigger', 0.0)), float(state.get('grip', 0.0))
