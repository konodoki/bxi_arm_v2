"""Pico trigger to dual-hand joint command bridge."""

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import rclpy
from aero_hand_open_msgs.msg import JointControl
from rclpy.node import Node
from std_msgs.msg import Float32


JOINT_COUNT = 16
NODE_NAME = 'pico_dual_hand_controller'

DEFAULT_OPEN_POSITIONS = [
    1.2,
    0.2,
    0.2,
    0.2,
    *([0.3] * 12),
]
DEFAULT_CLOSE_POSITIONS = [
    1.745,
    *([1.571] * 15),
]
DEFAULT_TIMING = [
    (0.0, 0.2),
    (0.4, 0.8),
    (0.1, 0.9),
    (0.2, 1.0),
    (0.0, 0.4),
    (0.1, 0.8),
    (0.2, 0.9),
    (0.1, 0.3),
    (0.2, 0.6),
    (0.3, 0.9),
    (0.2, 0.4),
    (0.3, 0.7),
    (0.4, 0.9),
    (0.4, 0.5),
    (0.5, 0.8),
    (0.6, 1.0),
]


def _flatten_timing(
    timing: Sequence[Tuple[float, float]]
) -> List[float]:
    return [value for pair in timing for value in pair]


def _pair_timing(values: Iterable[float]) -> List[Tuple[float, float]]:
    timing_values = [float(value) for value in values]
    if len(timing_values) != JOINT_COUNT * 2:
        raise ValueError(
            f'timing must contain {JOINT_COUNT * 2} values, '
            f'got {len(timing_values)}'
        )

    pairs = [
        (timing_values[index], timing_values[index + 1])
        for index in range(0, len(timing_values), 2)
    ]
    for joint_index, (start, end) in enumerate(pairs):
        if end < start:
            raise ValueError(
                f'timing pair {joint_index} must have end >= start'
            )
    return pairs


def _coerce_joint_values(
    name: str,
    values: Iterable[float],
) -> List[float]:
    joint_values = [float(value) for value in values]
    if len(joint_values) != JOINT_COUNT:
        raise ValueError(
            f'{name} must contain {JOINT_COUNT} values, '
            f'got {len(joint_values)}'
        )
    return joint_values


def _coerce_timing_pairs(
    timing: Iterable[Tuple[float, float]]
) -> List[Tuple[float, float]]:
    pairs = []
    for joint_index, pair in enumerate(timing):
        if len(pair) != 2:
            raise ValueError(
                f'timing pair {joint_index} must contain 2 values'
            )

        start, end = float(pair[0]), float(pair[1])
        if end < start:
            raise ValueError(
                f'timing pair {joint_index} must have end >= start'
            )
        pairs.append((start, end))

    if len(pairs) != JOINT_COUNT:
        raise ValueError(
            f'timing must contain {JOINT_COUNT} pairs, got {len(pairs)}'
        )
    return pairs


@dataclass(frozen=True)
class HandProfile:
    """Open/close profile and per-joint timing for one hand."""

    open_positions: Sequence[float]
    close_positions: Sequence[float]
    timing: Sequence[Tuple[float, float]]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'open_positions',
            _coerce_joint_values('open_positions', self.open_positions),
        )
        object.__setattr__(
            self,
            'close_positions',
            _coerce_joint_values('close_positions', self.close_positions),
        )
        object.__setattr__(
            self,
            'timing',
            _coerce_timing_pairs(self.timing),
        )

    def interpolate(self, trigger_value: float) -> List[float]:
        """Return target joint positions for a trigger value in [0, 1]."""
        clamped_trigger = max(0.0, min(1.0, float(trigger_value)))
        targets = []

        for open_pos, close_pos, (start, end) in zip(
            self.open_positions,
            self.close_positions,
            self.timing,
        ):
            if clamped_trigger <= start:
                fraction = 0.0
            elif clamped_trigger >= end:
                fraction = 1.0
            elif end == start:
                fraction = 1.0
            else:
                fraction = (clamped_trigger - start) / (end - start)

            targets.append(open_pos + (close_pos - open_pos) * fraction)

        return targets


class PicoDualHandNode(Node):
    """ROS node that maps Pico triggers to hand joint commands."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self._declare_parameters()
        self.left_hand = self._load_hand_profile('left')
        self.right_hand = self._load_hand_profile('right')

        left_trigger_topic = self._get_string_parameter(
            'left.trigger_topic'
        )
        right_trigger_topic = self._get_string_parameter(
            'right.trigger_topic'
        )
        left_output_topic = self._get_string_parameter(
            'left.output_topic'
        )
        right_output_topic = self._get_string_parameter(
            'right.output_topic'
        )

        self.left_pub = self.create_publisher(
            JointControl,
            left_output_topic,
            10,
        )
        self.right_pub = self.create_publisher(
            JointControl,
            right_output_topic,
            10,
        )
        self.create_subscription(
            Float32,
            left_trigger_topic,
            self.left_cb,
            10,
        )
        self.create_subscription(
            Float32,
            right_trigger_topic,
            self.right_cb,
            10,
        )

        self.get_logger().info(
            'Dual hand controller started: '
            f'{left_trigger_topic} -> {left_output_topic}, '
            f'{right_trigger_topic} -> {right_output_topic}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'left.trigger_topic',
            '/pico/left_trigger',
        )
        self.declare_parameter(
            'right.trigger_topic',
            '/pico/right_trigger',
        )
        self.declare_parameter(
            'left.output_topic',
            '/left/joint_control',
        )
        self.declare_parameter(
            'right.output_topic',
            '/right/joint_control',
        )

        for side in ('left', 'right'):
            self.declare_parameter(
                f'{side}.open_positions',
                list(DEFAULT_OPEN_POSITIONS),
            )
            self.declare_parameter(
                f'{side}.close_positions',
                list(DEFAULT_CLOSE_POSITIONS),
            )
            self.declare_parameter(
                f'{side}.timing',
                _flatten_timing(DEFAULT_TIMING),
            )

    def _get_string_parameter(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'parameter {name} must be a non-empty string')
        return value

    def _load_hand_profile(self, side: str) -> HandProfile:
        return HandProfile(
            open_positions=_coerce_joint_values(
                f'{side}.open_positions',
                self.get_parameter(f'{side}.open_positions').value,
            ),
            close_positions=_coerce_joint_values(
                f'{side}.close_positions',
                self.get_parameter(f'{side}.close_positions').value,
            ),
            timing=_pair_timing(
                self.get_parameter(f'{side}.timing').value
            ),
        )

    def _publish_joint_control(
        self,
        publisher,
        hand: HandProfile,
        trigger_value: float,
    ) -> None:
        joint_msg = JointControl()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.target_positions = hand.interpolate(trigger_value)
        publisher.publish(joint_msg)

    def left_cb(self, msg: Float32) -> None:
        self._publish_joint_control(
            self.left_pub,
            self.left_hand,
            msg.data,
        )

    def right_cb(self, msg: Float32) -> None:
        self._publish_joint_control(
            self.right_pub,
            self.right_hand,
            msg.data,
        )


def main(args=None) -> None:
    """Run the dual-hand controller node."""
    rclpy.init(args=args)
    node = PicoDualHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
