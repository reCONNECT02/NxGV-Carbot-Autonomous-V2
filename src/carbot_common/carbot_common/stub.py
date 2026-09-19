"""Phase-1 block skeleton.

Every block node starts as a StubBlockNode built from a BlockSpec: it declares
its YAML parameters (and fails loudly if any are missing), creates its real
publishers/subscribers so `ros2 node info` / `ros2 topic info` already show the
final graph, and reports level STUB. It never publishes motion (the command
owner stub publishes explicit zeros only).

Later phases replace the stub by subclassing CarbotNode directly while keeping
the same node name, topics, message types and YAML keys.
"""
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import rclpy
from carbot_interfaces.msg import NodeStatus

from .node import CarbotNode

PubSpec = Tuple[type, str, object]   # (msg type, topic, qos)


@dataclass
class BlockSpec:
    node: str
    block: str
    title: str
    required: List[str] = field(default_factory=list)
    subs: List[PubSpec] = field(default_factory=list)
    pubs: List[PubSpec] = field(default_factory=list)
    phase: int = 0     # phase that implements it


class StubBlockNode(CarbotNode):

    def __init__(self, spec: BlockSpec, extra: Optional[Callable[['StubBlockNode'], None]] = None):
        super().__init__(spec.node, spec.block, spec.required)
        self.spec = spec
        self.publishers_by_topic = {}
        for msg_type, topic, qos in spec.subs:
            self.sub(msg_type, topic, None, qos)
        for msg_type, topic, qos in spec.pubs:
            self.publishers_by_topic[topic] = self.create_publisher(msg_type, topic, qos)
        if self._level != NodeStatus.ERROR:
            self.set_status(NodeStatus.STUB, 'STUB',
                            f'block {spec.block} {spec.title}: implemented in phase {spec.phase}')
        if extra is not None:
            extra(self)
        self.get_logger().info(
            f'[block {spec.block}] {spec.title} skeleton up '
            f'({len(spec.subs)} inputs, {len(spec.pubs)} outputs, phase {spec.phase})')


def run_stub(spec: BlockSpec, extra: Optional[Callable[[StubBlockNode], None]] = None, args=None) -> None:
    rclpy.init(args=args)
    node = StubBlockNode(spec, extra)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
