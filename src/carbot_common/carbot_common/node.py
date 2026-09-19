"""CarbotNode: base class for every Carbot node.

* All tunables come from YAML (automatically declared from the parameter
  files the launch passes). A missing key is an ERROR, never a silent default:
  that is how "all tunables in YAML" is enforced.
* Publishes a NodeStatus heartbeat on /carbot/status with the age of every
  input topic, for the System health tab, preflight and the wizard.
"""
import time
from typing import Any, Callable, Dict, Iterable, List, Optional

from carbot_interfaces.msg import NodeStatus
from rclpy.node import Node

from . import topics as T

_REQUIRED = object()


class MissingParameter(RuntimeError):
    pass


class CarbotNode(Node):

    def __init__(self, name: str, block: str = '', required: Iterable[str] = ()):
        super().__init__(name, automatically_declare_parameters_from_overrides=True)
        self.block = block
        self._inputs: Dict[str, float] = {}
        self._level = NodeStatus.OK
        self._state = 'STARTING'
        self._detail = ''
        self._status_pub = self.create_publisher(NodeStatus, T.STATUS, 10)
        missing = [k for k in required if not self.has_parameter(k)]
        if missing:
            self.set_status(NodeStatus.ERROR, 'CONFIG_ERROR', 'missing YAML keys: ' + ', '.join(missing))
            self.get_logger().error(
                f'Missing YAML parameters for {name}: {missing}. '
                'Check carbot_bringup/config/params/*.yaml')
        rate = float(self.p('status_rate_hz', 1.0))
        self.create_timer(1.0 / max(rate, 0.1), self._publish_status)

    # ---------------------------------------------------------------- params
    def p(self, name: str, default: Any = _REQUIRED) -> Any:
        """Read a parameter; raise if missing and no default given."""
        if self.has_parameter(name):
            return self.get_parameter(name).value
        if default is _REQUIRED:
            raise MissingParameter(f'{self.get_name()}: parameter "{name}" not in YAML')
        return default

    def params_under(self, prefix: str) -> Dict[str, Any]:
        """All parameters below a dotted prefix, e.g. params_under('vehicle')."""
        return {k: v.value for k, v in
                self.get_parameters_by_prefix(prefix.rstrip('.')).items()}

    # ---------------------------------------------------------------- inputs
    def track_input(self, topic: str) -> None:
        self._inputs.setdefault(topic, -1.0)

    def touch(self, topic: str) -> None:
        self._inputs[topic] = time.monotonic()

    def input_age(self, topic: str) -> float:
        t = self._inputs.get(topic, -1.0)
        return -1.0 if t < 0 else time.monotonic() - t

    def sub(self, msg_type, topic: str, callback: Optional[Callable] = None, qos=10):
        """create_subscription that also records input freshness."""
        self.track_input(topic)

        def _cb(msg, _topic=topic, _user=callback):
            self.touch(_topic)
            if _user is not None:
                _user(msg)
        return self.create_subscription(msg_type, topic, _cb, qos)

    # ---------------------------------------------------------------- status
    def set_status(self, level: int, state: str, detail: str = '') -> None:
        self._level, self._state, self._detail = level, state, detail

    def _publish_status(self) -> None:
        msg = NodeStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node = self.get_name()
        msg.block = self.block
        msg.level = self._level
        msg.state = self._state
        msg.detail = self._detail
        names: List[str] = sorted(self._inputs)
        msg.input_topics = names
        msg.input_age_s = [float(self.input_age(n)) for n in names]
        self._status_pub.publish(msg)
