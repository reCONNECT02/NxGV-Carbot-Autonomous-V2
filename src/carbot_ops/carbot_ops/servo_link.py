"""Non-blocking parameter link to the base servo_controller (calibration wizard, phase 8).

The wizard runs a single-threaded executor, so it can never wait for a service
reply inside a callback (calib_tools.RemoteParams spins its own node and would
deadlock here). This link sends the request and calls done(...) from the
executor when the reply arrives, or done(None / False) after timeout_s.

Interface used by the steps (pure code, faked in the tests):
    get(names, done)    done({name: value} | None)
    set(values, done)   done(True | False)
"""
import time
from typing import Any, Callable, Dict, List, Optional

SERVO = 'servo_controller'      # same target as calib_odometry / calib_steering


class ServoLink:

    def __init__(self, node, target: str = SERVO, timeout_s: float = 3.0):
        from rcl_interfaces.srv import GetParameters, SetParameters
        self.node, self.target, self.timeout_s = node, target.lstrip('/'), float(timeout_s)
        self._get = node.create_client(GetParameters, f'/{self.target}/get_parameters')
        self._set = node.create_client(SetParameters, f'/{self.target}/set_parameters')
        self._pending: List[Dict] = []
        node.create_timer(0.2, self._expire)

    def _call(self, cli, req, on_reply: Callable[[Any], None], on_fail: Callable[[], None]) -> None:
        if not cli.service_is_ready():
            on_fail()
            return
        fut = cli.call_async(req)
        entry = {'fut': fut, 'until': time.monotonic() + self.timeout_s, 'fail': on_fail}
        self._pending.append(entry)

        def _done(f):
            if entry not in self._pending:
                return                               # already timed out
            self._pending.remove(entry)
            try:
                on_reply(f.result())
            except Exception:  # noqa: BLE001
                on_fail()
        fut.add_done_callback(_done)

    def _expire(self) -> None:
        now = time.monotonic()
        for e in [e for e in self._pending if now > e['until']]:
            self._pending.remove(e)
            e['fut'].cancel()
            e['fail']()

    def get(self, names: List[str], done: Callable[[Optional[Dict[str, Any]]], None]) -> None:
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        self._call(self._get, GetParameters.Request(names=list(names)),
                   lambda r: done({n: parameter_value_to_python(v) for n, v in zip(names, r.values)}),
                   lambda: done(None))

    def set(self, values: Dict[str, Any], done: Callable[[bool], None]) -> None:
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        req = SetParameters.Request(parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in values.items()])
        self._call(self._set, req, lambda r: done(all(x.successful for x in r.results)), lambda: done(False))
