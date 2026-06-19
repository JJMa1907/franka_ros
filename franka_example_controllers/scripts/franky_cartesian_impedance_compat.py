#!/usr/bin/env python3
from __future__ import annotations

"""franky-based replacement for cartesian impedance control.

Modes:
- ros: ROS interface compatibility bridge (default)
- cli: interactive shell for manual control
"""

import argparse
from collections import deque
import importlib.util
import json
import os
import select
import shlex
import shutil
import subprocess
import sys
import threading
import time
from urllib.parse import urlparse
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple

BOOT_LOG = os.environ.get("FRANKY_COMPAT_BOOT_LOG", "").strip()


def _boot_log(message: str) -> None:
    if not BOOT_LOG:
        return
    try:
        with open(BOOT_LOG, "a", encoding="utf-8") as handle:
            handle.write(f"{time.time():.3f} pid={os.getpid()} exe={sys.executable} {message}\n")
    except Exception:
        pass


def _append_no_proxy(hosts: Sequence[str]) -> None:
    current_items = []
    for name in ("NO_PROXY", "no_proxy"):
        current_items.extend([item.strip() for item in os.environ.get(name, "").split(",") if item.strip()])
    seen = set()
    merged = []
    for item in current_items + [host for host in hosts if host]:
        if item not in seen:
            merged.append(item)
            seen.add(item)
    value = ",".join(merged)
    if value:
        os.environ["NO_PROXY"] = value
        os.environ["no_proxy"] = value


def _ensure_ros_no_proxy() -> None:
    hosts = ["localhost", "127.0.0.1", "::1"]
    for env_name in ("ROS_IP", "ROS_HOSTNAME"):
        value = os.environ.get(env_name, "").strip()
        if value:
            hosts.append(value)
    master_uri = os.environ.get("ROS_MASTER_URI", "").strip()
    if master_uri:
        parsed = urlparse(master_uri)
        if parsed.hostname:
            hosts.append(parsed.hostname)
    _append_no_proxy(hosts)
    _boot_log(f"network: NO_PROXY={os.environ.get('NO_PROXY', '')}")


def _can_import_with(interpreter: str, modules: Sequence[str]) -> bool:
    code = "; ".join([f"import {name}" for name in modules])
    try:
        subprocess.check_call(
            [interpreter, "-c", code],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=os.environ.copy(),
        )
        return True
    except Exception:
        return False


def _ensure_runtime_interpreter() -> None:
    _boot_log("ensure_runtime_interpreter: enter")
    if importlib.util.find_spec("franky") is not None:
        _boot_log("ensure_runtime_interpreter: current interpreter has franky")
        return
    if os.environ.get("FRANKY_COMPAT_REEXEC") == "1":
        _boot_log("ensure_runtime_interpreter: reexec already attempted")
        return

    candidates: List[str] = []
    explicit = os.environ.get("FRANKY_PYTHON", "").strip()
    if explicit:
        candidates.append(explicit)
    path_python3 = shutil.which("python3")
    if path_python3:
        candidates.append(path_python3)
    conda_python = os.path.expanduser("~/anaconda3/bin/python3")
    candidates.append(conda_python)
    path_python = shutil.which("python")
    if path_python:
        candidates.append(path_python)

    seen = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)
        if os.path.abspath(candidate) == os.path.abspath(sys.executable):
            continue
        _boot_log(f"ensure_runtime_interpreter: testing {candidate}")
        if _can_import_with(candidate, ["franky", "rospy"]):
            env = os.environ.copy()
            env["FRANKY_COMPAT_REEXEC"] = "1"
            _boot_log(f"ensure_runtime_interpreter: reexec {candidate}")
            os.execvpe(candidate, [candidate, os.path.abspath(__file__)] + sys.argv[1:], env)
    _boot_log("ensure_runtime_interpreter: no suitable interpreter found")


def _candidate_interpreters() -> List[str]:
    candidates: List[str] = []
    explicit = os.environ.get("FRANKY_PYTHON", "").strip()
    if explicit:
        candidates.append(explicit)
    path_python3 = shutil.which("python3")
    if path_python3:
        candidates.append(path_python3)
    candidates.append(os.path.expanduser("~/anaconda3/bin/python3"))
    path_python = shutil.which("python")
    if path_python:
        candidates.append(path_python)

    result = []
    seen = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)
        result.append(candidate)
    return result


def _find_franky_interpreter() -> Optional[str]:
    for candidate in _candidate_interpreters():
        if _can_import_with(candidate, ["franky"]):
            return candidate
    return None


def _selected_interface_from_argv() -> str:
    for index, arg in enumerate(sys.argv[1:], start=1):
        if arg == "--interface" and index + 1 < len(sys.argv):
            return sys.argv[index + 1]
        if arg.startswith("--interface="):
            return arg.split("=", 1)[1]
    return "ros"


SELECTED_INTERFACE = _selected_interface_from_argv()
if SELECTED_INTERFACE in {"cli", "runtime"}:
    _ensure_runtime_interpreter()
else:
    _boot_log("ensure_runtime_interpreter: skipped for ROS bridge process")
_ensure_ros_no_proxy()
_boot_log("module: after runtime interpreter check")

_FRANKY_MODULE: Optional[Any] = None
_FRANKY_IMPORT_LOCK = threading.Lock()


def _load_franky():
    """Import franky only when a real robot command needs it.

    Keeping this lazy lets the ROS compatibility API come up even if robot
    networking or the franky runtime is slow/problematic during process start.
    """
    global _FRANKY_MODULE
    if _FRANKY_MODULE is not None:
        return _FRANKY_MODULE
    with _FRANKY_IMPORT_LOCK:
        if _FRANKY_MODULE is None:
            import franky

            _FRANKY_MODULE = franky
        return _FRANKY_MODULE

ROS_AVAILABLE = False
ROS_IMPORT_ERROR: Optional[Exception] = None

try:
    _boot_log("module: importing ROS packages")
    import rospy
    from dynamic_reconfigure.server import Server as DynamicReconfigureServer
    from franka_example_controllers.cfg import compliance_paramConfig
    from franka_msgs.msg import FrankaState
    from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String
    from tf.transformations import quaternion_matrix

    ROS_AVAILABLE = True
    _boot_log("module: ROS packages imported")
except Exception as exc:
    ROS_IMPORT_ERROR = exc
    _boot_log(f"module: ROS import failed: {exc}")


def str2bool(value: str) -> bool:
    text = value.strip().lower()
    if text in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def copy_pose(pose: Optional["Pose"]) -> Optional["Pose"]:
    if pose is None:
        return None
    result = Pose()
    result.position.x = pose.position.x
    result.position.y = pose.position.y
    result.position.z = pose.position.z
    result.orientation.x = pose.orientation.x
    result.orientation.y = pose.orientation.y
    result.orientation.z = pose.orientation.z
    result.orientation.w = pose.orientation.w
    return result


def ros_init_argv() -> List[str]:
    argv = [sys.argv[0]]
    for arg in sys.argv[1:]:
        if ":=" not in arg:
            continue
        if arg.startswith("__name:=") or arg.startswith("__log:="):
            continue
        argv.append(arg)
    return argv


class FrankyRuntimeClient:
    """Persistent child process for franky when ROS must stay on system Python."""

    def __init__(self, robot_ip: str, load_gripper: bool, dynamics: float, auto_recover: bool) -> None:
        self.robot_ip = robot_ip
        self.load_gripper = load_gripper
        self.dynamics = dynamics
        self.auto_recover = auto_recover
        self.process: Optional[subprocess.Popen] = None
        self.lock = threading.Lock()
        self.last_error = ""
        self._stderr_thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        if self.process is not None and self.process.poll() is None:
            return True
        interpreter = _find_franky_interpreter()
        if not interpreter:
            self.last_error = "no Python interpreter can import franky"
            return False
        env = os.environ.copy()
        env["FRANKY_COMPAT_REEXEC"] = "1"
        args = [
            interpreter,
            os.path.abspath(__file__),
            "--interface",
            "runtime",
            "--robot-ip",
            self.robot_ip,
            "--load-gripper",
            str(self.load_gripper).lower(),
            "--dynamics",
            str(self.dynamics),
            "--auto-recover",
            str(self.auto_recover).lower(),
        ]
        try:
            self.process = subprocess.Popen(
                args,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env,
            )
            self._stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
            self._stderr_thread.start()
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self.process = None
            return False

    def _drain_stderr(self) -> None:
        process = self.process
        if process is None or process.stderr is None:
            return
        for line in process.stderr:
            print(f"[franky_runtime] {line.rstrip()}", file=sys.stderr, flush=True)

    def stop(self) -> None:
        process = self.process
        self.process = None
        if process is None:
            return
        try:
            if process.stdin:
                process.stdin.write(json.dumps({"command": "shutdown"}) + "\n")
                process.stdin.flush()
        except Exception:
            pass
        try:
            process.terminate()
        except Exception:
            pass

    def request(
        self,
        command: str,
        payload: Optional[Dict[str, Any]] = None,
        timeout: float = 0.5,
        lock_timeout: Optional[float] = None,
    ):
        if not self.start():
            raise RuntimeError(self.last_error)
        assert self.process is not None
        if lock_timeout is None:
            acquired = self.lock.acquire()
        elif lock_timeout <= 0.0:
            acquired = self.lock.acquire(blocking=False)
        else:
            acquired = self.lock.acquire(timeout=lock_timeout)
        if not acquired:
            raise TimeoutError(f"franky runtime is busy: {command}")
        try:
            if self.process.poll() is not None:
                self.last_error = "franky runtime process exited"
                raise RuntimeError(self.last_error)
            try:
                assert self.process.stdin is not None
                assert self.process.stdout is not None
                request = {"command": command, "payload": payload or {}}
                self.process.stdin.write(json.dumps(request) + "\n")
                self.process.stdin.flush()
                ready, _, _ = select.select([self.process.stdout], [], [], timeout)
                if not ready:
                    raise TimeoutError(f"franky runtime command timed out: {command}")
                line = self.process.stdout.readline()
                if not line:
                    raise RuntimeError("franky runtime closed stdout")
                response = json.loads(line)
            except Exception as exc:
                self.last_error = str(exc)
                raise
        finally:
            self.lock.release()
        if not response.get("ok"):
            self.last_error = str(response.get("error", "unknown runtime error"))
            raise RuntimeError(self.last_error)
        self.last_error = ""
        return response.get("result")


class FrankyCartesianController:
    def __init__(
        self,
        robot_ip: str,
        load_gripper: bool,
        dynamics: float,
        auto_recover: bool,
    ) -> None:
        self.robot_ip = robot_ip
        self.load_gripper = load_gripper
        self.dynamics = dynamics
        self.auto_recover = auto_recover
        self.robot = None
        self.gripper: Optional[Any] = None
        self.runtime: Optional[FrankyRuntimeClient] = None
        self._state_cache: Optional[Dict[str, Any]] = None
        self._state_cache_time = 0.0
        self.last_error = ""
        self._connect_lock = threading.Lock()
        self._last_connect_message = ""
        self._last_connect_message_time = 0.0

    def _log_connect_event(self, message: str, min_interval: float = 3.0) -> None:
        now = time.time()
        if message == self._last_connect_message and now - self._last_connect_message_time < min_interval:
            return
        self._last_connect_message = message
        self._last_connect_message_time = now
        print(message, file=sys.stderr, flush=True)

    def connect(self) -> bool:
        if self.runtime is not None:
            try:
                self._log_connect_event(f"[franky_compat] connecting to Franka robot at {self.robot_ip}")
                self.runtime.request("connect", timeout=3.0)
                self.last_error = ""
                self._log_connect_event(f"[franky_compat] connected to Franka robot at {self.robot_ip}", min_interval=0.0)
                return True
            except Exception as exc:
                self.last_error = str(exc)
                self._log_connect_event(
                    f"[franky_compat] failed to connect to Franka robot at {self.robot_ip}: {self.last_error}"
                )
                return False
        if self.robot is not None:
            return True
        with self._connect_lock:
            if self.robot is not None:
                return True
            try:
                self._log_connect_event(f"[franky_compat] connecting to Franka robot at {self.robot_ip}")
                try:
                    franky = _load_franky()
                except Exception:
                    self.runtime = FrankyRuntimeClient(
                        self.robot_ip,
                        self.load_gripper,
                        self.dynamics,
                        self.auto_recover,
                    )
                    self.runtime.request("connect", timeout=3.0)
                    self.last_error = ""
                    self._log_connect_event(
                        f"[franky_compat] connected to Franka robot at {self.robot_ip}",
                        min_interval=0.0,
                    )
                    return True
                robot = franky.Robot(self.robot_ip)
                if self.auto_recover:
                    robot.recover_from_errors()
                robot.relative_dynamics_factor = self.dynamics
                gripper = franky.Gripper(self.robot_ip) if self.load_gripper else None
            except Exception as exc:
                self.last_error = str(exc)
                self.robot = None
                self.gripper = None
                self._log_connect_event(
                    f"[franky_compat] failed to connect to Franka robot at {self.robot_ip}: {self.last_error}"
                )
                return False
            self.robot = robot
            self.gripper = gripper
            self.last_error = ""
            self._log_connect_event(f"[franky_compat] connected to Franka robot at {self.robot_ip}", min_interval=0.0)
            return True

    def disconnect(self) -> None:
        with self._connect_lock:
            if self.runtime is not None:
                self.runtime.stop()
                self.runtime = None
            self.robot = None
            self.gripper = None

    def is_connected(self) -> bool:
        return self.robot is not None or self.runtime is not None

    def _require_robot(self):
        if self.runtime is not None:
            if not self.connect():
                raise RuntimeError(f"robot is not connected: {self.last_error or 'unknown error'}")
            return None
        if self.robot is None and not self.connect():
            raise RuntimeError(f"robot is not connected: {self.last_error or 'unknown error'}")
        return self.robot

    def print_pose(self) -> None:
        robot = self._require_robot()
        state = robot.current_cartesian_state
        pose = getattr(state, "pose", None)
        if pose is None:
            print("current_cartesian_state does not expose pose")
            return
        ee_pose = getattr(pose, "end_effector_pose", pose)
        print(f"pose: {ee_pose}")

    def move_rel(self, dx: float, dy: float, dz: float) -> None:
        if self.runtime is not None or self.robot is None:
            self._require_robot()
            if self.runtime is not None:
                self.runtime.request("move_rel", {"dx": dx, "dy": dy, "dz": dz}, timeout=30.0)
                return
        robot = self._require_robot()
        franky = _load_franky()
        motion = franky.CartesianMotion(franky.Affine([dx, dy, dz]), franky.ReferenceType.Relative)
        robot.move(motion)

    def move_abs(
        self,
        x: float,
        y: float,
        z: float,
        quat: Optional[Sequence[float]],
        asynchronous: bool = False,
    ) -> None:
        if self.runtime is not None or self.robot is None:
            self._require_robot()
            if self.runtime is not None:
                self.runtime.request(
                    "move_abs",
                    {
                        "x": x,
                        "y": y,
                        "z": z,
                        "quat": list(quat) if quat is not None else None,
                        "asynchronous": asynchronous,
                    },
                    timeout=2.0 if asynchronous else 30.0,
                )
                return
        robot = self._require_robot()
        franky = _load_franky()
        if quat is None:
            target = franky.Affine([x, y, z])
        else:
            target = franky.Affine([x, y, z], list(quat))
        motion = franky.CartesianMotion(target)
        robot.move(motion, asynchronous=asynchronous)

    def move_joint(self, joints: Sequence[float], asynchronous: bool = False) -> None:
        if self.runtime is not None or self.robot is None:
            self._require_robot()
            if self.runtime is not None:
                self.runtime.request(
                    "move_joint",
                    {"joints": list(joints), "asynchronous": asynchronous},
                    timeout=2.0 if asynchronous else 30.0,
                )
                return
        robot = self._require_robot()
        franky = _load_franky()
        JointMotion = getattr(franky, "JointMotion", None)
        if JointMotion is None:
            raise RuntimeError("this franky build does not expose JointMotion")
        if len(joints) != 7:
            raise ValueError("joint command must have exactly 7 values")
        robot.move(JointMotion(list(joints)), asynchronous=asynchronous)

    def motion_active(self) -> bool:
        if self.runtime is not None:
            try:
                return bool(self.runtime.request("motion_active", timeout=0.2))
            except Exception as exc:
                self.last_error = str(exc)
                return False
        if self.robot is None:
            return False
        try:
            return bool(self.robot.poll_motion())
        except Exception as exc:
            self.last_error = str(exc)
            return False

    def stop_motion(self) -> None:
        if self.runtime is not None:
            if not self.motion_active():
                return
            self.runtime.request("stop_motion", timeout=2.0)
            return
        robot = self._require_robot()
        try:
            if not self.motion_active():
                return
            robot.stop()
            join_motion = getattr(robot, "join_motion", None)
            if callable(join_motion):
                join_motion(1.0)
        except Exception as exc:
            self.last_error = str(exc)
            raise

    def set_dynamics(self, factor: float) -> None:
        self.dynamics = factor
        if self.runtime is not None:
            self.runtime.request("set_dynamics", {"factor": factor}, timeout=2.0)
            return
        robot = self._require_robot()
        robot.relative_dynamics_factor = factor

    def set_cartesian_stiffness(self, stiffness: Sequence[float]) -> None:
        if self.runtime is not None:
            self.runtime.request("set_cartesian_stiffness", {"stiffness": list(stiffness)}, timeout=2.0)
            return
        robot = self._require_robot()
        if len(stiffness) != 6:
            raise ValueError("cartesian stiffness must have exactly 6 values")
        robot.set_cartesian_impedance(list(stiffness))

    def set_joint_stiffness(self, stiffness: Sequence[float]) -> None:
        if self.runtime is not None:
            self.runtime.request("set_joint_stiffness", {"stiffness": list(stiffness)}, timeout=2.0)
            return
        robot = self._require_robot()
        if len(stiffness) != 7:
            raise ValueError("joint stiffness must have exactly 7 values")
        robot.set_joint_impedance(list(stiffness))

    def gripper_open(self, speed: float) -> None:
        if self.runtime is not None:
            self.runtime.request("gripper_open", {"speed": speed}, timeout=10.0)
            return
        self._require_gripper()
        self.gripper.open(speed)

    def gripper_move(self, width: float, speed: float) -> None:
        if self.runtime is not None:
            self.runtime.request("gripper_move", {"width": width, "speed": speed}, timeout=10.0)
            return
        self._require_gripper()
        success = self.gripper.move(width, speed)
        print(f"gripper move success={success}", file=sys.stderr)

    def gripper_grasp(self, width: float, speed: float, force: float) -> None:
        if self.runtime is not None:
            self.runtime.request("gripper_grasp", {"width": width, "speed": speed, "force": force}, timeout=10.0)
            return
        self._require_gripper()
        success = self.gripper.grasp(width, speed, force)
        print(f"gripper grasp success={success}", file=sys.stderr)

    def gripper_home(self) -> None:
        if self.runtime is not None:
            self.runtime.request("gripper_home", timeout=10.0)
            return
        self._require_gripper()
        if hasattr(self.gripper, "homing"):
            success = self.gripper.homing()
            print(f"gripper homing success={success}", file=sys.stderr)
        else:
            print("gripper.homing() is not available in this franky version", file=sys.stderr)

    def _runtime_state(self) -> Optional[Dict[str, Any]]:
        if self.runtime is None:
            return None
        now = time.time()
        if self._state_cache is not None and now - self._state_cache_time < 0.05:
            return self._state_cache
        try:
            state = self.runtime.request("state", timeout=0.05, lock_timeout=0.0)
            self._state_cache = state if isinstance(state, dict) else None
            self._state_cache_time = now
            return self._state_cache
        except Exception:
            return self._state_cache

    def get_robot_state(self):
        if self.runtime is not None:
            return None
        if self.robot is None:
            return None
        try:
            return self.robot.state
        except Exception:
            self.disconnect()
            return None

    def get_joint_positions(self) -> Optional[List[float]]:
        runtime_state = self._runtime_state()
        if runtime_state and len(runtime_state.get("positions", [])) >= 7:
            return list(runtime_state["positions"][:7])
        if self.robot is None:
            return None
        values = self._to_float_seq(getattr(self.robot, "current_joint_positions", None))
        if values and len(values) >= 7:
            return values[:7]
        state = self.get_robot_state()
        values = self._to_float_seq(getattr(state, "q", None))
        if values and len(values) >= 7:
            return values[:7]
        joint_state = getattr(self.robot, "current_joint_state", None)
        values = self._to_float_seq(getattr(joint_state, "position", None))
        if values and len(values) >= 7:
            return values[:7]
        return None

    def get_joint_velocities(self) -> Optional[List[float]]:
        runtime_state = self._runtime_state()
        if runtime_state and len(runtime_state.get("velocities", [])) >= 7:
            return list(runtime_state["velocities"][:7])
        if self.robot is None:
            return None
        values = self._to_float_seq(getattr(self.robot, "current_joint_velocities", None))
        if values and len(values) >= 7:
            return values[:7]
        state = self.get_robot_state()
        values = self._to_float_seq(getattr(state, "dq", None))
        if values and len(values) >= 7:
            return values[:7]
        joint_state = getattr(self.robot, "current_joint_state", None)
        values = self._to_float_seq(getattr(joint_state, "velocity", None))
        if values and len(values) >= 7:
            return values[:7]
        return None

    def get_joint_efforts(self) -> Optional[List[float]]:
        runtime_state = self._runtime_state()
        if runtime_state and len(runtime_state.get("efforts", [])) >= 7:
            return list(runtime_state["efforts"][:7])
        if self.robot is None:
            return None
        state = self.get_robot_state()
        values = self._to_float_seq(getattr(state, "tau_J", None))
        if values and len(values) >= 7:
            return values[:7]
        joint_state = getattr(self.robot, "current_joint_state", None)
        values = self._to_float_seq(getattr(joint_state, "torque", None))
        if values and len(values) >= 7:
            return values[:7]
        return None

    def get_external_wrench(self) -> List[float]:
        runtime_state = self._runtime_state()
        if runtime_state and len(runtime_state.get("wrench", [])) >= 6:
            return list(runtime_state["wrench"][:6])
        state = self.get_robot_state()
        values = self._to_float_seq(getattr(state, "K_F_ext_hat_K", None))
        if values and len(values) >= 6:
            return values[:6]
        values = self._to_float_seq(getattr(state, "O_F_ext_hat_K", None))
        if values and len(values) >= 6:
            return values[:6]
        return [0.0] * 6

    def get_gripper_width(self) -> float:
        runtime_state = self._runtime_state()
        if runtime_state and "gripper_width" in runtime_state:
            return float(runtime_state["gripper_width"])
        if self.gripper is None:
            return 0.0
        width = getattr(self.gripper, "width", None)
        if width is not None:
            try:
                return float(width)
            except Exception:
                pass
        state = getattr(self.gripper, "state", None)
        width = getattr(state, "width", None)
        try:
            return float(width)
        except Exception:
            return 0.0

    def _require_gripper(self) -> None:
        if self.gripper is None:
            raise RuntimeError("gripper not loaded, start with --load-gripper true")

    def _to_float_seq(self, value) -> Optional[List[float]]:
        if value is None:
            return None
        try:
            return [float(v) for v in value]
        except Exception:
            return None


class FrankyRosCompatBridge:
    """Expose ROS topics compatible with common franka_example_controllers usage."""

    def __init__(
        self,
        ctrl: FrankyCartesianController,
        arm_id: str,
        pose_rate_hz: float,
        connect_on_start: bool,
        external_command_hold_sec: float,
    ) -> None:
        if not ROS_AVAILABLE:
            raise RuntimeError(f"ROS Python packages are not available: {ROS_IMPORT_ERROR}")

        self.ctrl = ctrl
        self.arm_id = arm_id
        self.pose_rate_hz = max(1.0, pose_rate_hz)
        self.external_command_hold_sec = max(0.0, external_command_hold_sec)
        self._external_equilibrium_pose_until = 0.0
        self.impedance_mode = True
        self.sequence_number = 0
        self.last_command_pose: Optional[Pose] = Pose()
        self.last_command_pose.position.x = 0.4
        self.last_command_pose.position.y = 0.0
        self.last_command_pose.position.z = 0.4
        self.last_command_pose.orientation.w = 1.0
        self.last_joint_command: List[float] = []
        self.last_joint_state: List[float] = [0.0] * 7
        self.last_joint_velocity: List[float] = [0.0] * 7
        self.last_joint_effort: List[float] = [0.0] * 7
        self.stiffness_lock = threading.Lock()
        self.stiffness_config: Dict[str, float] = {
            "translational_stiffness_X": 300.0,
            "translational_stiffness_Y": 300.0,
            "translational_stiffness_Z": 300.0,
            "rotational_stiffness_X": 30.0,
            "rotational_stiffness_Y": 30.0,
            "rotational_stiffness_Z": 30.0,
            "nullspace_stiffness": 0.0,
        }

        self.joint_names = [f"{arm_id}_joint{i}" for i in range(1, 8)]
        self.gripper_joint_names = [f"{arm_id}_finger_joint1", f"{arm_id}_finger_joint2"]

        self._command_condition = threading.Condition()
        self._command_queue_limit = 64
        self._command_queue: Deque[Tuple[str, Callable[[], None]]] = deque()
        self._pending_stream: Optional[Tuple[str, int, Callable[[], None]]] = None
        self._running_stream: Optional[Tuple[str, int]] = None
        self._stream_generation = 0
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        _boot_log("bridge: before rospy.init_node")
        rospy.init_node("franky_cartesian_impedance_compat", anonymous=False, argv=ros_init_argv())
        _boot_log("bridge: after rospy.init_node")

        self.pub_cartesian_pose = rospy.Publisher("/cartesian_pose", PoseStamped, queue_size=10)
        self.pub_robot_pose = rospy.Publisher("/robot_pose", Pose, queue_size=10)
        self.pub_impedance_mode_status = rospy.Publisher("/impedance_mode_status", Bool, queue_size=1, latch=True)
        self.pub_current_impedance_mode = rospy.Publisher("/current_impedance_mode", String, queue_size=1, latch=True)
        self.pub_joint_states = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.pub_state_joint_states = rospy.Publisher("/franka_state_controller/joint_states", JointState, queue_size=10)
        self.pub_state_joint_states_desired = rospy.Publisher(
            "/franka_state_controller/joint_states_desired", JointState, queue_size=10
        )
        self.pub_franka_states = rospy.Publisher("/franka_state_controller/franka_states", FrankaState, queue_size=10)
        self.pub_external_wrench = rospy.Publisher("/franka_state_controller/F_ext", WrenchStamped, queue_size=10)
        self.pub_force_torque_ext = rospy.Publisher("/force_torque_ext", WrenchStamped, queue_size=10)
        self.pub_gripper_joint_states = rospy.Publisher("/franka_gripper/joint_states", JointState, queue_size=10)

        self.sub_equilibrium_pose = rospy.Subscriber(
            "/equilibrium_pose", PoseStamped, self._on_equilibrium_pose, queue_size=10
        )
        self.sub_joint_command = rospy.Subscriber(
            "/joint_command", Float64MultiArray, self._on_joint_command, queue_size=10
        )
        self.sub_gripper = rospy.Subscriber(
            "/gripper_control", Float64MultiArray, self._on_gripper_control, queue_size=10
        )
        self.sub_impedance_mode = rospy.Subscriber(
            "/impedance_mode", Bool, self._on_impedance_mode, queue_size=10
        )
        self.sub_stiffness = rospy.Subscriber(
            "/stiffness", Float32MultiArray, self._on_stiffness, queue_size=10
        )
        self.sub_eq_config = rospy.Subscriber(
            "/equilibrium_configuration", Float32MultiArray, self._on_equilibrium_configuration, queue_size=10
        )

        _boot_log("bridge: before dynamic_reconfigure server")
        self.dynamic_server = DynamicReconfigureServer(
            compliance_paramConfig,
            self._on_dynamic_reconfigure,
            namespace="/dynamic_reconfigure_compliance_param_node",
        )
        _boot_log("bridge: after dynamic_reconfigure server")
        if connect_on_start:
            self._connector = threading.Thread(target=self._run_connector, daemon=True)
            self._connector.start()
        self._publish_mode_status()

    def run(self) -> None:
        rospy.loginfo("franky ROS compatibility bridge started")
        rospy.loginfo(
            "subscribed topics: /equilibrium_pose /joint_command /gripper_control "
            "/impedance_mode /stiffness /equilibrium_configuration"
        )
        rospy.loginfo(
            "published topics: /cartesian_pose /robot_pose /joint_states "
            "/franka_state_controller/joint_states /franka_state_controller/franka_states"
        )

        rate = rospy.Rate(self.pose_rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            pose = self._estimate_pose()
            if pose is not None:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = now
                pose_stamped.pose = pose
                self.pub_cartesian_pose.publish(pose_stamped)
                self.pub_robot_pose.publish(pose)
            self._publish_joint_states(now)
            self._publish_franka_state(now, pose)
            self._publish_external_wrench(now)
            self.sequence_number += 1
            rate.sleep()

    def _run_worker(self) -> None:
        while True:
            action = None
            with self._command_condition:
                while action is None:
                    if self._running_stream is not None:
                        running_name, running_generation = self._running_stream
                        if self._pending_stream is not None and self._pending_stream[1] != running_generation:
                            action = ("stop_stream", running_name, running_generation)
                        else:
                            action = ("poll_stream", running_name, running_generation)
                    elif self._command_queue:
                        name, fn = self._command_queue.popleft()
                        action = ("sync", name, fn)
                    elif self._pending_stream is not None:
                        name, generation, fn = self._pending_stream
                        self._pending_stream = None
                        self._running_stream = (name, generation)
                        action = ("start_stream", name, generation, fn)
                    else:
                        self._command_condition.wait()

            kind = action[0]
            try:
                if kind == "sync":
                    _, name, fn = action
                    fn()
                elif kind == "start_stream":
                    _, name, generation, fn = action
                    fn()
                elif kind == "poll_stream":
                    _, name, generation = action
                    if self.ctrl.motion_active():
                        time.sleep(0.05)
                        continue
                    with self._command_condition:
                        if self._running_stream == (name, generation):
                            self._running_stream = None
                            self._command_condition.notify_all()
                elif kind == "stop_stream":
                    _, name, generation = action
                    self.ctrl.stop_motion()
                    with self._command_condition:
                        if self._running_stream == (name, generation):
                            self._running_stream = None
                            self._command_condition.notify_all()
            except Exception as exc:
                if rospy.is_shutdown():
                    return
                if kind == "sync":
                    name = action[1]
                else:
                    name = action[1]
                    with self._command_condition:
                        if kind in {"start_stream", "poll_stream", "stop_stream"} and self._running_stream == (
                            action[1],
                            action[2],
                        ):
                            self._running_stream = None
                            self._command_condition.notify_all()
                rospy.logerr("command %s failed: %s", name, exc)

    def _enqueue(self, name: str, fn: Callable[[], None]) -> None:
        with self._command_condition:
            if len(self._command_queue) >= self._command_queue_limit:
                dropped_name, _ = self._command_queue.popleft()
                rospy.logwarn_throttle(
                    2.0,
                    "compat command queue full, dropping oldest command %s to keep bridge responsive",
                    dropped_name,
                )
            self._command_queue.append((name, fn))
            self._command_condition.notify_all()

    def _enqueue_stream(self, name: str, fn: Callable[[], None]) -> None:
        # Match the original controller semantics more closely: new target poses
        # replace older ones instead of building an ever-growing motion backlog.
        with self._command_condition:
            self._stream_generation += 1
            self._pending_stream = (name, self._stream_generation, fn)
            self._command_condition.notify_all()

    def _run_connector(self) -> None:
        last_error = None
        while not rospy.is_shutdown():
            if self.ctrl.is_connected():
                time.sleep(1.0)
                continue
            if self.ctrl.connect():
                rospy.loginfo("connected to Franka robot at %s", self.ctrl.robot_ip)
                self._apply_stiffness()
                last_error = None
                continue
            if self.ctrl.last_error != last_error:
                rospy.logwarn("waiting for Franka robot at %s: %s", self.ctrl.robot_ip, self.ctrl.last_error)
                last_error = self.ctrl.last_error
            time.sleep(3.0)

    def _on_equilibrium_pose(self, msg: PoseStamped) -> None:
        caller_id = self._caller_id(msg)
        if self._is_interactive_marker_source(caller_id):
            if time.time() < self._external_equilibrium_pose_until:
                rospy.loginfo_throttle(
                    5.0,
                    "ignoring /interactive_marker equilibrium_pose while external command source is active",
                )
                return
        else:
            self._external_equilibrium_pose_until = time.time() + self.external_command_hold_sec

        self.last_command_pose = copy_pose(msg.pose)
        pose = copy_pose(msg.pose)
        self._enqueue_stream(
            "equilibrium_pose",
            lambda: self.ctrl.move_abs(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                asynchronous=True,
            ),
        )

    def _caller_id(self, msg) -> str:
        header = getattr(msg, "_connection_header", None) or {}
        try:
            return str(header.get("callerid", ""))
        except Exception:
            return ""

    def _is_interactive_marker_source(self, caller_id: str) -> bool:
        return caller_id == "/interactive_marker" or caller_id.endswith("/interactive_marker")

    def _on_joint_command(self, msg: Float64MultiArray) -> None:
        joints = list(msg.data)
        if len(joints) != 7:
            rospy.logwarn("/joint_command expects 7 values, got %d", len(joints))
            return
        self.last_joint_command = joints
        self._enqueue_stream("joint_command", lambda: self.ctrl.move_joint(joints, asynchronous=True))

    def _normalize_gripper_width(self, raw: float) -> float:
        if 0.0 <= raw <= 1.0:
            width = raw * 0.08
        else:
            width = raw
        return clamp(width, 0.0, 0.08)

    def _on_gripper_control(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        if not self.ctrl.load_gripper:
            rospy.logwarn("/gripper_control received but gripper is disabled")
            return

        data = list(msg.data)
        width = self._normalize_gripper_width(float(data[0]))
        speed = float(data[1]) if len(data) >= 2 else 0.05
        force = float(data[2]) if len(data) >= 3 else 0.0

        if force > 0.0:
            self._enqueue("gripper_grasp", lambda: self.ctrl.gripper_grasp(width, speed, force))
        else:
            self._enqueue("gripper_move", lambda: self.ctrl.gripper_move(width, speed))

    def _publish_mode_status(self) -> None:
        self.pub_impedance_mode_status.publish(Bool(data=self.impedance_mode))
        mode_text = "cartesian" if self.impedance_mode else "joint"
        self.pub_current_impedance_mode.publish(String(data=mode_text))

    def _on_impedance_mode(self, msg: Bool) -> None:
        self.impedance_mode = bool(msg.data)
        self._publish_mode_status()
        rospy.loginfo("/impedance_mode set to %s", self.impedance_mode)

    def _on_stiffness(self, msg: Float32MultiArray) -> None:
        values = list(msg.data)
        if len(values) != 7:
            rospy.logwarn("/stiffness expects 7 values, got %d", len(values))
            return
        self.dynamic_server.update_configuration(
            {
                "translational_stiffness_X": clamp(float(values[0]), 0.0, 4000.0),
                "translational_stiffness_Y": clamp(float(values[1]), 0.0, 4000.0),
                "translational_stiffness_Z": clamp(float(values[2]), 0.0, 4000.0),
                "rotational_stiffness_X": clamp(float(values[3]), 0.0, 400.0),
                "rotational_stiffness_Y": clamp(float(values[4]), 0.0, 400.0),
                "rotational_stiffness_Z": clamp(float(values[5]), 0.0, 400.0),
                "nullspace_stiffness": clamp(float(values[6]), 0.0, 100.0),
            }
        )

    def _on_equilibrium_configuration(self, msg: Float32MultiArray) -> None:
        values = list(msg.data)
        if len(values) != 7:
            rospy.logwarn("/equilibrium_configuration expects 7 values, got %d", len(values))
            return
        self.last_joint_command = values
        self._enqueue_stream(
            "equilibrium_configuration",
            lambda: self.ctrl.move_joint(values, asynchronous=True),
        )

    def _on_dynamic_reconfigure(self, config, _level):
        with self.stiffness_lock:
            self.stiffness_config = {
                "translational_stiffness_X": float(config["translational_stiffness_X"]),
                "translational_stiffness_Y": float(config["translational_stiffness_Y"]),
                "translational_stiffness_Z": float(config["translational_stiffness_Z"]),
                "rotational_stiffness_X": float(config["rotational_stiffness_X"]),
                "rotational_stiffness_Y": float(config["rotational_stiffness_Y"]),
                "rotational_stiffness_Z": float(config["rotational_stiffness_Z"]),
                "nullspace_stiffness": float(config["nullspace_stiffness"]),
            }
        self._apply_stiffness()
        return config

    def _apply_stiffness(self) -> None:
        if not self.ctrl.is_connected():
            return
        with self.stiffness_lock:
            config = dict(self.stiffness_config)
        try:
            self.ctrl.set_cartesian_stiffness(
                [
                    config["translational_stiffness_X"],
                    config["translational_stiffness_Y"],
                    config["translational_stiffness_Z"],
                    config["rotational_stiffness_X"],
                    config["rotational_stiffness_Y"],
                    config["rotational_stiffness_Z"],
                ]
            )
            self.ctrl.set_joint_stiffness([config["nullspace_stiffness"]] * 7)
        except Exception as exc:
            if rospy.is_shutdown():
                return
            rospy.logwarn_throttle(5.0, "failed to apply stiffness to franky runtime: %s", exc)

    def _estimate_pose(self) -> Optional[Pose]:
        pose = self._pose_from_robot_state()
        if pose is not None:
            self.last_command_pose = copy_pose(pose)
            return pose
        return copy_pose(self.last_command_pose)

    def _pose_from_robot_state(self) -> Optional[Pose]:
        if self.ctrl.robot is None:
            return None
        try:
            cart_state = self.ctrl.robot.current_cartesian_state
            robot_pose = getattr(cart_state, "pose", None)
            if robot_pose is None:
                return None
            ee_pose = getattr(robot_pose, "end_effector_pose", robot_pose)
            return self._pose_from_affine(ee_pose)
        except Exception:
            return None

    def _pose_from_affine(self, affine_obj) -> Optional[Pose]:
        pose = Pose()
        translation = self._extract_translation(affine_obj)
        if translation is None:
            return None
        pose.position.x, pose.position.y, pose.position.z = translation

        quat = self._extract_quaternion(affine_obj)
        if quat is None:
            pose.orientation.w = 1.0
        else:
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        return pose

    def _extract_translation(self, affine_obj) -> Optional[Sequence[float]]:
        for attr in ("translation", "position"):
            value = getattr(affine_obj, attr, None)
            if value is None:
                continue
            if callable(value):
                value = value()
            seq = self._to_float_seq(value)
            if seq and len(seq) >= 3:
                return seq[:3]
        try:
            import numpy as np

            matrix = np.asarray(affine_obj, dtype=float)
            if matrix.shape == (4, 4):
                return [float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3])]
        except Exception:
            pass
        return None

    def _extract_quaternion(self, affine_obj) -> Optional[Sequence[float]]:
        for attr in ("quaternion", "quat"):
            value = getattr(affine_obj, attr, None)
            if value is None:
                continue
            if callable(value):
                value = value()
            seq = self._to_float_seq(value)
            if seq and len(seq) >= 4:
                return seq[:4]
        try:
            import numpy as np
            from tf.transformations import quaternion_from_matrix

            matrix = np.asarray(affine_obj, dtype=float)
            if matrix.shape == (4, 4):
                quat = quaternion_from_matrix(matrix)
                return [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
        except Exception:
            pass
        return None

    def _to_float_seq(self, value) -> Optional[List[float]]:
        if value is None:
            return None
        try:
            if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
                if hasattr(value, "w"):
                    return [float(value.x), float(value.y), float(value.z), float(value.w)]
                return [float(value.x), float(value.y), float(value.z)]
            return [float(v) for v in value]
        except Exception:
            return None

    def _estimate_joint_vectors(self) -> Tuple[List[float], List[float], List[float]]:
        positions = self.ctrl.get_joint_positions()
        velocities = self.ctrl.get_joint_velocities()
        efforts = self.ctrl.get_joint_efforts()

        if positions and len(positions) == 7:
            self.last_joint_state = positions
        positions = list(self.last_joint_state)

        if velocities and len(velocities) == 7:
            self.last_joint_velocity = velocities
        velocities = list(self.last_joint_velocity)

        if efforts and len(efforts) == 7:
            self.last_joint_effort = efforts
        efforts = list(self.last_joint_effort)
        return positions, velocities, efforts

    def _publish_joint_states(self, now) -> None:
        positions, velocities, efforts = self._estimate_joint_vectors()
        desired = list(self.last_joint_command) if len(self.last_joint_command) == 7 else list(positions)

        arm_msg = JointState()
        arm_msg.header.stamp = now
        arm_msg.header.seq = self.sequence_number
        arm_msg.name = list(self.joint_names)
        arm_msg.position = list(positions)
        arm_msg.velocity = list(velocities)
        arm_msg.effort = list(efforts)
        self.pub_state_joint_states.publish(arm_msg)

        desired_msg = JointState()
        desired_msg.header.stamp = now
        desired_msg.header.seq = self.sequence_number
        desired_msg.name = list(self.joint_names)
        desired_msg.position = desired
        desired_msg.velocity = [0.0] * 7
        desired_msg.effort = [0.0] * 7
        self.pub_state_joint_states_desired.publish(desired_msg)

        aggregate_msg = JointState()
        aggregate_msg.header.stamp = now
        aggregate_msg.header.seq = self.sequence_number
        aggregate_msg.name = list(self.joint_names)
        aggregate_msg.position = list(positions)
        aggregate_msg.velocity = list(velocities)
        aggregate_msg.effort = list(efforts)

        if self.ctrl.load_gripper:
            width = clamp(self.ctrl.get_gripper_width(), 0.0, 0.08)
            finger_position = width * 0.5

            gripper_msg = JointState()
            gripper_msg.header.stamp = now
            gripper_msg.header.seq = self.sequence_number
            gripper_msg.name = list(self.gripper_joint_names)
            gripper_msg.position = [finger_position, finger_position]
            gripper_msg.velocity = [0.0, 0.0]
            gripper_msg.effort = [0.0, 0.0]
            self.pub_gripper_joint_states.publish(gripper_msg)

            aggregate_msg.name.extend(self.gripper_joint_names)
            aggregate_msg.position.extend(gripper_msg.position)
            aggregate_msg.velocity.extend(gripper_msg.velocity)
            aggregate_msg.effort.extend(gripper_msg.effort)

        self.pub_joint_states.publish(aggregate_msg)

    def _publish_external_wrench(self, now) -> None:
        wrench_values = self.ctrl.get_external_wrench()

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = now
        wrench_msg.header.frame_id = f"{self.arm_id}_K"
        wrench_msg.wrench.force.x = wrench_values[0]
        wrench_msg.wrench.force.y = wrench_values[1]
        wrench_msg.wrench.force.z = wrench_values[2]
        wrench_msg.wrench.torque.x = wrench_values[3]
        wrench_msg.wrench.torque.y = wrench_values[4]
        wrench_msg.wrench.torque.z = wrench_values[5]
        self.pub_external_wrench.publish(wrench_msg)
        self.pub_force_torque_ext.publish(wrench_msg)

    def _publish_franka_state(self, now, pose: Optional[Pose]) -> None:
        positions, velocities, efforts = self._estimate_joint_vectors()
        desired = list(self.last_joint_command) if len(self.last_joint_command) == 7 else list(positions)
        pose_matrix = self._pose_to_matrix_list(pose)
        desired_pose = copy_pose(self.last_command_pose) or copy_pose(pose)
        desired_pose_matrix = self._pose_to_matrix_list(desired_pose)
        wrench_values = self.ctrl.get_external_wrench()

        msg = FrankaState()
        msg.header.stamp = now
        msg.header.seq = self.sequence_number
        msg.q = list(positions)
        msg.q_d = list(desired)
        msg.dq = list(velocities)
        msg.dq_d = [0.0] * 7
        msg.ddq_d = [0.0] * 7
        msg.theta = list(positions)
        msg.dtheta = list(velocities)
        msg.tau_J = list(efforts)
        msg.dtau_J = [0.0] * 7
        msg.tau_J_d = [0.0] * 7
        msg.tau_ext_hat_filtered = [0.0] * 7
        msg.cartesian_collision = [0.0] * 6
        msg.cartesian_contact = [0.0] * 6
        msg.joint_collision = [0.0] * 7
        msg.joint_contact = [0.0] * 7
        msg.K_F_ext_hat_K = list(wrench_values)
        msg.O_F_ext_hat_K = list(wrench_values)
        msg.O_dP_EE_d = [0.0] * 6
        msg.O_dP_EE_c = [0.0] * 6
        msg.O_ddP_EE_c = [0.0] * 6
        msg.O_ddP_O = [0.0, 0.0, -9.81]
        msg.elbow = [0.0] * 2
        msg.elbow_d = [0.0] * 2
        msg.elbow_c = [0.0] * 2
        msg.delbow_c = [0.0] * 2
        msg.ddelbow_c = [0.0] * 2
        msg.F_x_Cee = [0.0] * 3
        msg.F_x_Cload = [0.0] * 3
        msg.F_x_Ctotal = [0.0] * 3
        msg.I_ee = [0.0] * 9
        msg.I_load = [0.0] * 9
        msg.I_total = [0.0] * 9
        msg.O_T_EE = pose_matrix
        msg.O_T_EE_d = desired_pose_matrix
        msg.O_T_EE_c = desired_pose_matrix
        msg.F_T_EE = self._identity_matrix_list()
        msg.F_T_NE = self._identity_matrix_list()
        msg.NE_T_EE = self._identity_matrix_list()
        msg.EE_T_K = self._identity_matrix_list()
        msg.m_ee = 0.0
        msg.m_load = 0.0
        msg.m_total = 0.0
        msg.time = now.to_sec()
        msg.control_command_success_rate = 1.0
        msg.robot_mode = self._map_robot_mode()
        self.pub_franka_states.publish(msg)

    def _pose_to_matrix_list(self, pose: Optional[Pose]) -> List[float]:
        if pose is None:
            return self._identity_matrix_list()
        matrix = quaternion_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        matrix[0][3] = pose.position.x
        matrix[1][3] = pose.position.y
        matrix[2][3] = pose.position.z
        result = [0.0] * 16
        for row in range(4):
            for col in range(4):
                result[col * 4 + row] = float(matrix[row][col])
        return result

    def _identity_matrix_list(self) -> List[float]:
        return [
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]

    def _map_robot_mode(self) -> int:
        state = self.ctrl.get_robot_state()
        mode = getattr(state, "robot_mode", None)
        if mode is None:
            return FrankaState.ROBOT_MODE_IDLE
        mode_text = str(mode).lower()
        if "move" in mode_text:
            return FrankaState.ROBOT_MODE_MOVE
        if "guid" in mode_text:
            return FrankaState.ROBOT_MODE_GUIDING
        if "reflex" in mode_text:
            return FrankaState.ROBOT_MODE_REFLEX
        if "user" in mode_text or "stop" in mode_text:
            return FrankaState.ROBOT_MODE_USER_STOPPED
        if "recover" in mode_text:
            return FrankaState.ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY
        if "idle" in mode_text:
            return FrankaState.ROBOT_MODE_IDLE
        return FrankaState.ROBOT_MODE_OTHER


HELP_TEXT = """
Commands:
  help
  pose
  move_rel DX DY DZ
  move_abs X Y Z [QX QY QZ QW]
  dynamics FACTOR
  open [SPEED]
  close WIDTH [SPEED]
  grasp WIDTH [SPEED] [FORCE]
  home
  quit | exit
""".strip()


def repl(ctrl: FrankyCartesianController) -> None:
    print("franky cartesian control ready. Type 'help' for commands.")
    while True:
        try:
            line = input("franky> ").strip()
        except EOFError:
            print()
            break

        if not line:
            continue

        parts = shlex.split(line)
        cmd = parts[0].lower()

        try:
            if cmd in {"quit", "exit"}:
                break
            if cmd == "help":
                print(HELP_TEXT)
            elif cmd == "pose":
                ctrl.print_pose()
            elif cmd == "move_rel":
                if len(parts) != 4:
                    raise ValueError("usage: move_rel DX DY DZ")
                ctrl.move_rel(float(parts[1]), float(parts[2]), float(parts[3]))
            elif cmd == "move_abs":
                if len(parts) not in {4, 8}:
                    raise ValueError("usage: move_abs X Y Z [QX QY QZ QW]")
                quat = None
                if len(parts) == 8:
                    quat = [float(v) for v in parts[4:8]]
                ctrl.move_abs(float(parts[1]), float(parts[2]), float(parts[3]), quat)
            elif cmd == "dynamics":
                if len(parts) != 2:
                    raise ValueError("usage: dynamics FACTOR")
                ctrl.set_dynamics(float(parts[1]))
            elif cmd == "open":
                speed = float(parts[1]) if len(parts) == 2 else 0.05
                ctrl.gripper_open(speed)
            elif cmd == "close":
                if len(parts) not in {2, 3}:
                    raise ValueError("usage: close WIDTH [SPEED]")
                speed = float(parts[2]) if len(parts) == 3 else 0.05
                ctrl.gripper_move(float(parts[1]), speed)
            elif cmd == "grasp":
                if len(parts) < 2 or len(parts) > 4:
                    raise ValueError("usage: grasp WIDTH [SPEED] [FORCE]")
                width = float(parts[1])
                speed = float(parts[2]) if len(parts) >= 3 else 0.05
                force = float(parts[3]) if len(parts) >= 4 else 20.0
                ctrl.gripper_grasp(width, speed, force)
            elif cmd == "home":
                ctrl.gripper_home()
            else:
                print(f"unknown command: {cmd}")
        except Exception as exc:
            print(f"error: {exc}")


def _runtime_state(ctrl: FrankyCartesianController) -> Dict[str, Any]:
    return {
        "positions": ctrl.get_joint_positions() or [0.0] * 7,
        "velocities": ctrl.get_joint_velocities() or [0.0] * 7,
        "efforts": ctrl.get_joint_efforts() or [0.0] * 7,
        "wrench": ctrl.get_external_wrench(),
        "gripper_width": ctrl.get_gripper_width(),
    }


def runtime_loop(ctrl: FrankyCartesianController) -> None:
    for line in sys.stdin:
        try:
            request = json.loads(line)
            command = request.get("command")
            payload = request.get("payload") or {}
            if command == "shutdown":
                break
            if command == "connect":
                if not ctrl.connect():
                    raise RuntimeError(ctrl.last_error or "connect failed")
                result = True
            elif command == "move_rel":
                ctrl.move_rel(float(payload["dx"]), float(payload["dy"]), float(payload["dz"]))
                result = True
            elif command == "move_abs":
                ctrl.move_abs(
                    float(payload["x"]),
                    float(payload["y"]),
                    float(payload["z"]),
                    payload.get("quat"),
                    asynchronous=bool(payload.get("asynchronous", False)),
                )
                result = True
            elif command == "move_joint":
                ctrl.move_joint(payload["joints"], asynchronous=bool(payload.get("asynchronous", False)))
                result = True
            elif command == "motion_active":
                result = ctrl.motion_active()
            elif command == "stop_motion":
                ctrl.stop_motion()
                result = True
            elif command == "set_dynamics":
                ctrl.set_dynamics(float(payload["factor"]))
                result = True
            elif command == "set_cartesian_stiffness":
                ctrl.set_cartesian_stiffness(payload["stiffness"])
                result = True
            elif command == "set_joint_stiffness":
                ctrl.set_joint_stiffness(payload["stiffness"])
                result = True
            elif command == "gripper_open":
                ctrl.gripper_open(float(payload.get("speed", 0.05)))
                result = True
            elif command == "gripper_move":
                ctrl.gripper_move(float(payload["width"]), float(payload.get("speed", 0.05)))
                result = True
            elif command == "gripper_grasp":
                ctrl.gripper_grasp(
                    float(payload["width"]),
                    float(payload.get("speed", 0.05)),
                    float(payload.get("force", 20.0)),
                )
                result = True
            elif command == "gripper_home":
                ctrl.gripper_home()
                result = True
            elif command == "state":
                result = _runtime_state(ctrl)
            else:
                raise ValueError(f"unknown runtime command: {command}")
            response = {"ok": True, "result": result}
        except Exception as exc:
            response = {"ok": False, "error": str(exc)}
        sys.stdout.write(json.dumps(response) + "\n")
        sys.stdout.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="franky replacement for cartesian_impedance_example_controller.launch"
    )
    parser.add_argument("--robot-ip", default="172.16.0.2", help="Franka robot IP")
    parser.add_argument(
        "--load-gripper",
        type=str2bool,
        default=True,
        help="Load Franka gripper control (true/false)",
    )
    parser.add_argument(
        "--dynamics",
        type=float,
        default=0.05,
        help="relative dynamics factor in [0, 1]",
    )
    parser.add_argument(
        "--auto-recover",
        type=str2bool,
        default=True,
        help="recover robot from errors before entering REPL",
    )
    parser.add_argument(
        "--interface",
        choices=["ros", "cli", "runtime"],
        default="ros",
        help="control interface mode",
    )
    parser.add_argument(
        "--pose-rate-hz",
        type=float,
        default=20.0,
        help="publish rate for compatibility state output in ROS mode",
    )
    parser.add_argument("--arm-id", default="panda", help="Franka arm id for ROS topics and joint names")
    parser.add_argument(
        "--connect-on-start",
        type=str2bool,
        default=False,
        help="Try to connect to the robot immediately instead of waiting for the first command",
    )
    parser.add_argument(
        "--external-command-hold-sec",
        type=float,
        default=10.0,
        help="Seconds to ignore interactive marker pose refreshes after an external /equilibrium_pose command",
    )
    args, unknown = parser.parse_known_args()
    ros_args = [arg for arg in unknown if ":=" in arg]
    other_unknown = [arg for arg in unknown if ":=" not in arg]
    if other_unknown:
        parser.error(f"unrecognized arguments: {' '.join(other_unknown)}")
    if ros_args:
        _boot_log(f"parse_args: ignoring ROS remap arguments: {' '.join(ros_args)}")
    return args


def main() -> int:
    _boot_log("main: enter")
    args = parse_args()
    _boot_log(f"main: parsed args interface={args.interface} connect_on_start={args.connect_on_start}")
    try:
        ctrl = FrankyCartesianController(
            robot_ip=args.robot_ip,
            load_gripper=args.load_gripper,
            dynamics=args.dynamics,
            auto_recover=args.auto_recover,
        )

        if args.interface == "cli":
            ctrl.connect()
            repl(ctrl)
        elif args.interface == "runtime":
            runtime_loop(ctrl)
        else:
            _boot_log("main: constructing ROS bridge")
            bridge = FrankyRosCompatBridge(
                ctrl,
                arm_id=args.arm_id,
                pose_rate_hz=args.pose_rate_hz,
                connect_on_start=args.connect_on_start,
                external_command_hold_sec=args.external_command_hold_sec,
            )
            _boot_log("main: running ROS bridge")
            bridge.run()
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"fatal: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
