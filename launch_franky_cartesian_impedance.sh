#!/usr/bin/env bash
# If invoked via sh, re-exec with bash to keep consistent behavior.
if [ -z "${BASH_VERSION:-}" ]; then
  exec bash "$0" "$@"
fi
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_SCRIPT="${SCRIPT_DIR}/franka_example_controllers/scripts/franky_cartesian_impedance_compat.py"

ROBOT_IP="172.16.0.2"
LOAD_GRIPPER="true"
DYNAMICS="0.05"
AUTO_RECOVER="true"
INTERFACE="ros"
POSE_RATE_HZ="20.0"
ARM_ID="panda"
CONNECT_ON_START="false"

for arg in "$@"; do
  case "$arg" in
    robot_ip:=*)
      ROBOT_IP="${arg#robot_ip:=}"
      ;;
    load_gripper:=*)
      LOAD_GRIPPER="${arg#load_gripper:=}"
      ;;
    dynamics:=*)
      DYNAMICS="${arg#dynamics:=}"
      ;;
    auto_recover:=*)
      AUTO_RECOVER="${arg#auto_recover:=}"
      ;;
    interface:=*)
      INTERFACE="${arg#interface:=}"
      ;;
    pose_rate_hz:=*)
      POSE_RATE_HZ="${arg#pose_rate_hz:=}"
      ;;
    arm_id:=*)
      ARM_ID="${arg#arm_id:=}"
      ;;
    connect_on_start:=*)
      CONNECT_ON_START="${arg#connect_on_start:=}"
      ;;
    *)
      echo "[WARN] ignored unknown arg: ${arg}" >&2
      ;;
  esac
done

# Choose a Python interpreter that can import franky.
PYTHON_BIN="${FRANKY_PYTHON:-}"
if [ -n "$PYTHON_BIN" ]; then
  if ! "$PYTHON_BIN" -c "import franky" >/dev/null 2>&1; then
    echo "[ERROR] FRANKY_PYTHON=${PYTHON_BIN} cannot import franky" >&2
    exit 1
  fi
else
  CANDIDATES=("python3" "$HOME/anaconda3/bin/python3" "python")
  for cand in "${CANDIDATES[@]}"; do
    if command -v "$cand" >/dev/null 2>&1; then
      if "$cand" -c "import franky" >/dev/null 2>&1; then
        PYTHON_BIN="$cand"
        break
      fi
    fi
  done
fi

if [ -z "$PYTHON_BIN" ]; then
  echo "[ERROR] No Python interpreter with franky module found." >&2
  echo "[HINT] Install with: python3 -m pip install --user franky-control" >&2
  echo "[HINT] Or set FRANKY_PYTHON=/path/to/python" >&2
  exit 1
fi

echo "[franky] robot_ip=${ROBOT_IP}, load_gripper=${LOAD_GRIPPER}, dynamics=${DYNAMICS}, auto_recover=${AUTO_RECOVER}, interface=${INTERFACE}, pose_rate_hz=${POSE_RATE_HZ}, arm_id=${ARM_ID}, connect_on_start=${CONNECT_ON_START}"
echo "[franky] python=${PYTHON_BIN}"

exec "$PYTHON_BIN" "${PY_SCRIPT}" \
  --robot-ip "${ROBOT_IP}" \
  --load-gripper "${LOAD_GRIPPER}" \
  --dynamics "${DYNAMICS}" \
  --auto-recover "${AUTO_RECOVER}" \
  --interface "${INTERFACE}" \
  --pose-rate-hz "${POSE_RATE_HZ}" \
  --arm-id "${ARM_ID}" \
  --connect-on-start "${CONNECT_ON_START}"
