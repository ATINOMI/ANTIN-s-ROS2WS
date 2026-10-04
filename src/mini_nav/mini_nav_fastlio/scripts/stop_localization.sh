#!/usr/bin/env bash
set -eo pipefail
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
scurm_ws=$(cd -- "$scurm_root/../../.." && pwd -P)
scurm_pid=$(cat "$scurm_ws/log_scurm/localization_runtime/launch.pid")
if [[ $(tr '\0' ' ' < "/proc/$scurm_pid/cmdline") != *'ros2 launch scurm_sim mini_nav_fastlio.launch.py'* ]]; then
  echo 'PID does not belong to the SCURM mini_nav launch'; exit 1
fi
kill -INT "$scurm_pid"
