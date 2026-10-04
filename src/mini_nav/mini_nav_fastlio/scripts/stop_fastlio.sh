#!/usr/bin/env bash
set -euo pipefail
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
scurm_ws=$(cd -- "$scurm_root/../../.." && pwd -P)
scurm_pid_file="$scurm_ws/log_scurm/fastlio_runtime/launch.pid"
if [[ ! -f "$scurm_pid_file" ]]; then echo 'FAST-LIO2 demo is not running'; exit 0; fi
read -r scurm_pid < "$scurm_pid_file"
[[ "$scurm_pid" =~ ^[0-9]+$ ]] || { echo 'Invalid launch PID'; exit 1; }
if [[ ! -r "/proc/$scurm_pid/cmdline" ]]; then echo 'FAST-LIO2 demo has stopped'; exit 0; fi
scurm_command=$(tr '\0' ' ' < "/proc/$scurm_pid/cmdline")
[[ "$scurm_command" == *'ros2 launch scurm_sim fastlio_mapping.launch.py'* ]] || {
  echo 'PID is not the FAST-LIO2 demo; leaving it untouched'; exit 1;
}
kill -INT "$scurm_pid"
