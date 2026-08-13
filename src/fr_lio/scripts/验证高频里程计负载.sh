#!/usr/bin/env bash
set -Eeuo pipefail

# Dedicated entry point for prerequisite-chain CPU load validation.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

export VALIDATION_MODE=load_only
export BASELINE_DURATION_SEC="${BASELINE_DURATION_SEC:-60}"
export STATIC_DURATION_SEC="${STATIC_DURATION_SEC:-300}"
export CPU_LOAD_WORKERS="${CPU_LOAD_WORKERS:-2}"
export CPU_LOAD_DUTY_PERCENT="${CPU_LOAD_DUTY_PERCENT:-50}"
export RECORD_LIVOX_TOPICS="${RECORD_LIVOX_TOPICS:-false}"

exec "${script_dir}/验证高频里程计全链路.sh"
