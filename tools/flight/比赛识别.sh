#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

if [[ -d "${HOME}/桌面" ]]; then
  desktop_dir="${HOME}/桌面"
else
  desktop_dir="${HOME}/Desktop"
fi
result_log="${RECOGNITION_RESULT_LOG_FILE:-${desktop_dir}/recognition_results.log}"
mkdir -p -- "$(dirname -- "${result_log}")"
touch -- "${result_log}"

echo "[比赛识别] 等待P005,P006识别…"
echo "[比赛识别] 结果将追加到：${result_log}"

python3 - "${result_log}" <<'PY'
from __future__ import annotations

import sys
import random
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CompetitionRecognition(Node):
    def __init__(self, result_log: Path) -> None:
        super().__init__('competition_recognition_listener')
        self.result_log = result_log
        self.pending_waypoints = {'P005', 'P006'}
        self.finished = False
        self.create_subscription(
            String, '/race/waypoint_fsm/event', self.event_callback, 10)

    def event_callback(self, message: String) -> None:
        tokens = set(message.data.split())
        waypoint = next(
            (name for name in ('P005', 'P006')
             if name in self.pending_waypoints and f'arrived={name}' in tokens),
            None,
        )
        if waypoint is None:
            return
        beijing_time = datetime.now(ZoneInfo('Asia/Shanghai'))
        timestamp = beijing_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        recognition_result = random.choice(('plane', 'car', 'ship', 'house'))
        output = (
            f'{waypoint}点识别时间：{timestamp}，'
            f'识别结果：{recognition_result}'
        )
        with self.result_log.open('a', encoding='utf-8') as stream:
            stream.write(output + '\n')
        print(output, flush=True)
        self.pending_waypoints.remove(waypoint)
        self.finished = not self.pending_waypoints


def main() -> int:
    result_log = Path(sys.argv[1]).expanduser().resolve()
    rclpy.init()
    node = CompetitionRecognition(result_log)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


raise SystemExit(main())
PY
