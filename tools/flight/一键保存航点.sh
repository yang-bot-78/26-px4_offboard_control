#!/usr/bin/env bash
set -euo pipefail

# 交互式保存 map/ENU 航点。实际检查和安全写文件由 race_offboard 节点完成。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "用法："
  echo "  交互模式：$0"
  echo "  参数模式：$0 航点名称 xy|xyz 飞行高度 地图PCD路径 [true]"
  echo
  echo "示例："
  echo "  $0 door_01 xy 0.78 /绝对路径/地图.pcd"
  echo "  $0 door_01 xyz 0.78 /绝对路径/地图.pcd true"
  exit 0
fi

echo "========================================"
echo "        一键保存航点"
echo "========================================"
echo "航点来源：/race/odom（map/ENU）"
echo "保存前将检查定位健康、飞机静止和位置稳定性。"
echo

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "错误：未找到 ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${project_root}/install/setup.bash" ]]; then
  echo "错误：尚未找到工程安装环境：${project_root}/install/setup.bash" >&2
  echo "请先执行：colcon build --packages-select race_offboard" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"

waypoint_name="${1:-}"
mode="${2:-}"
flight_z="${3:-}"
map_file="${4:-${MAP_FILE:-}}"
force="${5:-false}"

if [[ -z "${waypoint_name}" ]]; then
  read -r -p "请输入航点名称（例如 door_01）：" waypoint_name
fi
if [[ -z "${waypoint_name}" || "${waypoint_name}" == *"/"* || "${waypoint_name}" == *"\\"* ]]; then
  echo "错误：航点名称不能为空，也不能包含路径分隔符。" >&2
  exit 2
fi

if [[ -z "${mode}" ]]; then
  read -r -p "请输入保存模式 [xy/xyz，默认 xy]：" mode
  mode="${mode:-xy}"
fi
if [[ "${mode}" != "xy" && "${mode}" != "xyz" ]]; then
  echo "错误：保存模式只能是 xy 或 xyz。" >&2
  exit 2
fi

if [[ -z "${flight_z}" ]]; then
  read -r -p "请输入默认飞行高度（米，默认 0.78）：" flight_z
  flight_z="${flight_z:-0.78}"
fi
if ! [[ "${flight_z}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "错误：飞行高度必须是非负数字。" >&2
  exit 2
fi

if [[ -z "${map_file}" ]]; then
  echo "请输入当前规划使用的 PCD 绝对路径。"
  echo "直接回车将不记录 PCD SHA256，读取航点时也无法校验地图身份。"
  read -r -p "PCD 路径：" map_file
fi
if [[ -n "${map_file}" && ! -f "${map_file}" ]]; then
  echo "错误：PCD 文件不存在：${map_file}" >&2
  exit 2
fi

if [[ "${force}" != "true" ]]; then
  read -r -p "如果同名航点已存在，是否允许覆盖？[y/N]：" overwrite
  if [[ "${overwrite}" == "y" || "${overwrite}" == "Y" ]]; then
    force="true"
  fi
fi

echo
echo "即将保存："
echo "  名称：${waypoint_name}"
echo "  模式：${mode}"
echo "  高度：${flight_z} 米"
echo "  地图：${map_file:-未指定（不记录 SHA256）}"
echo
echo "请确认飞机已基本静止，并且 /ev_health/flight_ready=true。"
read -r -p "按回车开始采样，输入 q 取消：" confirmation
if [[ "${confirmation}" == "q" || "${confirmation}" == "Q" ]]; then
  echo "已取消。"
  exit 0
fi

command=(ros2 run race_offboard save_waypoint.py
  --name "${waypoint_name}"
  --mode "${mode}"
  --flight-z "${flight_z}")
if [[ -n "${map_file}" ]]; then
  command+=(--map-file "${map_file}")
fi
if [[ "${force}" == "true" ]]; then
  command+=(--force)
fi

echo "正在采集约 1 秒位置数据，请保持飞机静止……"
"${command[@]}"
echo "航点保存完成。"
