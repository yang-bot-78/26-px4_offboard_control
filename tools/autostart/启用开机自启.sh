#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

autostart_dir="${HOME}/.config/autostart"
desktop_file="${autostart_dir}/ws_offboard_takeoff_stack.desktop"
user_systemd_dir="${HOME}/.config/systemd/user"
service_src="${project_root}/tools/rosbag/ws_offboard_rosbag_shutdown.service"
service_dst="${user_systemd_dir}/ws_offboard_rosbag_shutdown.service"
# .desktop 条目需要解析后的绝对路径；${project_root} 必须在这里展开，
# 不能原样写进文件。
entry_exec="Exec=/bin/bash -lc '\"${project_root}/tools/flight/开机自启动起飞栈.sh\"'"

"${project_root}/tools/checks/起飞前自检.sh"

mkdir -p "${autostart_dir}"
cat > "${desktop_file}" <<DESKTOP
[Desktop Entry]
Type=Application
Version=1.0
Name=WS Offboard Takeoff Stack
Comment=Autostart the complete MID360, FAST-LIO, MAVROS/EV, rosbag, and navigation chain after login
${entry_exec}
Terminal=false
X-GNOME-Autostart-enabled=true
StartupNotify=false
Categories=Utility;
DESKTOP
chmod 644 "${desktop_file}"

mkdir -p "${user_systemd_dir}"
cp "${service_src}" "${service_dst}"
chmod 644 "${service_dst}"
systemctl --user daemon-reload
systemctl --user enable --now ws_offboard_rosbag_shutdown.service

echo "开机自启已启用：${desktop_file}"
echo "rosbag 关机保护已启用：${service_dst}"
echo
echo "当前文件内容："
sed -n '1,200p' "${desktop_file}"
