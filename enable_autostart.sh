#!/usr/bin/env bash
set -euo pipefail

autostart_dir="${HOME}/.config/autostart"
desktop_file="${autostart_dir}/ws_offboard_takeoff_stack.desktop"
user_systemd_dir="${HOME}/.config/systemd/user"
service_src="${HOME}/ws_offboard_control/ws_offboard_rosbag_shutdown.service"
service_dst="${user_systemd_dir}/ws_offboard_rosbag_shutdown.service"
entry_exec='Exec=/bin/bash -lc '\''"$HOME/ws_offboard_control/start_takeoff_1m_stack_login.sh"'\'''

"${HOME}/ws_offboard_control/check_takeoff_autostart_ready.sh"

mkdir -p "${autostart_dir}"
cat > "${desktop_file}" <<DESKTOP
[Desktop Entry]
Type=Application
Version=1.0
Name=WS Offboard Takeoff Stack
Comment=Autostart MID360, FAST-LIO, MAVROS, and takeoff terminals after login
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

echo "Autostart enabled: ${desktop_file}"
echo "Rosbag shutdown guard enabled: ${service_dst}"
echo
echo "Current file content:"
sed -n '1,200p' "${desktop_file}"
