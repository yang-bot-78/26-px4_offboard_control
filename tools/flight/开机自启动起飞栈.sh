#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

mkdir -p "${project_root}/runtime"
{
  echo "开机自启动包装脚本已启动：$(date '+%F %T %z')"
  echo "DISPLAY=${DISPLAY:-}"
  echo "XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-}"
} >>"${project_root}/runtime/autostart_login.log"

sleep "${AUTOSTART_DELAY_SEC:-10}"
# 依赖就绪检查和组件关闭由整栈入口负责。保留这个登录包装脚本是为了兼容已有的
# 桌面自启动文件，但它现在走统一入口，不再走旧版多终端编排脚本。
exec "${project_root}/tools/flight/一键启动导航栈.sh" >>"${project_root}/runtime/autostart_login.log" 2>&1
