#!/usr/bin/env bash

set -o pipefail

# Desktop sessions launched from Snap applications can leak GTK/GIO and Python
# paths into child processes. They can prevent the system PyQt5/ROS 2 process
# from starting, so build a predictable environment before sourcing ROS.
unset GTK_EXE_PREFIX
unset GTK_PATH
unset GIO_EXTRA_MODULES
unset GIO_MODULE_DIR
unset PYTHONHOME
unset PYTHONPATH
unset SNAP_LIBRARY_PATH
unset QT_PLUGIN_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH
unset QT_QPA_FONTDIR

log_dir="${HOME}/.ros"
log_file="${log_dir}/dobot_operator_gui_launcher.log"
agv_log_file="${log_dir}/seer_agv_integrated_driver.log"
workspace="${DOBOT_WS:-${HOME}/dobot_ws}"
mkdir -p "${log_dir}"

agv_pid=""
gui_pid=""
cleanup_started=0
export IP_address="${IP_address:-192.168.192.201}"
export DOBOT_FEEDBACK_PORT="${DOBOT_FEEDBACK_PORT:-30005}"
agv_host="${SEER_AGV_HOST:-192.168.192.5}"
agv_interface="${SEER_AGV_INTERFACE:-}"

cleanup() {
    if [ "${cleanup_started}" -ne 0 ]; then
        return
    fi
    cleanup_started=1
    trap - EXIT INT TERM
    if [ -n "${agv_pid}" ] && kill -0 "${agv_pid}" 2>/dev/null; then
        timeout 5 ros2 service call /seer_agv/stop std_srvs/srv/Trigger "{}" \
            >>"${agv_log_file}" 2>&1 || true
        kill -TERM -- "-${agv_pid}" 2>/dev/null || true
        wait "${agv_pid}" 2>/dev/null || true
    fi
}

on_signal() {
    if [ -n "${gui_pid}" ] && kill -0 "${gui_pid}" 2>/dev/null; then
        kill -TERM "${gui_pid}" 2>/dev/null || true
        wait "${gui_pid}" 2>/dev/null || true
        gui_pid=""
    fi
    cleanup
    exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

{
    printf '\n[%s] Starting unified CR5/SC3000/SEER operator GUI\n' "$(date '+%F %T')"
    source /opt/ros/humble/setup.bash
    source "${workspace}/install/setup.bash"

    if ! ros2 node list 2>/dev/null | grep -Fxq '/seer_agv_node'; then
        route_line="$(ip route get "${agv_host}" 2>/dev/null | head -n 1)"
        route_interface="$(awk '{for (i = 1; i < NF; i++) if ($i == "dev") {print $(i + 1); exit}}' <<<"${route_line}")"
        if [ -z "${agv_interface}" ]; then
            agv_interface="${route_interface}"
        fi
        carrier_file="/sys/class/net/${agv_interface}/carrier"
        if [ -r "${carrier_file}" ] \
            && [ "$(<"${carrier_file}")" = "1" ] \
            && [ -n "${agv_interface}" ] \
            && [[ " ${route_line} " == *" dev ${agv_interface} "* ]] \
            && nc -z -w 1 "${agv_host}" 19204 >/dev/null 2>&1; then
            setsid ros2 launch seer_agv_driver seer_agv.launch.py \
                host:="${agv_host}" enable_cmd_vel:=true \
                >>"${agv_log_file}" 2>&1 &
            agv_pid=$!
            printf 'Started SEER AGV driver pid=%s; log=%s\n' \
                "${agv_pid}" "${agv_log_file}"
        else
            printf 'SEER AGV %s:19204 unavailable (interface=%s, route=%s; carrier/route/port check failed); GUI will continue without AGV driver\n' \
                "${agv_host}" "${agv_interface:-<none>}" "${route_line:-<none>}"
        fi
    else
        printf 'Using existing /seer_agv_node; launcher will not terminate it\n'
    fi

    gui_executable="${workspace}/install/dobot_operator_gui/lib/dobot_operator_gui/dobot_operator_gui"
    "${gui_executable}" &
    gui_pid=$!
    wait "${gui_pid}"
    gui_status=$?
    gui_pid=""
    (exit "${gui_status}")
} >>"${log_file}" 2>&1

status=$?
if [ "${status}" -ne 0 ] && command -v zenity >/dev/null 2>&1; then
    zenity --error \
        --title='Dobot CR5 上位机启动失败' \
        --text="启动失败，错误已写入：\n${log_file}"
fi
cleanup
exit "${status}"
