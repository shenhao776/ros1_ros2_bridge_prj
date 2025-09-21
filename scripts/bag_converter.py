#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import time
import argparse
import os
import signal
import sys
from typing import List, Dict, Optional

# ==============================================================================
# 用户配置区域
# ==============================================================================
# 定义基础命令，方便复用和修改
NOETIC_SETUP = "source /opt/ros/noetic/setup.bash"
GALACTIC_SETUP = "source /opt/ros/galactic/setup.bash"
LIVOX2_ROS1_WS_SETUP = (
    "source /root/shared_files/ros1_ros2_bridge_prj/ws_livox2_ros1/devel/setup.bash"
)
BRIDGE_WS_SETUP = (
    "source /root/shared_files/ros1_ros2_bridge_prj/bridge_ws/install/setup.bash"
)
ROS_MASTER_URI_EXPORT = "export ROS_MASTER_URI=http://localhost:11311"

# 根据别名组合完整的环境设置命令
ROS1_ENV_CMD = f"{NOETIC_SETUP} && {LIVOX2_ROS1_WS_SETUP}"
ROS2_ENV_CMD = f"{GALACTIC_SETUP} && {BRIDGE_WS_SETUP}"
BRIDGE_ENV_CMD = f"{NOETIC_SETUP} && {GALACTIC_SETUP} && {LIVOX2_ROS1_WS_SETUP} && {BRIDGE_WS_SETUP} && {ROS_MASTER_URI_EXPORT}"

ENV_SETUP = {
    "roscore": ROS1_ENV_CMD,
    "bridge": BRIDGE_ENV_CMD,
    "ros1_record": BRIDGE_ENV_CMD,
    "ros2_play": ROS2_ENV_CMD,
}

# 动态桥接的启动文件路径
BRIDGE_LAUNCH_FILE = os.path.expanduser("~/dynamic_bridge_launch.py")
# ==============================================================================


def print_info(message: str):
    """打印蓝色信息文本"""
    print(f"\033[94m[INFO] {message}\033[0m")


def print_error(message: str):
    """打印红色错误文本"""
    print(f"\033[91m[ERROR] {message}\033[0m", file=sys.stderr)


def print_success(message: str):
    """打印绿色成功文本"""
    print(f"\033[92m[SUCCESS] {message}\033[0m")


def build_command(env_key: str, command: str) -> str:
    """构建带有环境设置的完整 shell 命令"""
    env_cmd = ENV_SETUP.get(env_key)
    if env_cmd:
        # 如果定义了环境命令，则使用 '&&' 连接
        return f"{env_cmd} && {command}"
    return command


def run_process_manager(ros2_bag_path: str, ros1_bag_name: str):
    """
    主函数，用于启动、管理和关闭所有必要的 ROS 进程。
    """
    processes: List[subprocess.Popen] = []
    player_process: Optional[subprocess.Popen] = None

    # 使用 preexec_fn=os.setsid 创建新的进程组
    # 这使得我们可以通过发送信号到整个组来可靠地终止子进程及其后代
    def preexec_fn():
        os.setsid()

    try:
        # 1. 启动 roscore
        print_info("启动 roscore...")
        roscore_cmd = build_command("roscore", "roscore")
        p_roscore = subprocess.Popen(
            roscore_cmd,
            shell=True,
            preexec_fn=preexec_fn,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            executable="/bin/bash",
        )
        processes.append(p_roscore)
        time.sleep(2)  # 等待 roscore 完全启动
        if p_roscore.poll() is not None:
            raise RuntimeError(
                f"roscore 启动失败! 错误: {p_roscore.stderr.read().decode()}"
            )
        print_info("roscore 已启动 (PID: {})".format(p_roscore.pid))

        # 2. 启动 ros1_ros2_bridge
        print_info("启动 dynamic_bridge...")
        bridge_cmd = build_command("bridge", f"ros2 launch {BRIDGE_LAUNCH_FILE}")
        p_bridge = subprocess.Popen(
            bridge_cmd,
            shell=True,
            preexec_fn=preexec_fn,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            executable="/bin/bash",
        )
        processes.append(p_bridge)
        time.sleep(3)  # 等待 bridge 启动并建立连接
        if p_bridge.poll() is not None:
            raise RuntimeError(
                f"Bridge 启动失败! 错误: {p_bridge.stderr.read().decode()}"
            )
        print_info("dynamic_bridge 已启动 (PID: {})".format(p_bridge.pid))

        # 3. 启动 rosbag record
        print_info(f"开始录制 ROS 1 topics 到 '{ros1_bag_name}'...")
        record_cmd = build_command(
            "ros1_record", f"rosbag record -a -O {ros1_bag_name}"
        )
        p_recorder = subprocess.Popen(
            record_cmd,
            shell=True,
            preexec_fn=preexec_fn,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            executable="/bin/bash",
        )
        processes.append(p_recorder)
        time.sleep(1)
        if p_recorder.poll() is not None:
            raise RuntimeError(
                f"rosbag record 启动失败! 错误: {p_recorder.stderr.read().decode()}"
            )
        print_info("rosbag record 已启动 (PID: {})".format(p_recorder.pid))

        # 4. 播放 ros2 bag
        print_info(f"开始播放 ROS 2 bag: '{ros2_bag_path}'...")

        print_info("使用 'sensor_data' QoS 配置来播放 bag，以提高兼容性。")
        play_cmd = build_command(
            "ros2_play", f"ros2 bag play {ros2_bag_path} --qos-profile sensor_data"
        )
        # player_process 单独处理，因为它的结束是整个流程结束的标志
        player_process = subprocess.Popen(play_cmd, shell=True, executable="/bin/bash")
        print_info(
            "ros2 bag play 已启动 (PID: {}). 等待播放完成...".format(player_process.pid)
        )

        # 等待 ros2 bag play 进程结束
        player_process.wait()

        if player_process.returncode == 0:
            print_success("ROS 2 bag 播放完成。")
        else:
            print_error(
                f"ROS 2 bag 播放似乎遇到了错误 (退出码: {player_process.returncode})。"
            )

    except KeyboardInterrupt:
        print_info("\n检测到手动中断 (Ctrl+C)，开始清理进程...")
    except Exception as e:
        print_error(f"脚本运行出错: {e}")
        print_info("开始清理已启动的进程...")
    finally:
        # 清理程序
        print_info("=" * 20)
        print_info("开始关闭所有进程...")

        # 首先尝试终止播放器进程（如果它还在运行）
        if player_process and player_process.poll() is None:
            try:
                # 给它一个机会自己退出
                player_process.terminate()
                player_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                player_process.kill()  # 如果不行就强制杀死
            print_info(f"已停止 ros2 bag play (PID: {player_process.pid})")

        # 逆序关闭其他后台进程
        for p in reversed(processes):
            if p.poll() is None:  # 如果进程还在运行
                try:
                    # 发送 SIGINT (等同于 Ctrl+C)，让 roscore 和 rosbag record 优雅关闭
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGINT)
                    print_info(f"正在停止进程组 (PGID: {pgid})...")
                    # 等待进程关闭
                    p.wait(timeout=5)
                except ProcessLookupError:
                    # 进程已经不存在
                    pass
                except subprocess.TimeoutExpired:
                    print_error(f"进程组 {pgid} 未能优雅退出，强制终止...")
                    os.killpg(pgid, signal.SIGKILL)  # 强制杀死
                except Exception as e:
                    print_error(f"关闭进程 {p.pid} 时出错: {e}")

        print_success("所有进程已清理完毕。")
        print_success(f"ROS 1 bag 文件已保存为: {ros1_bag_name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="将 ROS 2 bag 文件通过 ros1_bridge 转换为 ROS 1 bag 文件。",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "ros2_bag", type=str, help="输入的 ROS 2 bag 文件路径 (例如: 'my_ros2_bag/')"
    )
    parser.add_argument(
        "ros1_bag", type=str, help="输出的 ROS 1 bag 文件名 (例如: 'output.bag')"
    )

    args = parser.parse_args()

    # 检查 ROS 2 bag 路径是否存在
    if not os.path.isdir(args.ros2_bag):
        print_error(f"指定的 ROS 2 bag 路径不存在: {args.ros2_bag}")
        sys.exit(1)

    # 检查 launch 文件是否存在
    if not os.path.isfile(BRIDGE_LAUNCH_FILE):
        print_error(f"找不到 Bridge 启动文件: {BRIDGE_LAUNCH_FILE}")
        sys.exit(1)

    run_process_manager(args.ros2_bag, args.ros1_bag)
