#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import time
import argparse
import os
import signal
import sys
from typing import List, Optional
import queue
import threading

# ==============================================================================
# 用户配置区域
# ==============================================================================
NOETIC_SETUP = "source /opt/ros/noetic/setup.bash"
GALACTIC_SETUP = "source /opt/ros/galactic/setup.bash"
LIVOX2_ROS1_WS_SETUP = "source ~/catkin_ws/devel/setup.bash"
BRIDGE_WS_SETUP = (
    "source /root/shared_files/ros1_ros2_bridge_prj/bridge_ws/install/local_setup.bash"
)
ROS_MASTER_URI_EXPORT = "export ROS_MASTER_URI=http://localhost:11311"

ROS1_ENV_CMD = f"{NOETIC_SETUP} && {LIVOX2_ROS1_WS_SETUP}"
ROS2_ENV_CMD = f"{GALACTIC_SETUP} && {BRIDGE_WS_SETUP}"
BRIDGE_ENV_CMD = f"{ROS1_ENV_CMD} && {ROS2_ENV_CMD} && {ROS_MASTER_URI_EXPORT}"

ENV_SETUP = {
    "roscore": ROS1_ENV_CMD,
    "bridge": BRIDGE_ENV_CMD,
    "ros1_record": ROS1_ENV_CMD,
    "ros2_play": ROS2_ENV_CMD,
    "ros2_compress": ROS2_ENV_CMD,
}

BRIDGE_LAUNCH_FILE = os.path.expanduser("~/dynamic_bridge_launch.py")

# --- 基础话题配置 ---
RAW_IMAGE_TOPIC = "/camera/camera/color/image_raw"
COMPRESSED_IMAGE_TOPIC = f"{RAW_IMAGE_TOPIC}/compressed"
OTHER_TOPICS_TO_RECORD = [
    "/livox/lidar",
    "/livox/imu",
    "/livox/point",
    "/pcl_pose",
]
# ==============================================================================


def print_info(message: str):
    print(f"\033[94m[INFO] {message}\033[0m")


def print_error(message: str):
    print(f"\033[91m[ERROR] {message}\033[0m", file=sys.stderr)


def print_success(message: str):
    print(f"\033[92m[SUCCESS] {message}\033[0m")


def print_warning(message: str):
    print(f"\033[93m[WARNING] {message}\033[0m")


def build_command(env_key: str, command: str) -> str:
    env_cmd = ENV_SETUP.get(env_key)
    return f"{env_cmd} && {command}" if env_cmd else command


def preexec_fn():
    os.setsid()


def is_roscore_running() -> bool:
    try:
        cmd = build_command("roscore", "rosnode list")
        subprocess.run(
            cmd,
            shell=True,
            check=True,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
    except subprocess.CalledProcessError:
        return False


def _reader_thread(stream, q):
    for line in iter(stream.readline, ""):
        q.put(line)
    stream.close()


def verify_data_flow(
    ros2_bag_path: str,
    topic: str,
    max_attempts: int = 15,
    silence_timeout: int = 10,
    required_successes: int = 5,
) -> bool:
    for attempt in range(1, max_attempts + 1):
        print_info(
            f"数据流稳定性验证尝试: {attempt}/{max_attempts} (验证话题: {topic})..."
        )
        player_proc, hz_proc = None, None
        try:
            play_cmd = build_command(
                "ros2_play",
                f"ros2 bag play {ros2_bag_path} -r 0.5 --read-ahead-queue-size 2000",
            )
            player_proc = subprocess.Popen(
                play_cmd,
                shell=True,
                preexec_fn=preexec_fn,
                executable="/bin/bash",
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            hz_cmd_str = f"python3 -u $(which rostopic) hz {topic}"
            hz_cmd = build_command("ros1_record", hz_cmd_str)
            hz_proc = subprocess.Popen(
                hz_cmd,
                shell=True,
                preexec_fn=preexec_fn,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            output_queue = queue.Queue()
            reader = threading.Thread(
                target=_reader_thread, args=(hz_proc.stdout, output_queue)
            )
            reader.daemon = True
            reader.start()
            success_counter = 0
            last_valid_data_time = time.time()
            while True:
                if time.time() - last_valid_data_time > silence_timeout:
                    print_warning(
                        f"在 {silence_timeout} 秒内未检测到新的有效数据，此轮尝试失败。"
                    )
                    break
                try:
                    line = output_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if "average rate" in line:
                    success_counter += 1
                    last_valid_data_time = time.time()
                    print_info(
                        f"检测到有效速率更新... (连续成功: {success_counter}/{required_successes})"
                    )
                    if success_counter >= required_successes:
                        print_success("检测到连续稳定的数据流！")
                        return True
                elif "no new messages" in line:
                    success_counter = 0
        finally:
            for p in [player_proc, hz_proc]:
                if p and p.poll() is None:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            time.sleep(2)
    print_error("所有验证尝试均失败，无法建立稳定的数据流。")
    return False


def run_process_manager(ros2_bag_path: str, ros1_bag_name: str, compress: bool):
    # =======================================================
    # === 改动 1: 在函数开始时记录时间 ===
    # =======================================================
    start_time = time.time()

    processes: List[subprocess.Popen] = []
    player_process: Optional[subprocess.Popen] = None
    p_roscore: Optional[subprocess.Popen] = None
    roscore_started_by_script = False
    conversion_successful = False

    if compress:
        print_info("运行在 [压缩模式]下，将启动图像压缩节点。")
        verification_topic = COMPRESSED_IMAGE_TOPIC
        image_topic_to_record = COMPRESSED_IMAGE_TOPIC
    else:
        print_info("运行在 [标准模式]下，不进行图像压缩。")
        verification_topic = COMPRESSED_IMAGE_TOPIC
        image_topic_to_record = COMPRESSED_IMAGE_TOPIC

    ros1_record_topics = [image_topic_to_record] + OTHER_TOPICS_TO_RECORD

    try:
        print_info("检查 roscore 状态...")
        if not is_roscore_running():
            print_info("roscore 未运行，正在启动...")
            p_roscore = subprocess.Popen(
                build_command("roscore", "roscore"),
                shell=True,
                preexec_fn=preexec_fn,
                executable="/bin/bash",
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            roscore_started_by_script = True
            time.sleep(2)
            processes.append(p_roscore)
            if p_roscore.poll() is not None:
                raise RuntimeError(
                    f"roscore 启动失败! 错误: {p_roscore.stderr.read().decode()}"
                )
            print_success(f"roscore 已由本脚本启动 (PID: {p_roscore.pid})")
        else:
            print_success("检测到 roscore 已在运行，将使用现有实例。")

        print_info("启动 dynamic_bridge...")
        p_bridge = subprocess.Popen(
            build_command("bridge", f"ros2 launch {BRIDGE_LAUNCH_FILE}"),
            shell=True,
            preexec_fn=preexec_fn,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        processes.append(p_bridge)
        print_info("等待 3 秒，确保 bridge 完成初始化...")
        time.sleep(3)
        if p_bridge.poll() is not None:
            raise RuntimeError(
                f"Bridge 启动失败! 错误: {p_bridge.stderr.read().decode()}"
            )
        print_info(f"dynamic_bridge 已启动 (PID: {p_bridge.pid})")

        if compress:
            print_info("启动图像压缩节点 (image_transport republish)...")
            compress_cmd = build_command(
                "ros2_compress",
                f"ros2 run image_transport republish raw compressed --ros-args --remap in:={RAW_IMAGE_TOPIC} --remap out/compressed:={COMPRESSED_IMAGE_TOPIC}",
            )
            p_compressor = subprocess.Popen(
                compress_cmd,
                shell=True,
                preexec_fn=preexec_fn,
                executable="/bin/bash",
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            processes.append(p_compressor)
            time.sleep(2)
            if p_compressor.poll() is not None:
                raise RuntimeError(
                    f"图像压缩节点启动失败! 错误: {p_compressor.stderr.read().decode()}"
                )
            print_info(f"图像压缩节点已启动 (PID: {p_compressor.pid})")

        if not verify_data_flow(ros2_bag_path, verification_topic):
            raise RuntimeError("无法通过主动验证建立数据流，脚本终止。")

        print_info("=" * 30)
        print_success("系统已就绪，开始正式录制和播放！")
        print_info(f"将要录制以下 topics: {', '.join(ros1_record_topics)}")
        record_cmd = build_command(
            "ros1_record",
            f"rosbag record {' '.join(ros1_record_topics)} -O {ros1_bag_name}",
        )
        p_recorder = subprocess.Popen(
            record_cmd,
            shell=True,
            preexec_fn=preexec_fn,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        processes.append(p_recorder)
        time.sleep(2)
        if p_recorder.poll() is not None:
            raise RuntimeError(f"rosbag record 启动失败!")
        print_info(f"rosbag record 已启动 (PID: {p_recorder.pid})")

        play_cmd = build_command(
            "ros2_play", f"ros2 bag play {ros2_bag_path} --read-ahead-queue-size 2000"
        )
        player_process = subprocess.Popen(
            play_cmd, shell=True, preexec_fn=preexec_fn, executable="/bin/bash"
        )
        print_info(f"ros2 bag play 已启动 (PID: {player_process.pid}). 等待播放完成...")
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
    finally:
        print_info("=" * 30)
        print_info("开始清理所有由本脚本启动的进程...")
        if player_process and player_process.poll() is None:
            try:
                os.killpg(os.getpgid(player_process.pid), signal.SIGKILL)
                print_info("已停止 ros2 bag play")
            except ProcessLookupError:
                pass
        for p in reversed(processes):
            if p and p.poll() is None:
                pgid = os.getpgid(p.pid)
                print_info(f"正在尝试优雅关闭进程组 (PGID: {pgid})...")
                try:
                    os.killpg(pgid, signal.SIGINT)
                    p.wait(timeout=5)
                    print_success(f"进程组 (PGID: {pgid}) 已优雅关闭。")
                except (subprocess.TimeoutExpired, PermissionError):
                    print_warning(f"进程组 {pgid} 未能优雅关闭，强制终止...")
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        print_success("所有转换相关进程已清理完毕。")

        if os.path.exists(ros1_bag_name):
            print_success(f"ROS 1 bag 文件已成功保存为: {ros1_bag_name}")
            conversion_successful = True
        elif os.path.exists(ros1_bag_name + ".active"):
            print_error(f"Bag 文件未能成功关闭，仍为: {ros1_bag_name}.active")
        else:
            print_warning("未找到输出的 bag 文件，跳过后续步骤。")

    if conversion_successful:
        print_info("=" * 40)
        print_success("ROS1 Bag 转换完成，开始启动建图和回放流程...")
        print_info("=" * 40)

        p_mapping = None
        p_play_ros1 = None

        try:
            print_info("正在启动建图节点 (mapping_mid360.launch)...")
            map_launch_cmd_str = (
                "cd /root/catkin_ws && " "roslaunch map_updater mapping_mid360.launch"
            )
            map_launch_cmd = build_command("ros1_record", map_launch_cmd_str)
            p_mapping = subprocess.Popen(
                map_launch_cmd,
                shell=True,
                preexec_fn=preexec_fn,
                executable="/bin/bash",
            )
            print_success(
                f"建图节点已启动 (PID: {p_mapping.pid})。等待 5 秒以确保节点完全初始化..."
            )
            time.sleep(5)

            if p_mapping.poll() is not None:
                raise RuntimeError("建图节点启动失败，请检查launch文件和ROS环境。")

            print_info(f"开始播放转换后的 ROS1 bag: {ros1_bag_name}")
            play_ros1_cmd_str = f"rosbag play {ros1_bag_name}"
            play_ros1_cmd = build_command("ros1_record", play_ros1_cmd_str)
            p_play_ros1 = subprocess.Popen(
                play_ros1_cmd, shell=True, preexec_fn=preexec_fn, executable="/bin/bash"
            )
            print_info(f"rosbag play 已启动 (PID: {p_play_ros1.pid})，等待播放结束...")
            p_play_ros1.wait()

            if p_play_ros1.returncode == 0:
                print_success("ROS1 bag 播放完成。")
            else:
                print_error(
                    f"ROS1 bag 播放似乎遇到了错误 (退出码: {p_play_ros1.returncode})。"
                )

            print_info("ROS1 bag 播放完毕，准备保存地图...")
            save_map_cmd_str = "rosservice call /save_map"
            save_map_cmd = build_command("ros1_record", save_map_cmd_str)

            save_map_result = subprocess.run(
                save_map_cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
            )

            if save_map_result.returncode == 0:
                print_success("地图保存服务调用成功！")
                print_info(f"服务输出: {save_map_result.stdout.strip()}")

                # =======================================================
                # === 改动 2: 计算并打印总耗时 ===
                # =======================================================
                end_time = time.time()
                elapsed_seconds = end_time - start_time
                minutes = int(elapsed_seconds // 60)
                seconds = int(elapsed_seconds % 60)
                print_success(f"全流程处理完成，总耗时: {minutes} 分 {seconds} 秒。")

                # Debug: 暂停以便检查输出
                time.sleep(888888)
                # =======================================================
            else:
                print_error("地图保存服务调用失败！")
                print_error(f"错误详情: {save_map_result.stderr.strip()}")

        except KeyboardInterrupt:
            print_info("\n检测到手动中断 (Ctrl+C)，开始清理建图和回放进程...")
        except Exception as e:
            print_error(f"建图或回放阶段出错: {e}")
        finally:
            print_info("开始清理建图和回放进程...")
            for p in [p_play_ros1, p_mapping]:
                if p and p.poll() is None:
                    pgid = os.getpgid(p.pid)
                    print_info(f"正在终止进程组 (PGID: {pgid})...")
                    try:
                        os.killpg(pgid, signal.SIGINT)
                        p.wait(timeout=3)
                        print_success(f"进程组 (PGID: {pgid}) 已关闭。")
                    except (subprocess.TimeoutExpired, ProcessLookupError):
                        print_warning(f"进程组 {pgid} 关闭超时，强制终止...")
                        try:
                            os.killpg(pgid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
            print_success("所有进程清理完毕，脚本执行结束。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="将 ROS 2 bag 转换为 ROS 1 bag，并可选择进行图像压缩。转换成功后会自动启动建图、回放并保存地图。",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("ros2_bag", type=str, help="输入的 ROS 2 bag 文件路径")
    parser.add_argument("ros1_bag", type=str, help="输出的 ROS 1 bag 文件名")
    parser.add_argument(
        "--compress",
        action="store_true",
        help="添加此标志以对包内的 raw image 进行压缩处理。默认不压缩。",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.ros2_bag):
        print_error(f"指定的 ROS 2 bag 路径不存在: {args.ros2_bag}")
        sys.exit(1)
    if not os.path.isfile(BRIDGE_LAUNCH_FILE):
        print_error(f"找不到 Bridge 启动文件: {BRIDGE_LAUNCH_FILE}")
        sys.exit(1)

    run_process_manager(args.ros2_bag, args.ros1_bag, args.compress)
