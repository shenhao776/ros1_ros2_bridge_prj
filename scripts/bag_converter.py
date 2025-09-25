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
# User Configuration Area
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

# --- Basic Topic Configuration ---
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
            f"Data flow stability verification attempt: {attempt}/{max_attempts} (verifying topic: {topic})..."
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
                        f"No new valid data detected within {silence_timeout} seconds, this attempt failed."
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
                        f"Detected valid rate update... (consecutive successes: {success_counter}/{required_successes})"
                    )
                    if success_counter >= required_successes:
                        print_success("Detected continuous stable data flow!")
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
    print_error(
        "All verification attempts failed, unable to establish stable data flow."
    )
    return False


def run_process_manager(ros2_bag_path: str, ros1_bag_name: str, compress: bool):
    # =======================================================
    # === Change 1: Record time at the start of the function ===
    # =======================================================
    start_time = time.time()

    processes: List[subprocess.Popen] = []
    player_process: Optional[subprocess.Popen] = None
    p_roscore: Optional[subprocess.Popen] = None
    roscore_started_by_script = False
    conversion_successful = False

    if compress:
        print_info("Running in [compression mode], will start image compression node.")
        verification_topic = COMPRESSED_IMAGE_TOPIC
        image_topic_to_record = COMPRESSED_IMAGE_TOPIC
    else:
        print_info("Running in [standard mode], no image compression.")
        verification_topic = COMPRESSED_IMAGE_TOPIC
        image_topic_to_record = COMPRESSED_IMAGE_TOPIC

    ros1_record_topics = [image_topic_to_record] + OTHER_TOPICS_TO_RECORD

    try:
        print_info("Checking roscore status...")
        if not is_roscore_running():
            print_info("roscore is not running, starting now...")
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
                    f"roscore failed to start! Error: {p_roscore.stderr.read().decode()}"
                )
            print_success(f"roscore started by this script (PID: {p_roscore.pid})")
        else:
            print_success(
                "Detected roscore is already running, will use existing instance."
            )

        print_info("Starting dynamic_bridge...")
        p_bridge = subprocess.Popen(
            build_command("bridge", f"ros2 launch {BRIDGE_LAUNCH_FILE}"),
            shell=True,
            preexec_fn=preexec_fn,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        processes.append(p_bridge)
        print_info("Waiting 3 seconds to ensure bridge completes initialization...")
        time.sleep(3)
        if p_bridge.poll() is not None:
            raise RuntimeError(
                f"Bridge failed to start! Error: {p_bridge.stderr.read().decode()}"
            )
        print_info(f"dynamic_bridge started (PID: {p_bridge.pid})")

        if compress:
            print_info("Starting image compression node (image_transport republish)...")
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
                    f"Image compression node failed to start! Error: {p_compressor.stderr.read().decode()}"
                )
            print_info(f"Image compression node started (PID: {p_compressor.pid})")

        if not verify_data_flow(ros2_bag_path, verification_topic):
            raise RuntimeError(
                "Failed to establish data flow through active verification, script terminated."
            )

        print_info("=" * 30)
        print_success("System is ready, starting formal recording and playback!")
        print_info(f"Topics to be recorded: {', '.join(ros1_record_topics)}")
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
            raise RuntimeError(f"rosbag record failed to start!")
        print_info(f"rosbag record started (PID: {p_recorder.pid})")

        play_cmd = build_command(
            "ros2_play", f"ros2 bag play {ros2_bag_path} --read-ahead-queue-size 2000"
        )
        player_process = subprocess.Popen(
            play_cmd, shell=True, preexec_fn=preexec_fn, executable="/bin/bash"
        )
        print_info(
            f"ros2 bag play started (PID: {player_process.pid}). Waiting for playback to complete..."
        )
        player_process.wait()

        if player_process.returncode == 0:
            print_success("ROS 2 bag playback completed.")
        else:
            print_error(
                f"ROS 2 bag playback似乎遇到了错误 (exit code: {player_process.returncode})。"
            )

    except KeyboardInterrupt:
        print_info(
            "\nManual interruption detected (Ctrl+C), starting process cleanup..."
        )
    except Exception as e:
        print_error(f"Script error occurred: {e}")
    finally:
        print_info("=" * 30)
        print_info("Starting cleanup of all processes started by this script...")
        if player_process and player_process.poll() is None:
            try:
                os.killpg(os.getpgid(player_process.pid), signal.SIGKILL)
                print_info("Stopped ros2 bag play")
            except ProcessLookupError:
                pass
        for p in reversed(processes):
            if p and p.poll() is None:
                pgid = os.getpgid(p.pid)
                print_info(
                    f"Attempting graceful shutdown of process group (PGID: {pgid})..."
                )
                try:
                    os.killpg(pgid, signal.SIGINT)
                    p.wait(timeout=5)
                    print_success(f"Process group (PGID: {pgid}) shut down gracefully.")
                except (subprocess.TimeoutExpired, PermissionError):
                    print_warning(
                        f"Process group {pgid} failed to shut down gracefully, force terminating..."
                    )
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        print_success("All conversion-related processes have been cleaned up.")

        if os.path.exists(ros1_bag_name):
            print_success(f"ROS 1 bag file successfully saved as: {ros1_bag_name}")
            conversion_successful = True
        elif os.path.exists(ros1_bag_name + ".active"):
            print_error(
                f"Bag file failed to close properly, still exists as: {ros1_bag_name}.active"
            )
        else:
            print_warning("No output bag file found, skipping subsequent steps.")

    if conversion_successful:
        print_info("=" * 40)
        print_success(
            "ROS1 Bag conversion completed, starting mapping and playback process..."
        )
        print_info("=" * 40)

        p_mapping = None
        p_play_ros1 = None

        try:
            print_info("Starting mapping node (mapping_mid360.launch)...")
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
                f"Mapping node started (PID: {p_mapping.pid}). Waiting 8 seconds to ensure node fully initializes..."
            )
            time.sleep(8)

            if p_mapping.poll() is not None:
                raise RuntimeError(
                    "Mapping node failed to start, please check launch file and ROS environment."
                )

            print_info(f"Starting playback of converted ROS1 bag: {ros1_bag_name}")
            play_ros1_cmd_str = f"rosbag play {ros1_bag_name}"
            play_ros1_cmd = build_command("ros1_record", play_ros1_cmd_str)
            p_play_ros1 = subprocess.Popen(
                play_ros1_cmd, shell=True, preexec_fn=preexec_fn, executable="/bin/bash"
            )
            print_info(
                f"rosbag play started (PID: {p_play_ros1.pid}), waiting for playback to finish..."
            )
            p_play_ros1.wait()

            if p_play_ros1.returncode == 0:
                print_success("ROS1 bag playback completed.")
            else:
                print_error(
                    f"ROS1 bag playback encountered an error (exit code: {p_play_ros1.returncode})."
                )

            print_info("ROS1 bag playback finished, preparing to save map...")
            end_time = time.time()
            elapsed_seconds = end_time - start_time
            minutes = int(elapsed_seconds // 60)
            seconds = int(elapsed_seconds % 60)
            print_success(
                f"The time of conversion from ros2 bag to ros1 bag is: {minutes} minutes {seconds} seconds."
            )
            # Debug: Pause to inspect output
            time.sleep(888888)

        except KeyboardInterrupt:
            print_info(
                "\nManual interruption detected (Ctrl+C), starting cleanup of mapping and playback processes..."
            )
        except Exception as e:
            print_error(f"Error in mapping or playback phase: {e}")
        finally:
            print_info("Starting cleanup of mapping and playback processes...")
            for p in [p_play_ros1, p_mapping]:
                if p and p.poll() is None:
                    pgid = os.getpgid(p.pid)
                    print_info(f"Terminating process group (PGID: {pgid})...")
                    try:
                        os.killpg(pgid, signal.SIGINT)
                        p.wait(timeout=3)
                        print_success(f"Process group (PGID: {pgid}) closed.")
                    except (subprocess.TimeoutExpired, ProcessLookupError):
                        print_warning(
                            f"Process group {pgid} shutdown timed out, force terminating..."
                        )
                        try:
                            os.killpg(pgid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
            print_success("All processes cleaned up, script execution completed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Converts ROS 2 bag to ROS 1 bag with optional image compression. Automatically starts mapping, playback and saves map after successful conversion.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("ros2_bag", type=str, help="Path to input ROS 2 bag file")
    parser.add_argument("ros1_bag", type=str, help="Name for output ROS 1 bag file")
    parser.add_argument(
        "--compress",
        action="store_true",
        help="Add this flag to compress raw images in the bag. No compression by default.",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.ros2_bag):
        print_error(f"Specified ROS 2 bag path does not exist: {args.ros2_bag}")
        sys.exit(1)
    if not os.path.isfile(BRIDGE_LAUNCH_FILE):
        print_error(f"Bridge launch file not found: {BRIDGE_LAUNCH_FILE}")
        sys.exit(1)

    run_process_manager(args.ros2_bag, args.ros1_bag, args.compress)
