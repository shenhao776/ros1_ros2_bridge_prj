# Convert the ros2bag with livox lidar custom messages to a ros1bag
# 把带有livox雷达的自定义消息的ros2bag转成ros1bag

## .bashrc for reference
```bash
# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# don't put duplicate lines in the history.
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "$debian_chroot" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Alias definitions.
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# ===============================================================
# ================= ROS Environment Setup =======================
# ===============================================================

# --- Base ROS Environments ---
# ROS 1 Noetic
alias ros1_env="source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && echo 'ROS 1 Noetic environment sourced.'"

# ROS 2 Galactic
alias ros2_env="source /opt/ros/galactic/setup.bash && source /root/shared_files/ros1_ros2_bridge_prj/bridge_ws/install/local_setup.bash && echo 'ROS 2 Galactic environment sourced.'"

# --- Bridge Environment ---
# ROS 1, ROS 2, and Bridge Workspaces
alias bridge_env="source /opt/ros/noetic/setup.bash && \
		          source ~/catkin_ws/devel/setup.bash && \
                  source /opt/ros/galactic/setup.bash && \
                  source /root/shared_files/ros1_ros2_bridge_prj/bridge_ws/install/local_setup.bash && \
                  export ROS_MASTER_URI=http://localhost:11311 && \
                  echo 'ROS1, ROS2 & Bridge environments sourced. ROS_MASTER_URI is set.'"

# Set a default environment for new terminals
#ros1_env
# ros2_env
# bridge_env

# ===============================================================
# ================= End of ROS Setup ============================
# ===============================================================
```

# Run with docker (recommend)
```bash
docker pull shenhao776/amr_ros1_x86:v0.3

# Build 
TODO

# replace '/path_to_ros2_bag_file' and '/root/shared_files/rosbag/ros1bag/lvio_bag/bag_name.bag'
docker run -it --rm -v /tmp/.x11-unix:/tmp/.x11-unix \
                -v ~/shared_files:/root/shared_files \
                -v /dev:/dev \
                --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
                -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
                --privileged --net=host --user root shenhao776/amr_ros1_x86:v0.3 \
                /root/shared_files/ros1_ros2_bridge_prj/scripts/bag_converter.py \
                /root/shared_files/rosbag/ros1bag/ros2bags/nav_bag_2025-09-21_23-33-36 /root/shared_files/rosbag/ros1bag/lvio_bag/nav_bag_2025-09-21_23-33-36.bag

# run in another docker container
     ssh -o StrictHostKeyChecking=no hao@localhost \
     "docker run --rm -e ROS_DOMAIN_ID=1 \
     -v /tmp/.x11-unix:/tmp/.x11-unix \
     -v ~/shared_files:/root/shared_files \
     -v /dev:/dev \
     --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
     -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
     --privileged --net=host --user root shenhao776/amr_ros1_x86:v0.3 \
     /root/shared_files/ros1_ros2_bridge_prj/scripts/bag_converter.py \
     /root/shared_files/rosbag/ros1bag/ros2bags/nav_bag_2025-09-21_23-33-36 /root/shared_files/rosbag/ros1bag/lvio_bag/nav_bag_2025-09-21_23-33-36.bag"
```

## Manually run
```bash
# one command 
./bag_converter.py /ros2_bag_file ros1_bag.bag

# or step by step: 
# terminal 1
ros1_env
roscore
# terminal 2
bridge_env
ros2 launch ~/dynamic_bridge_launch.py
# terminal 3
bridge_env
rosbag record /camera/camera/color/image_raw/compressed /livox/lidar /livox/imu /livox/point -O ros1_bag_name.bag
# terminal 4
ros2_env
# compress images
ros2 run image_transport republish raw compressed --ros-args --remap in:=/camera/camera/color/image_raw --remap out/compressed:=/camera/camera/color/image_raw/compressed
# terminal 5
ros2_env
# play bag
ros2 bag play /root/shared_files/rosbag/ros1bag/ros2bags/nav_bag_2025-09-21_23-33-36 -r 0.5 --read-ahead-queue-size 2000

# if uncompress
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/camera/color/image_raw/compressed --remap out:=/camera/camera/color/image_raw
```