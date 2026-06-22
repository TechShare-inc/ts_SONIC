#!/bin/bash

# ==========================================
# 0. 定义变量
# ==========================================
PROJECT_ROOT="~/techshare_ws/sonic/GR00T-WholeBodyControl/"
ROBOT_IP="192.168.0.232"
ROBOT_USER="unitree"
ROBOT_PASS="123"
VR_IP="192.168.0.200"

# 定义用于同步状态的临时文件
READY_FILE_1="/tmp/robot_t1_ready.tmp"
READY_FILE_2="/tmp/robot_t2_ready.tmp"

# 清理上一次可能残留的临时文件
rm -f "$READY_FILE_1" "$READY_FILE_2"

echo "Start deploying robot on $ROBOT_IP..."

# ==========================================
# 1. 打开 terminal-1 并等待 release mode
# ==========================================
echo "--> Opening Terminal-1..."

# 修复点：--title 放在 -- 之前；-- 后面直接接 bash 调用 expect
gnome-terminal --title="Terminal-1-Robotics-Service" -- bash -c "expect -c '
    set timeout -1
    spawn ssh ${ROBOT_USER}@${ROBOT_IP}
    expect {
        \"*yes/no*\" { send \"yes\r\"; exp_continue }
        \"*password:*\" { send \"${ROBOT_PASS}\r\" }
    }
    expect \"*@*\"
    send \"cd /opt/apps/roboticsservice && bash runService.sh\r\"
    expect \"release mode\" {
        exec touch $READY_FILE_1
    }
    interact
'" &

# 主脚本等待 T1 的信号
echo "Waiting for Terminal-1 to return 'release mode'..."
while [ ! -f "$READY_FILE_1" ]; do
    sleep 0.5
done
echo "Terminal-1 is ready."


# ==========================================
# 2. 打开 terminal-2 执行视频推流
# ==========================================
echo "--> Opening Terminal-2..."

gnome-terminal --title="Terminal-2-Video-Sender" -- bash -c "expect -c '
    set timeout -1
    spawn ssh ${ROBOT_USER}@${ROBOT_IP}
    expect {
        \"*yes/no*\" { send \"yes\r\"; exp_continue }
        \"*password:*\" { send \"${ROBOT_PASS}\r\" }
    }
    expect \"*@*\"
    send \"cd ~/techshare_ws/ts-XRoboToolkit-Orin-Video-Sender/ && ./OrinVideoSender --send --server ${VR_IP} --port 12345 --camera stereo\r\"
    interact
'" &

# ==========================================
# 3. 打开 terminal-3 自动处理密码、Y、Init Done
# ==========================================
echo "--> Opening Terminal-3..."

gnome-terminal --title="Terminal-3-Deploy-Real" -- bash -c "expect -c '
    set timeout -1
    spawn ssh ${ROBOT_USER}@${ROBOT_IP}
    expect {
        \"*yes/no*\" { send \"yes\r\"; exp_continue }
        \"*password:*\" { send \"${ROBOT_PASS}\r\" }
    }
    expect \"*@*\"
    send \"cd ${PROJECT_ROOT}\r\"
    send \"cd gear_sonic_deploy && source scripts/setup_env.sh && bash deploy.sh real --input-type zmq_manager\r\"

    expect \"password for unitree\" { send \"${ROBOT_PASS}\r\" }
    expect \"Proceed with deployment*\" { send \"Y\r\" }

    expect \"Init Done\" {
        exec touch $READY_FILE_2
    }
    interact
'" &

# 主脚本等待 T2 的信号
echo "Waiting for Terminal-2 to return 'Init Done'..."
while [ ! -f "$READY_FILE_2" ]; do
    sleep 0.5
done
echo "Terminal-2 is ready."


# ==========================================
# 4. 打开 terminal-4 执行 python 脚本
# ==========================================
echo "--> Opening Terminal-4..."

gnome-terminal --title="Terminal-4-Pico-Manager" -- bash -c "expect -c '
    set timeout -1
    spawn ssh ${ROBOT_USER}@${ROBOT_IP}
    expect {
        \"*yes/no*\" { send \"yes\r\"; exp_continue }
        \"*password:*\" { send \"${ROBOT_PASS}\r\" }
    }
    expect \"*@*\"
    send \"cd ${PROJECT_ROOT}\r\"
    send \"source .venv_teleop/bin/activate && python gear_sonic/scripts/pico_manager_thread_server.py --manager --vr_ip_address ${VR_IP}\r\"
    interact
'" &

echo "=========================================="
echo "All terminals are ready."
echo "=========================================="
