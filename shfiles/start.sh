#!/bin/bash

# =====================================================
# 脚本名称: start_all.sh
# 功能: 启动 AirSim、PX4 SITL、ROS 节点、MADL Python 节点和 QGroundControl
# 用法: ./start_all.sh
# 退出: 按 Ctrl+C 会终止所有启动的进程
# =====================================================

set -e  # 遇到错误立即退出（但会在子进程中处理）

# 颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 用于存储所有后台进程 PID 的数组
declare -a PIDS=()

# 记录脚本启动目录
SCRIPT_DIR="$(pwd)"

# 清理函数：按逆序终止所有记录的后台进程
cleanup() {
    echo -e "${RED}[INFO] 正在关闭所有进程...${NC}"
    # 逆序终止（后启动的先关闭）
    for (( idx=${#PIDS[@]}-1 ; idx>=0 ; idx-- )); do
        pid="${PIDS[idx]}"
        if kill -0 "$pid" 2>/dev/null; then
            echo "  终止进程 $pid"
            kill -SIGTERM "$pid" 2>/dev/null
            # 等待最多 3 秒，然后强制终止
            for _ in {1..30}; do
                if ! kill -0 "$pid" 2>/dev/null; then
                    break
                fi
                sleep 0.1
            done
            if kill -0 "$pid" 2>/dev/null; then
                echo "  强制终止进程 $pid"
                kill -SIGKILL "$pid" 2>/dev/null
            fi
        fi
    done
    echo -e "${GREEN}[INFO] 所有进程已关闭。${NC}"
    exit 0
}

# 捕获 Ctrl+C 和终止信号
trap cleanup SIGINT SIGTERM

# 初始化 conda 函数（用于激活环境）
# 请根据你的 conda 安装路径修改下面这一行（常见路径：~/anaconda3 或 ~/miniconda3）
CONDA_BASE=$(conda info --base 2>/dev/null)
if [ -z "$CONDA_BASE" ]; then
    echo -e "${YELLOW}警告: 无法自动找到 conda 安装路径，尝试使用默认路径 ~/anaconda3${NC}"
    CONDA_BASE="$HOME/anaconda3"
fi
if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
    source "$CONDA_BASE/etc/profile.d/conda.sh"
else
    echo -e "${RED}错误: 找不到 conda.sh，请检查 conda 安装路径${NC}"
    exit 1
fi

# 启动前检查必要的目录和文件
if [ ! -d "/home/sean/work_ws/AirSim/Unreal/Environments/AirSimNH/LinuxNoEditor/AirSimNH/Binaries/Linux" ]; then
    echo -e "${RED}错误: AirSimNH 目录不存在${NC}"
    exit 1
fi
if [ ! -d "/home/sean/work_ws/PX4/Firmware" ]; then
    echo -e "${RED}错误: PX4 Firmware 目录不存在${NC}"
    exit 1
fi
if [ ! -f "/home/sean/Downloads/QGroundControl.AppImage" ]; then
    echo -e "${YELLOW}警告: QGroundControl.AppImage 未找到，将跳过启动${NC}"
fi
if [ ! -d "/home/sean/work_ws/diff_deploy/MADL" ]; then
    echo -e "${RED}错误: MADL 目录不存在${NC}"
    exit 1
fi

# ========== 1. 启动 AirSim 仿真器 ==========
echo -e "${GREEN}[1/7] 启动 AirSim 仿真器...${NC}"
cd /home/sean/work_ws/AirSim/Unreal/Environments/AirSimNH/LinuxNoEditor/AirSimNH/Binaries/Linux/ || exit 1
./AirSimNH &
PIDS+=($!)
echo "   AirSim PID: ${PIDS[-1]}"
sleep 3

# ========== 2. 启动 PX4 SITL ==========
echo -e "${GREEN}[2/7] 启动 PX4 SITL...${NC}"
cd /home/sean/work_ws/PX4/Firmware || exit 1
make px4_sitl none_iris &
PIDS+=($!)
echo "   PX4 SITL PID: ${PIDS[-1]}"
sleep 4

# ========== 3. 启动 AirSim ROS 节点 ==========
echo -e "${GREEN}[3/7] 启动 AirSim ROS 节点...${NC}"
cd /home/sean/work_ws/AirSim/ros || exit 1
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch &
PIDS+=($!)
echo "   airsim_node PID: ${PIDS[-1]}"
sleep 3



# ========== 4. 启动 MAVROS ==========
echo -e "${GREEN}[4/7] 启动 MAVROS px4.launch...${NC}"
roslaunch mavros px4.launch &
PIDS+=($!)
echo "   mavros PID: ${PIDS[-1]}"
sleep 2

# ==========5. 启动 QGroundControl ==========
if [ -f "/home/sean/Downloads/QGroundControl.AppImage" ]; then
    echo -e "${GREEN}[5/7] 启动 QGroundControl...${NC}"
    cd /home/sean/Downloads || exit 1
    ./QGroundControl.AppImage &
    PIDS+=($!)
    echo "   QGroundControl PID: ${PIDS[-1]}"
else
    echo -e "${YELLOW}[5/7] 跳过 QGroundControl (文件不存在)${NC}"
fi


# ========== 6. 启动 MADL Python 节点（conda 环境 yopo） ==========
echo -e "${GREEN}[6/7] 启动 MADL Python 节点 (conda env: yopo)...${NC}"
cd /home/sean/work_ws/diff_deploy/MADL || exit 1
conda activate yopo
python test_madl_ros.py &
PIDS+=($!)
echo "   MADL Python PID: ${PIDS[-1]}"
# 返回原目录（可选）
cd "$SCRIPT_DIR"
sleep 1

# ========== 7. 启动 offboard_controller ==========
echo -e "${GREEN}[7/7] 启动 offboard_controller...${NC}"
cd /home/sean/work_ws/diff_deploy/controller || exit 1
source devel/setup.bash
roslaunch diffphy_uav_controller offboard_controller.launch &
PIDS+=($!)
echo "   offboard_controller PID: ${PIDS[-1]}"
cd "$SCRIPT_DIR"
sleep 2


echo -e "${GREEN}所有进程已启动。按 Ctrl+C 终止。${NC}"
echo -e "${YELLOW}当前运行的进程 PID: ${PIDS[*]}${NC}"

# 等待所有后台进程（保持脚本运行）
wait