#!/bin/bash

# Franka机械臂简化启动脚本
# 这个脚本提供多种启动选项，帮助解决MoveIt启动问题

echo "🤖 Franka机械臂启动助手"
echo "=========================="

# 获取机械臂IP地址
read -p "请输入机械臂IP地址 (默认: 192.168.2.2): " ROBOT_IP
if [ -z "$ROBOT_IP" ]; then
    ROBOT_IP="192.168.2.2"
fi

echo "使用机械臂IP: $ROBOT_IP"
echo ""

# 测试网络连接
echo "🔍 测试网络连接..."
if ping -c 3 $ROBOT_IP > /dev/null 2>&1; then
    echo "✅ 网络连接正常"
else
    echo "❌ 网络连接失败"
    echo "请检查:"
    echo "  1. 机械臂IP地址是否正确"
    echo "  2. 网络连接是否正常"
    echo "  3. 防火墙设置"
    exit 1
fi

echo ""
echo "请选择启动模式:"
echo "1) 模拟硬件模式 (推荐用于测试)"
echo "2) 基础Franka控制 (不使用MoveIt)"
echo "3) 重力补偿模式"
echo "4) 完整MoveIt启动"
echo "5) 诊断模式"

read -p "请选择 (1-5): " CHOICE

case $CHOICE in
    1)
        echo "🚀 启动模拟硬件模式..."
        ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
        ;;
    2)
        echo "🚀 启动基础Franka控制..."
        ros2 launch franka_bringup franka.launch.py robot_ip:=$ROBOT_IP
        ;;
    3)
        echo "🚀 启动重力补偿模式..."
        ros2 launch franka_bringup example.launch.py controller_name:=gravity_compensation_example_controller robot_ip:=$ROBOT_IP
        ;;
    4)
        echo "🚀 启动完整MoveIt..."
        echo "注意: 如果遇到问题，请先尝试模式1-3"
        ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=$ROBOT_IP
        ;;
    5)
        echo "🔍 启动诊断模式..."
        python3 franka_diagnostic.py
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac
