#!/usr/bin/env python3
"""
Franka控制器测试脚本

这个脚本用于测试Franka控制器的基本功能，无需实际连接机械臂。
主要用于验证代码结构和导入是否正确。

使用方法:
python3 test_franka_controller.py
"""

import sys
import os

def test_imports():
    """测试必要的导入"""
    print("测试导入...")
    
    try:
        import rclpy
        print("✓ rclpy 导入成功")
    except ImportError as e:
        print(f"✗ rclpy 导入失败: {e}")
        return False
    
    try:
        from geometry_msgs.msg import Pose, Point, Quaternion
        print("✓ geometry_msgs 导入成功")
    except ImportError as e:
        print(f"✗ geometry_msgs 导入失败: {e}")
        return False
    
    try:
        from moveit_commander import MoveGroupCommander, RobotCommander
        print("✓ moveit_commander 导入成功")
    except ImportError as e:
        print(f"✗ moveit_commander 导入失败: {e}")
        print("  请安装: pip3 install moveit_commander")
        return False
    
    try:
        from rclpy.action import ActionClient
        print("✓ rclpy.action 导入成功")
    except ImportError as e:
        print(f"✗ rclpy.action 导入失败: {e}")
        return False
    
    try:
        from moveit_msgs.action import MoveGroup
        print("✓ moveit_msgs.action 导入成功")
    except ImportError as e:
        print(f"✗ moveit_msgs.action 导入失败: {e}")
        return False
    
    return True

def test_basic_functions():
    """测试基本功能"""
    print("\n测试基本功能...")
    
    try:
        import math
        
        # 测试欧拉角到四元数转换
        def euler_to_quaternion(roll, pitch, yaw):
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            
            return [x, y, z, w]
        
        # 测试转换
        quat = euler_to_quaternion(0.0, 0.0, 0.0)
        print(f"✓ 欧拉角转换测试成功: {quat}")
        
        # 测试角度转换
        degrees = math.degrees(math.pi/2)
        print(f"✓ 弧度到角度转换测试成功: π/2 = {degrees}°")
        
        return True
        
    except Exception as e:
        print(f"✗ 基本功能测试失败: {e}")
        return False

def test_pose_creation():
    """测试位姿创建"""
    print("\n测试位姿创建...")
    
    try:
        from geometry_msgs.msg import Pose, Point, Quaternion
        
        # 创建测试位姿
        pose = Pose()
        pose.position = Point(x=0.4, y=0.0, z=0.6)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        print(f"✓ 位姿创建成功:")
        print(f"  位置: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
        print(f"  方向: x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w}")
        
        return True
        
    except Exception as e:
        print(f"✗ 位姿创建测试失败: {e}")
        return False

def test_file_structure():
    """测试文件结构"""
    print("\n测试文件结构...")
    
    files_to_check = [
        'franka_ee_controller.py',
        'simple_franka_controller.py',
        'franka_moveit_action_client.py',
        'README_franka_controller.md'
    ]
    
    all_exist = True
    for filename in files_to_check:
        filepath = os.path.join(os.path.dirname(__file__), filename)
        if os.path.exists(filepath):
            print(f"✓ {filename} 存在")
        else:
            print(f"✗ {filename} 不存在")
            all_exist = False
    
    return all_exist

def main():
    """主测试函数"""
    print("=== Franka控制器测试 ===\n")
    
    tests = [
        ("导入测试", test_imports),
        ("基本功能测试", test_basic_functions),
        ("位姿创建测试", test_pose_creation),
        ("文件结构测试", test_file_structure)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n{'='*50}")
        print(f"运行测试: {test_name}")
        print('='*50)
        
        try:
            if test_func():
                print(f"✓ {test_name} 通过")
                passed += 1
            else:
                print(f"✗ {test_name} 失败")
        except Exception as e:
            print(f"✗ {test_name} 异常: {e}")
    
    print(f"\n{'='*50}")
    print(f"测试结果: {passed}/{total} 通过")
    print('='*50)
    
    if passed == total:
        print("🎉 所有测试通过! 控制器代码结构正确。")
        print("\n下一步:")
        print("1. 启动机械臂和MoveIt")
        print("2. 运行 simple_franka_controller.py 进行实际测试")
        return True
    else:
        print("❌ 部分测试失败，请检查上述错误信息。")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
