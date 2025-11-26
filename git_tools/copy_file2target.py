import shutil
import os
import datetime

dirs = '''/home/hello/nav_override_ws/debug_src/nav2_costmap_2d
/home/hello/nav_override_ws/src/hal_interface
/home/hello/nav_override_ws/src/whstlidar_ros2_driver
/home/hello/nav_override_ws/src/zeromq_bridge
/home/hello/nav_override_ws/src/agv_start_scrpts
/home/hello/nav_override_ws/src/hik_agv_debug_tools
/home/hello/nav_override_ws/override_src/nav2_behaviors
/home/hello/nav_override_ws/override_src/nav2_mppi_controller
/home/hello/nav_override_ws/override_src/nav2_controller
/home/hello/nav_override_ws/override_src/nav2_regulated_pure_pursuit_controller
/home/hello/nav_override_ws/override_src/nav2_bringup
/home/hello/nav_override_ws/override_src/nav2_bt_navigator
/home/hello/nav_override_ws/override_src/nav2_planner'''

time_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
target_dir = '/home/hello/backups/%s' % time_str  # 设置目标目录

# 需要删除的文件夹列表
folders_to_remove = ['.git', '.idea', '.vscode', 'cmake-build-debug']

# 确保目标目录存在
if not os.path.exists(target_dir):
    os.makedirs(target_dir)

# 复制所有文件夹到目标文件夹
for dir_path in dirs.strip().split('\n'):
    if os.path.exists(dir_path):
        dir_name = os.path.basename(dir_path)
        target_path = os.path.join(target_dir, dir_name)

        try:
            # 如果目标目录已存在，先删除
            if os.path.exists(target_path):
                shutil.rmtree(target_path)

            # 复制目录
            shutil.copytree(dir_path, target_path)
            print(f"已复制: {dir_path} -> {target_path}")

            # 删除指定文件夹
            for folder in folders_to_remove:
                folder_path = os.path.join(target_path, folder)
                if os.path.exists(folder_path):
                    shutil.rmtree(folder_path)
                    print(f"  已删除: {folder_path}")

        except Exception as e:
            print(f"处理失败: {dir_path} - 错误: {str(e)}")
    else:
        print(f"源目录不存在: {dir_path}")

print("所有目录复制和清理完成！")
