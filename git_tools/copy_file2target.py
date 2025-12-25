import shutil
import os
import datetime
import sys

from loguru import logger


def delete_log_files_and_dirs(root_dir, dry_run=False):
    """
    递归删除指定目录下所有后缀为 .log 的文件和文件夹。

    :param root_dir: 要扫描的根目录路径
    :param dry_run: 是否为模拟运行（True：只打印，不删除；False：真实删除）
    """
    if not os.path.exists(root_dir):
        print(f"❌ 路径不存在: {root_dir}")
        return

    print(f"🔍 开始扫描目录: {root_dir}")

    for dirpath, dirnames, filenames in os.walk(root_dir, topdown=False):
        # 1. 先处理文件：删除 .log 文件
        for filename in filenames:
            if filename.lower().endswith('.log'):
                file_path = os.path.join(dirpath, filename)
                if dry_run:
                    print(f"📄 [模拟] 将删除文件: {file_path}")
                else:
                    try:
                        os.remove(file_path)
                        print(f"✅ 已删除文件: {file_path}")
                    except Exception as e:
                        print(f"❌ 删除文件失败 {file_path}: {e}")

        # 2. 再处理文件夹：删除 .log 文件夹（注意：从下往上删，topdown=False）
        for dirname in dirnames:
            if dirname.lower().endswith('.log'):
                dir_path = os.path.join(dirpath, dirname)
                if dry_run:
                    print(f"📁 [模拟] 将删除文件夹: {dir_path}")
                else:
                    try:
                        shutil.rmtree(dir_path)
                        print(f"✅ 已删除文件夹: {dir_path}")
                    except Exception as e:
                        print(f"❌ 删除文件夹失败 {dir_path}: {e}")

    print("🎉 清理完成！")


def save_log():
    logger.add("save.log")
    logger.info('🎉' * 5 + target_dir + '🎉' * 5)
    for item in dirs.split('\n'):
        logger.info(item)
    logger.info('*' * 20 + '*' * 20)


dirs = '''/home/hello/nav_override_ws/src/agv_start_scrpts
/home/hello/nav_override_ws/override_src/nav2_controller
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
            delete_log_files_and_dirs(target_dir)

        except Exception as e:
            print(f"处理失败: {dir_path} - 错误: {str(e)}")
    else:
        print(f"源目录不存在: {dir_path}")

print("所有目录复制和清理完成！")
save_log()