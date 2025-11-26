import os
import argparse
from pathlib import Path


def is_file_ok(filename):
    end_list = ['.h', '.cpp', '.md', '.hpp', '.c','.py']
    for ext in end_list:
        if filename.endswith(ext):
            return True
    return False


def merge_markdown_files(source_dir, output_file, recursive=True, add_title=True, add_separator=True):
    """
    遍历指定目录中的所有 Markdown 文件，并将它们的内容合并到一个新文件中。

    参数:
        source_dir (str): 源目录路径
        output_file (str): 输出文件路径
        recursive (bool): 是否递归遍历子目录
        add_title (bool): 是否在每个文件内容前添加文件名作为标题
        add_separator (bool): 是否在每个文件内容后添加分隔符
    """
    # 确保源目录存在
    if not os.path.exists(source_dir):
        print(f"错误: 源目录 '{source_dir}' 不存在")
        return

    # 收集所有 Markdown 文件
    md_files = []
    if recursive:
        # 递归遍历
        for root, _, files in os.walk(source_dir):
            for file in files:
                if is_file_ok(file):
                    md_files.append(os.path.join(root, file))
    else:
        # 仅遍历当前目录
        for file in os.listdir(source_dir):
            if is_file_ok(file) and os.path.isfile(os.path.join(source_dir, file)):
                md_files.append(os.path.join(source_dir, file))

    # 如果没有找到 Markdown 文件
    if not md_files:
        print(f"警告: 在目录 '{source_dir}' 中未找到 Markdown 文件")
        return

    # 按文件路径排序
    md_files.sort()

    # 合并文件内容
    with open(output_file, 'w', encoding='utf-8') as outfile:
        for file_path in md_files:
            try:
                # 获取相对路径作为标题
                rel_path = os.path.relpath(file_path, source_dir)

                # 添加文件名作为标题
                if add_title:
                    outfile.write(f"# {rel_path}\n\n")

                # 读取并写入文件内容
                with open(file_path, 'r', encoding='utf-8') as infile:
                    content = infile.read()
                    outfile.write(content)
                    # 如果内容不以空行结尾，添加一个空行
                    if content and content[-1] != '\n':
                        outfile.write('\n')

                # 添加分隔符
                if add_separator:
                    outfile.write("\n\n" + "---" * 10 + "\n\n")

                print(f"已合并: {rel_path}")
            except Exception as e:
                print(f"错误: 无法合并文件 '{file_path}': {str(e)}")

    print(f"\n合并完成! 输出文件: {output_file}")


def main():
    # 执行合并操作
    merge_markdown_files(
        source_dir='/home/hello/cartographer_ws/src/cartographer_ros/cartographer_ros',
        output_file='/home/hello/桌面/知识库/cartographer_ros.md',
        recursive=True,
        add_title=True,
        add_separator=False
    )


if __name__ == "__main__":
    main()
