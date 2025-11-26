#!/usr/bin/env python3
"""
批量扫描目录下所有 Git 仓库，
找出本地分支领先于其远端跟踪分支的提交记录，
并识别出没有配置任何远端的仓库。
整个扫描过程在本地完成，无需网络。
"""
from datetime import datetime
from pathlib import Path
import argparse
import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed

import git  # GitPython


def find_git_repos(root_path: Path) -> list[Path]:
    """
    递归查找所有包含 .git 子目录的 Git 仓库。
    """
    # 使用 rglob('.git') 可以高效地找到所有 .git 目录
    # .parent 用于获取仓库的根目录
    # 检查 HEAD 文件可以避免将其他仓库的 .git 文件夹（如 submodule）误判为仓库
    return [p.parent for p in root_path.rglob(".git") if p.is_dir() and (p / 'HEAD').is_file()]


def analyze_repo(repo_path: Path) -> dict:
    """
    分析单个仓库的状态：找出领先的分支，或判断其是否无远端。
    这个函数设计为在多进程环境中安全运行。
    """
    try:
        repo = git.Repo(repo_path)

        # 1. 首先检查仓库是否有任何远端配置
        if not repo.remotes:
            return {
                "path": str(repo_path),
                "status": "no_remotes",
                "message": "该仓库没有配置任何远端。"
            }

        # 2. 如果有远端，则查找所有领先的分支
        ahead_results = []
        has_any_tracking_branch = False

        for branch in repo.branches:
            tracking_branch = branch.tracking_branch()
            if tracking_branch:
                has_any_tracking_branch = True
                # 使用 Git 的范围语法 A..B 来获取在 A 中但不在 B 中的提交
                # 范围是 tracking_branch..branch
                ahead_commits_hashes = list(repo.git.log('--pretty=format:%H', f'{tracking_branch.name}..{branch.name}').splitlines())

                if not ahead_commits_hashes:
                    continue

                commits_info = []
                for commit_hash in ahead_commits_hashes:
                    commit = repo.commit(commit_hash)
                    commits_info.append({
                        "hash": commit.hexsha[:8],
                        "author": commit.author.name,
                        "date": commit.committed_datetime.isoformat(timespec='seconds'),
                        "message": commit.message.split('\n')[0].strip(),
                    })

                ahead_results.append({
                    "branch": branch.name,
                    "tracking_branch": tracking_branch.name,
                    "ahead_count": len(commits_info),
                    "commits": commits_info
                })

        # 3. 根据分析结果返回状态
        if not has_any_tracking_branch:
            # 有远端配置，但没有任何分支设置了跟踪
            return {
                "path": str(repo_path),
                "status": "no_tracking",
                "message": "仓库配置了远端，但没有任何本地分支设置了跟踪分支。"
            }
        elif not ahead_results:
            # 所有分支都与远端同步
            return {
                "path": str(repo_path),
                "status": "synced",
                "message": "所有分支都与远端同步。"
            }
        else:
            # 找到了领先的分支
            return {
                "path": str(repo_path),
                "status": "ahead",
                "branches_ahead": ahead_results
            }

    except Exception as e:
        # 如果仓库处理出错，返回错误信息
        return {
            "path": str(repo_path),
            "status": "error",
            "message": str(e)
        }


def main():
    """主函数，处理参数和调度任务。"""
    # 为了方便您直接运行，这里暂时硬编码，您也可以改回 argparse
    root_dir = '/home/hello/nav_override_ws'

    root_path = Path(root_dir).expanduser().resolve()
    if not root_path.is_dir():
        print(f"错误: 目录 '{root_path}' 不存在。")
        return

    print(f"正在扫描目录: {root_path}")
    repo_paths = find_git_repos(root_path)
    if not repo_paths:
        print("未找到任何 Git 仓库。")
        return

    print(f"找到 {len(repo_paths)} 个仓库，开始并行处理...")

    all_results = []
    # 使用进程池来并行处理，大幅提升扫描大量仓库的速度
    with ProcessPoolExecutor(max_workers=3) as executor:
        future_to_repo = {executor.submit(analyze_repo, path): path for path in repo_paths}

        for future in as_completed(future_to_repo):
            result = future.result()
            if result:
                all_results.append(result)

    # --- 输出结果 ---
    print("\n" + "="*80)
    print("扫描结果汇总")
    print("="*80)

    if not all_results:
        print("没有找到任何 Git 仓库。")
        return

    # 按状态分类结果
    no_remotes_repos = [r for r in all_results if r['status'] == 'no_remotes']
    no_tracking_repos = [r for r in all_results if r['status'] == 'no_tracking']
    ahead_repos = [r for r in all_results if r['status'] == 'ahead']
    synced_repos = [r for r in all_results if r['status'] == 'synced']
    error_repos = [r for r in all_results if r['status'] == 'error']

    # 1. 报告需要推送的仓库
    if ahead_repos:
        print("\n📌 以下仓库存在本地分支领先于远端的情况 (需要推送):")
        for res in ahead_repos:
            print(f"\n📂 仓库: {res['path']}")
            for branch_info in res['branches_ahead']:
                print(f"  └─ 分支 '{branch_info['branch']}' (领先于 {branch_info['tracking_branch']}) 领先 {branch_info['ahead_count']} 个提交:")
                for commit in branch_info['commits']:
                    print(f"     {commit['date']} | {commit['hash']} | {commit['message']}")
                print("-" * 60)

    # 2. 报告没有远端的仓库
    if no_remotes_repos:
        print("\n🔗 以下仓库没有配置任何远端:")
        for res in no_remotes_repos:
            print(f"  - {res['path']}")

    # 3. 报告没有跟踪分支的仓库
    if no_tracking_repos:
        print("\n🔀 以下仓库配置了远端，但分支未设置跟踪:")
        for res in no_tracking_repos:
            print(f"  - {res['path']}")

    # 4. (可选) 报告已同步的仓库
    if synced_repos:
        print("\n✅ 以下仓库已与远端同步:")
        for res in synced_repos:
            print(f"  - {res['path']}")

    # 5. 报告处理出错的仓库
    if error_repos:
        print("\n❗️ 以下仓库处理时出错:")
        for res in error_repos:
            print(f"  - {res['path']}: {res['message']}")

    # 6. 如果所有仓库都是干净的
    if not (ahead_repos or no_remotes_repos or no_tracking_repos or error_repos):
        print("\n所有仓库都已与远端同步，无需操作。")

    print("\n" + "="*80)


if __name__ == "__main__":
    main()
