#!/usr/bin/env python3
"""
用 GitPython 批量扫描目录下所有 Git 仓库，
提取指定日期之后的提交记录。
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import git  # GitPython


def scan_repos(root: Path):
    """返回所有包含 .git 的子目录"""
    return [p.parent for p in root.rglob(".git") if p.is_dir()]


def collect_since(repo_path: Path, since: datetime):
    repo = git.Repo(repo_path)
    since = since.replace(tzinfo=since.tzinfo or timezone.utc)
    records = []
    for commit in repo.iter_commits(since=since):
        records.append({
            "hash": commit.hexsha,
            "author": commit.author.name,
            "date": commit.committed_datetime.isoformat(),
            "message": commit.message.split("\n")[0],
            "branches": [br.name for br in repo.branches if commit in br.commit.iter_parents(n=0, max_count=1)]
        })
    return repo, records


def main():
    root_dir = '/home/hello/nav_override_ws'
    since_date = '2025-12-10'
    since_dt = datetime.strptime(since_date, "%Y-%m-%d")

    root = Path(root_dir).expanduser().resolve()

    all_path = []
    for repo_path in scan_repos(root):
        try:
            repo, commits = collect_since(repo_path, since_dt)
        except Exception as e:
            print(f"跳过 {repo_path} : {e}")
            continue
        if commits:
            all_path.append(str(repo_path))
            print(f"\n📌 >>> {repo_path} 当前分支：<{repo.active_branch.name}>, 远端：<{repo.remotes}>")
            for c in commits:
                print(f"  {c['date'][:10]} {c['hash'][:8]}  {c['message']}")
            print('*' * 30)
    print('🔗 可拷贝的文件夹')
    print('\n'.join(all_path))


if __name__ == "__main__":
    main()
