---
name: commit
description: "检查 Git 仓库的当前改动，生成符合仓库风格的提交信息，完成暂存、提交和推送。仅在用户显式调用 $commit 时使用。"
---

# 提交 Git 改动

安全地检查、提交并推送当前项目的 Git 改动。

## 定位仓库

1. 如果用户指定了项目路径，优先使用该路径。
2. 否则从当前工作目录执行 `git rev-parse --show-toplevel`。
3. 如果当前目录不是 Git 仓库，在当前工作区内浅层查找 `.git`：
   - 只找到一个仓库时，使用该仓库。
   - 找到多个仓库时，列出仓库并让用户选择。
   - 没有找到仓库时，停止执行。
4. 记录仓库根目录，后续所有 Git 命令都针对该目录执行。
5. 读取该仓库适用的 `AGENTS.md`。

不要根据文件夹名称猜测仓库位置。

## 检查改动

执行并分析：

```bash
git status --short
git diff --stat
git diff
git diff --cached
git log --oneline -8
git branch --show-current
git remote -v   