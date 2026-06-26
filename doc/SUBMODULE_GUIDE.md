# Git Submodule 速查手册

> 本仓库（F1）使用 **薄集成仓库 + Git Submodule** 架构。
> 导航和控制是两个独立子仓库，分别由不同团队迭代。

## 仓库结构

```
weilai-robot/F1                    ← 你当前所在的集成仓库
├── navigation/                    ← submodule → weilai-robot/Humanoid_navigation
├── motion_control/                ← submodule → weilai-robot/Humanoid_motion
├── cmake/                         ← 公共 CMake 模块（集成仓库管理）
├── docker/                        ← 环境定义（集成仓库管理）
├── scripts/                     ← 统一构建/运行脚本（集成仓库管理）
│   ├── build_all.sh            ←   全量构建
│   ├── build.sh / build_nav.sh ←   单模块构建
│   ├── run_sim_nav.sh          ←   仿真导航启动
│   └── send_nav_goal.sh        ←   导航目标发送
└── doc/                           ← 跨模块文档
```

| 子仓库 | GitHub 地址 | 团队 |
|--------|------------|------|
| navigation | https://github.com/weilai-robot/Humanoid_navigation.git | 导航团队 |
| motion_control | https://github.com/weilai-robot/Humanoid_motion.git | 控制团队 |

---

## 日常操作速查

### 1. 首次克隆项目（含子模块）

```bash
# 方式一：克隆时自动拉取子模块
git clone --recursive https://github.com/weilai-robot/F1.git

# 方式二：如果已经 clone 了但忘了 --recursive
git clone https://github.com/weilai-robot/F1.git
cd F1
git submodule update --init --recursive
```

### 2. 拉取集成仓库最新代码（含子模块更新）

```bash
git pull origin main
git submodule update --init --recursive
```

> **注意**：`git pull` 只更新集成仓库自身文件，**不会自动更新子模块**。
> 必须紧跟 `git submodule update` 才能同步到集成仓库锁定的子模块版本。

### 3. 在子模块中开发代码

子模块就是普通的 Git 仓库，进入子模块目录后正常操作：

```bash
cd navigation/
git checkout devel          # 切到开发分支（不要在 detached HEAD 上工作！）
# ... 写代码、测试 ...
git add .
git commit -m "feat: your changes"
git push origin devel
```

> **⚠️ 关键规则**：始终在子模块目录内 `git checkout <branch>` 后再开发。
> 默认 clone 后子模块处于 **detached HEAD** 状态，直接 commit 会导致代码丢失。

### 4. 子模块代码推送后，更新集成仓库锁定版本

子模块 push 后，集成仓库记录的还是旧 commit。需要回集成仓库提交一次：

```bash
cd /path/to/F1              # 回到集成仓库根目录
git add navigation/         # 暂存子模块的新 commit hash
git commit -m "chore: bump navigation to latest (含XXX改动)"
git push origin main
```

> 这一步只是记录「集成仓库在哪个版本验证过哪个子模块版本」，不涉及代码拷贝。

### 5. 回退某个子模块到之前的版本

```bash
cd navigation/
git checkout <旧commit-hash>   # 切到想回退的版本
cd ..
git add navigation/
git commit -m "revert: rollback navigation to <version>"
```

> **这就是 submodule 架构的核心优势**：回退 navigation 不影响 motion_control。

### 6. 拉取子模块远程的最新代码（想要尝试最新版）

```bash
cd navigation/
git fetch origin
git checkout main
git pull origin main
cd ..
git add navigation/
git commit -m "chore: update navigation to remote latest"
```

---

## 各角色推荐工作流

### 导航团队开发者

```bash
# 1. 克隆集成仓库
git clone --recursive https://github.com/weilai-robot/F1.git
cd F1/navigation

# 2. 创建/切换到开发分支
git checkout -b devel  # 或 git checkout devel

# 3. 日常开发
# ... 修改代码 ...
git add . && git commit -m "..." && git push origin devel

# 4. 合并到 navigation main（PR 或直接 merge）
#    合并后回到集成仓库更新锁定版本
cd ..
git add navigation/
git commit -m "chore: bump navigation after PR merge"
git push origin main
```

### 控制团队开发者

同上，将 `navigation` 替换为 `motion_control`。

### 集成联调人员

```bash
# 1. 克隆并确认子模块版本
git clone --recursive https://github.com/weilai-robot/F1.git
cd F1
git submodule status    # 查看当前锁定的版本

# 2. 全量构建
scripts/build_all.sh

# 3. 如需测试某模块的最新开发版
cd navigation/ && git checkout devel && git pull && cd ..
# 测试通过后锁定
git add navigation/ && git commit -m "chore: pin navigation devel for integration test"
```

---

## 常见问题

### Q: 子模块目录是空的？

A: 克隆时没加 `--recursive`。执行：
```bash
git submodule update --init --recursive
```

### Q: git status 显示子模块有修改 (new commits)？

A: 这是正常的——子模块的 HEAD 和集成仓库锁定的版本不一致。如果是故意的，`git add` + `commit` 即可更新锁定版本。如果不是，`git submodule update` 回到锁定版本。

### Q: 在子模块里 git commit 报 "detached HEAD"？

A: 说明你忘了切分支。执行：
```bash
git checkout main  # 或 devel
# 然后重新开发
```

### Q: 两个人同时改了集成仓库的子模块版本，冲突了？

A: `.gitmodules` 本身不冲突（URL 不变），冲突的是子模块指针 hash。解决方法：
```bash
# 选择要保留的版本
git checkout --theirs navigation   # 或 --ours
git submodule update navigation
git add navigation/
git commit
```

---

## 分支策略建议

| 仓库 | 分支 | 用途 |
|------|------|------|
| Humanoid_navigation | `main` | 稳定发布版 |
| Humanoid_navigation | `devel` | 日常开发 |
| Humanoid_motion | `main` | 稳定发布版 |
| Humanoid_motion | `devel` | 日常开发 |
| F1（集成） | `main` | 锁定的稳定组合 |
| F1（集成） | `staging` | 联调中的组合（可选） |
