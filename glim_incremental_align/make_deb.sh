#!/bin/bash
# =============================================================================
# 一键出 deb: 编译 -> 收集全部依赖 -> 打包。
#
#   ./make_deb.sh              版本号自动带时间戳
#   ./make_deb.sh 1.2.0        自己指定版本号
#   ./make_deb.sh --no-build   跳过编译, 直接拿现有的 build/local_align 打包
#
# 出来的包**自带全部依赖** (gtsam / gtsam_points / OpenCV / TBB 那些都在里面),
# 目标机器只要 glibc >= 2.35 (Ubuntu 22.04+), 不用装任何开发库。
# 具体为什么必须捆绑、RPATH 那个坑, 见 packaging/build_deb.sh 顶部的注释。
# =============================================================================
set -e

ROOT=$(cd "$(dirname "$0")" && pwd)
cd "$ROOT"

BUILD=1
VER=""
for a in "$@"; do
  case "$a" in
    --no-build) BUILD=0 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) VER=$a ;;
  esac
done

# 版本号默认带**时间戳**: 每次跑都是一个新版本, dpkg 能直接 -i 覆盖装, 也看得出哪个新。
#
# 再按 git 状态加后缀, 这样拿到一个 deb 能知道它对应哪份代码:
#   (无后缀)     工作区干净, 对应 +<commit> 那个提交
#   +dirty       有未提交的改动
#   +untracked   **整个目录都没进 git** —— 现在就是这种情况 (git status 显示 "?? glim_incremental_align/")
# 为什么不能用 `git diff --quiet HEAD`: 对 untracked 的文件它**永远返回 0**(无改动),
# 于是这个判断静默失效, 每个包都显示干净。要用 git status --porcelain 才看得到 untracked。
if [ -z "$VER" ]; then
  VER="1.0.$(date +%y%m%d%H%M)"
  if git -C "$ROOT" rev-parse --git-dir >/dev/null 2>&1; then
    st=$(git -C "$ROOT" status --porcelain -- . 2>/dev/null)
    if [ -z "$st" ]; then
      h=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null) && [ -n "$h" ] && VER="$VER+$h"
    elif echo "$st" | grep -q '^??'; then
      VER="$VER+untracked"
    else
      VER="$VER+dirty"
    fi
  fi
fi

# ---- 1) 编译 ----
if [ "$BUILD" = 1 ]; then
  echo "=== 编译 ==="
  mkdir -p build
  [ -f build/CMakeCache.txt ] || (cd build && cmake .. >/dev/null)
  (cd build && make local_align -j"$(nproc)" 2>&1 | tail -3)
else
  echo "=== 跳过编译 (--no-build), 用现有的 build/local_align ==="
  [ -x build/local_align ] || { echo "!! build/local_align 不存在, 去掉 --no-build"; exit 1; }
fi

# ---- 2) patchelf ----
# 打包时必须改 RPATH (见 packaging/build_deb.sh), 而这台机器上 apt 装不了(要 sudo),
# 所以走 pip 的 wheel —— 它自带二进制, 不编译。
if ! command -v patchelf >/dev/null && [ ! -x "$HOME/.local/bin/patchelf" ]; then
  echo "=== 装 patchelf (打包要用它改 RPATH) ==="
  pip install --quiet --user patchelf || { echo "!! patchelf 装不上, 手动: pip install patchelf"; exit 1; }
fi
export PATH="$HOME/.local/bin:$PATH"

# ---- 3) 打包 ----
./packaging/build_deb.sh "$VER"
