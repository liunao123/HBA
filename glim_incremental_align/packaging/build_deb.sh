#!/bin/bash
# =============================================================================
# 把 local_align 打成一个**自带全部依赖**的 deb。
#
# 为什么必须捆绑而不是声明 apt 依赖: 四类依赖在目标机器上装不出来 ——
#   libgtsam.so.4        编译机上是 **build tree, 从未 install**, apt 里也没有 4.3a0
#   libgtsam_points.so.1 自编 (koide3/gtsam_points v1.2.2)
#   libopencv_*.so.406   自编装在 /usr/local/lib, 不是 apt 的 4.5.4
#   libtbb.so.2          来自 libtbb2 (2020.3); Ubuntu 22.04 默认只有 libtbb.so.12 (2021.5),
#                        而 gtsam 是按 GTSAM_DEFAULT_ALLOCATOR=TBB 编的, 换版本会崩
# 所以除 glibc 之外的**所有** .so 都复制进包里, 靠 wrapper 的 LD_LIBRARY_PATH 加载。
# (不用 patchelf 改 RPATH: 编译机上没装它, 而 wrapper 效果相同且看得见。)
#
# 唯一的硬性前提是 **glibc**: 包里的 .so 都是在 glibc 2.35 (Ubuntu 22.04) 上编的,
# 目标机器的 glibc 不能更老。control 里的 libc6 (>= 2.35) 就是这条, dpkg 会自己拦。
#
# 用法: ./packaging/build_deb.sh [版本号]
# =============================================================================
set -e

VER=${1:-1.0.0}
PKG=glim-incremental-align
ROOT=$(cd "$(dirname "$0")/.." && pwd)
BIN=$ROOT/build/local_align
PREFIX=/opt/$PKG
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

[ -x "$BIN" ] || { echo "!! 先编译: cd $ROOT/build && cmake .. && make -j8"; exit 1; }

D=$WORK/$PKG
mkdir -p "$D/DEBIAN" "$D$PREFIX/libexec" "$D$PREFIX/lib" "$D$PREFIX/share" "$D/usr/bin"

# ---- 1) 主程序 ----
cp "$BIN" "$D$PREFIX/libexec/local_align"
chmod 755 "$D$PREFIX/libexec/local_align"

# ---- 2) 收集依赖 ----
# 排除的只有 glibc 自身和内核提供的那几个: 它们必须用目标机器的版本, 换成编译机的会崩。
# libstdc++ / libgcc_s / libgomp **要带** —— 它们向后兼容, 而目标机器的 gcc 可能更老,
# 缺一个 GLIBCXX_3.4.30 就跑不起来。
EXCLUDE='^(linux-vdso|ld-linux-x86-64|libc|libm|libpthread|libdl|librt)\.so'
echo "=== 收集依赖 ==="
n=0
while read -r soname path; do
  [ -n "$path" ] && [ -f "$path" ] || continue
  echo "$soname" | grep -qE "$EXCLUDE" && continue
  # 按 **soname** 命名 + 解引用符号链接: loader 找的是 soname, 而 ldd 给的常是真实文件名
  cp -L "$path" "$D$PREFIX/lib/$soname"
  n=$((n + 1))
done < <(ldd "$BIN" | sed -n 's/^\s*\([^ ]*\) => \(.*\) (0x[0-9a-f]*)$/\1 \2/p')
echo "  复制了 $n 个 .so ($(du -sh "$D$PREFIX/lib" | cut -f1))"

# 二阶依赖: 捆绑的库自己还可能依赖别的东西, 而那些在目标机器上未必有。
# 反复扫直到不再新增 —— 一轮不够 (opencv -> IlmImf -> Iex ...)。
for pass in 1 2 3 4 5; do
  add=0
  for so in "$D$PREFIX/lib"/*.so*; do
    while read -r soname path; do
      [ -n "$path" ] && [ -f "$path" ] || continue
      echo "$soname" | grep -qE "$EXCLUDE" && continue
      [ -e "$D$PREFIX/lib/$soname" ] && continue
      cp -L "$path" "$D$PREFIX/lib/$soname"
      add=$((add + 1))
    done < <(ldd "$so" 2>/dev/null | sed -n 's/^\s*\([^ ]*\) => \(.*\) (0x[0-9a-f]*)$/\1 \2/p')
  done
  echo "  第 $pass 轮二阶依赖: 新增 $add 个"
  [ "$add" -eq 0 ] && break
done
chmod 644 "$D$PREFIX/lib"/*.so*

# ---- 2b) 改 RPATH 成相对路径 ($ORIGIN) ----
#
# **这一步不能省**。原始二进制带的是老式 DT_RPATH:
#   /mnt/nvme0n1p2/gtsam_map/gtsam/build/gtsam:...:/usr/local/lib
# 而 DT_RPATH **优先于 LD_LIBRARY_PATH** —— 光靠 wrapper 设环境变量挡不住它。
# 在目标机器上前几个路径不存在还好(会往后找), 但 `/usr/local/lib` 很可能**存在**:
# 那台机器要是自己装过 OpenCV/gtsam 到 /usr/local, 就会优先加载它们的版本,
# 而本包的 gtsam 是按 GTSAM_DEFAULT_ALLOCATOR=TBB + 特定 ABI 编的, 混用直接崩。
# 捆绑的库自己也带 RPATH (6 个 libopencv_*.so.406 都指着 /usr/local/lib), 同样要改。
#
# 必须用 **DT_RPATH** (--force-rpath), 不能用 RUNPATH。
# 搜索优先级: DT_RPATH > LD_LIBRARY_PATH > DT_RUNPATH > ld.so.cache
# 第一版用了 RUNPATH, 想着"低于 LD_LIBRARY_PATH, 用户还能覆盖来调试" —— 实测直接翻车:
# 这台机器的 shell 里有 ROS + CUDA 的 LD_LIBRARY_PATH, loader 于是先搜
#   /opt/ros/noetic/lib : /usr/local/cuda-11.8/lib64 : /usr/lib/x86_64-linux-gnu : /usr/local/lib
# **连 $ORIGIN/../lib 都没试**, 在 /usr/local/lib 撞上一个手工 make install 的
# libgtsam.so.4 -> 4.1.1 就用了它, 而包里是 4.3a0 -> libgtsam_points 起不来:
#   undefined symbol: _ZNK5gtsam15NonlinearFactor5errorERKNS_12HybridValuesE
# 装了 ROS 的机器几乎总有 LD_LIBRARY_PATH, 所以这不是边缘情况而是常态。
# 自包含的包必须让自己的库赢, 用 RPATH。
PATCHELF=$(command -v patchelf || echo "$HOME/.local/bin/patchelf")
[ -x "$PATCHELF" ] || { echo "!! 需要 patchelf: pip install patchelf"; exit 1; }
"$PATCHELF" --set-rpath '$ORIGIN/../lib' --force-rpath "$D$PREFIX/libexec/local_align"
for so in "$D$PREFIX/lib"/*.so*; do
  "$PATCHELF" --set-rpath '$ORIGIN' --force-rpath "$so" 2>/dev/null || true
done
chmod 644 "$D$PREFIX/lib"/*.so*
# 自检: 包里**不该**再出现编译机的绝对路径
bad=$(for f in "$D$PREFIX/libexec/local_align" "$D$PREFIX/lib"/*.so*; do
        readelf -d "$f" 2>/dev/null | grep -E "RPATH|RUNPATH" |
          grep -E "/mnt/|/usr/local/" | sed "s|^|  $(basename $f): |"
      done)
if [ -n "$bad" ]; then
  echo "!! 还有指向编译机的库搜索路径, 目标机器上会加载错版本:"
  echo "$bad"
  exit 1
fi
if readelf -d "$D$PREFIX/libexec/local_align" | grep -q RUNPATH; then
  echo "!! 主程序是 RUNPATH 而不是 RPATH —— 目标机器上的 LD_LIBRARY_PATH 会把它挤掉"
  exit 1
fi
echo "  RPATH 已全部改为 \$ORIGIN 且是 DT_RPATH (自检通过: 无 /mnt 或 /usr/local 残留)"

# ---- 3) 参数模板 + 文档 ----
cp -r "$ROOT/params" "$D$PREFIX/share/params"
[ -f "$ROOT/README.md" ] && cp "$ROOT/README.md" "$D$PREFIX/share/"

# ---- 4) wrapper ----
# 为什么用 wrapper 而不是 RPATH: 编译机没有 patchelf, 而这样做 LD_LIBRARY_PATH 是**可见的**
# —— 出问题时用户自己就能看出程序在哪儿找库。只对本进程生效, 不污染 shell。
cat > "$D$PREFIX/libexec/run.sh" <<'EOF'
#!/bin/sh
# 包内的库优先。追加而不是覆盖调用者的 LD_LIBRARY_PATH, 但放在**最前面** ——
# 目标机器上可能有版本不同的同名库 (尤其 libtbb / libopencv), 必须用包里这套。
P=/opt/glim-incremental-align
LD_LIBRARY_PATH="$P/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH
exec "$P/libexec/local_align" "$@"
EOF
chmod 755 "$D$PREFIX/libexec/run.sh"
ln -sf "$PREFIX/libexec/run.sh" "$D/usr/bin/local_align"

# ---- 5) DEBIAN/control ----
SIZE=$(du -sk "$D" | cut -f1)
cat > "$D/DEBIAN/control" <<EOF
Package: $PKG
Version: $VER
Section: science
Priority: optional
Architecture: amd64
Depends: libc6 (>= 2.35)
Installed-Size: $SIZE
Maintainer: liunao <liu123nao@gmail.com>
Description: 多 session LiDAR 点云增量建图一致性优化 (local_align)
 把多批次采集的点云统一到同一坐标系, 目标是肉眼无重影。
 同 session 内做分窗 BA + 回环, 跨 session 用 scan-to-submap 建约束,
 最后所有位姿一起联合优化; 位姿/约束存档, 新数据来了只算新的那部分。
 .
 本包**自带全部依赖** (gtsam 4.3a0 / gtsam_points 1.2.2 / OpenCV 4.6 /
 Ceres(静态) / TBB 2020.3 等), 目标机器只需要 glibc >= 2.35 (Ubuntu 22.04+),
 不必安装任何开发库。
EOF

cat > "$D/DEBIAN/postinst" <<EOF
#!/bin/sh
set -e
echo ""
echo "  local_align 已安装。参数模板: $PREFIX/share/params/main.yaml"
echo ""
echo "  先复制一份出来改数据路径 (root / out / pose_store 三项是绝对路径):"
echo "      cp $PREFIX/share/params/main.yaml ~/my.yaml"
echo "      local_align --params ~/my.yaml --list_done   # 先确认能扫到 session"
echo "      local_align --params ~/my.yaml               # 正式跑"
echo ""
echo "  全部参数说明: local_align --help    详细文档: $PREFIX/share/README.md"
echo ""
EOF
chmod 755 "$D/DEBIAN/postinst"

# ---- 6) 打包 ----
OUT=$ROOT/${PKG}_${VER}_amd64.deb
fakeroot dpkg-deb --build "$D" "$OUT" >/dev/null
echo
echo "=== 打好了 ==="
ls -lh "$OUT" | awk '{print "  "$9"  "$5}'
echo "  安装:   sudo dpkg -i $(basename "$OUT")"
echo "  卸载:   sudo dpkg -r $PKG"
