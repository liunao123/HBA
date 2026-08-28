#!/bin/bash
# 增量建图。全部参数在 yaml 里, 这个脚本只负责选哪份 yaml。
#
#   首次:   ./run_incr.sh params/full.yaml
#   增量:   ./run_incr.sh <上次的 pose_store>/params.yaml
#           ^ 直接用存档里那份, 前后参数必然一致 (out 也会沿用上次的, 要换就在下面覆盖)
#
# 输出目录**每次清空**(点云/las/dump 几十 GB, 都是一次性产物);
# 位姿存档目录**绝不清**(增量的全部状态: 位姿 + 约束 + g2o + 约束图, 几十 MB)。
set -u
YAML=${1:-params/full.yaml}
shift || true                       # 余下的都当命令行覆盖传下去
cd "$(dirname "$0")"
[ -f "$YAML" ] || { echo "参数文件不存在: $YAML"; exit 1; }

# 从 yaml 里取 out / pose_store, 免得在两处各写一遍路径而写歪
rd() { sed -n "s/^$1: *\"\\?\\([^\"]*\\)\"\\?$/\\1/p" "$YAML" | tail -1; }
OUT=$(rd out); STORE=$(rd pose_store)
# 命令行覆盖了 --out / --pose_store 的话, 以命令行为准
ARGS=("$@")
for i in "${!ARGS[@]}"; do
  case ${ARGS[$i]} in
    --out)        OUT=${ARGS[$((i+1))]} ;;
    --pose_store) STORE=${ARGS[$((i+1))]} ;;
  esac
done
[ -n "$OUT" ] && [ -n "$STORE" ] || { echo "yaml 里读不到 out / pose_store"; exit 1; }
case "$STORE" in "$OUT"/*) echo "!! pose_store 在 out 里面, out 每次清空会连存档一起删"; exit 1;; esac

echo "参数   $YAML ${ARGS[*]:-}"
echo "输出   $OUT      (清空)"
echo "存档   $STORE    (保留)"
rm -rf "$OUT"; mkdir -p "$OUT" "$STORE"
/usr/bin/time -v ./build/local_align --params "$YAML" "${ARGS[@]}" > "$OUT.log" 2>&1
echo "退出码 $?  ->  $OUT.log"
sed -n '/=== 参数 ===/,/^  写出/p' "$OUT.log"
