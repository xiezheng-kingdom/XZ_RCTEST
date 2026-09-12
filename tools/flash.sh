#!/usr/bin/env bash
# 编译 + 烧录 RCTEST_VOFA 到 STM32H743VIT6 (ST-Link / SWD)
#
# 用法:
#   tools/flash.sh              # 增量编译 Debug 并烧录
#   tools/flash.sh --no-build   # 只烧录已有的 build/Debug/RCTEST_VOFA.elf
#   tools/flash.sh --release    # 用 Release 预设
#   tools/flash.sh --erase      # 烧录前整片擦除
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PRESET="Debug"
BUILD=1
ERASE=""

for arg in "$@"; do
  case "$arg" in
    --no-build) BUILD=0 ;;
    --release)  PRESET="Release" ;;
    --erase)    ERASE="-e all" ;;
    *) echo "未知参数: $arg" >&2; exit 2 ;;
  esac
done

bundle() {
  # CUBE_BUNDLE_PATH 默认指向 STM32CubeCLT / STM32 VS Code 扩展下载的 bundles
  local base="${CUBE_BUNDLE_PATH:-${LOCALAPPDATA:-$HOME/AppData/Local}/stm32cube/bundles}"
  # Windows 路径的反斜杠 -> 正斜杠（模式必须用引号包住反斜杠，否则匹配不到）
  base="${base//'\'/'/'}"
  printf '%s\n' "$base"
}

BUNDLES="$(bundle)"
TOOLCHAIN_BIN="$BUNDLES/gnu-tools-for-stm32/14.3.1+st.2/bin"
NINJA_BIN="$BUNDLES/ninja/1.13.2+st.1/bin"
PROGRAMMER="$(ls -d "$BUNDLES"/programmer/*/bin/STM32_Programmer_CLI.exe 2>/dev/null | sort -V | tail -1)"

[ -x "$PROGRAMMER" ] || { echo "找不到 STM32_Programmer_CLI.exe（在 $BUNDLES/programmer 下）" >&2; exit 1; }

if [ "$BUILD" -eq 1 ]; then
  export PATH="$TOOLCHAIN_BIN:$NINJA_BIN:$PATH"
  echo "==> 配置 ($PRESET)"
  cmake --preset "$PRESET" -S "$ROOT" --log-level=WARNING
  echo "==> 编译 ($PRESET)"
  cmake --build --preset "$PRESET" -j
fi

ELF="$ROOT/build/$PRESET/RCTEST_VOFA.elf"
[ -f "$ELF" ] || { echo "找不到固件: $ELF" >&2; exit 1; }

# STM32_Programmer_CLI 是原生 Windows 程序，转换成 Windows 路径更稳妥
if command -v cygpath >/dev/null 2>&1; then
  ELF="$(cygpath -w "$ELF")"
fi

echo "==> 烧录 $ELF"
"$PROGRAMMER" -c port=SWD mode=UR reset=HWrst $ERASE -w "$ELF" -v -rst
echo "==> 完成"
