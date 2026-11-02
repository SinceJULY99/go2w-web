#!/usr/bin/env bash
# 在 Mac 上本地调试网页控制台，不连机器人。
#
#   web/dev/run.sh [地图.yaml]
#
# 跑的是真实的后端代码（web/server/go2w_web.py），ROS 和导航容器换成 web/dev/fake_ros 里的假模拟器：
# 加载真实地图，模拟各运行档位、Nav2、cmd_bridge、patrol.py，scan 按地图射线计算。
#   http://127.0.0.1:8080        ROS 模式（走真实的后端和 WebSocket 协议）
#   http://127.0.0.1:8080/?mock  纯前端模拟（不经过后端数据）
#
# 存图、录包写进数据副本 ~/.cache/go2w_web_data（第一次运行从仓库 data/maps 复制），不碰仓库里的 data/。
# 想从头再来：rm -rf ~/.cache/go2w_web_data
#
# 环境变量：PORT（默认 8080）、GO2W_FAKE_STAMP_RATE（/odom 时间戳走速倍数，默认 1.0）
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=${GO2W_WEB_VENV:-$HOME/.cache/go2w_web_venv}
if [ ! -x "$VENV/bin/python" ] || ! "$VENV/bin/python" -c 'import aiohttp, numpy' 2>/dev/null; then
  echo "第一次运行：在 $VENV 建 Python 环境（aiohttp、numpy）……"
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q aiohttp numpy
fi

DATA=${GO2W_DATA:-$HOME/.cache/go2w_web_data}
if [ ! -d "$DATA/maps" ]; then
  echo "准备数据副本：$DATA"
  mkdir -p "$DATA/maps" "$DATA/bags" "$DATA/logs"
  cp ../data/maps/*.yaml ../data/maps/*.pgm "$DATA/maps/"
fi

PORT=${PORT:-8080}
MAP=${1:-$DATA/maps/lab_20261021.yaml}
echo "地图：$MAP"
echo "ROS 模式（假机器人）：http://127.0.0.1:$PORT"
echo "纯前端模拟：        http://127.0.0.1:$PORT/?mock"
echo "按 Ctrl+C 停止"
exec env PYTHONPATH="$PWD/dev/fake_ros" GO2W_FAKE_MAP="$MAP" GO2W_DATA="$DATA" GO2W_FAKE_PROCS=1 \
  "$VENV/bin/python" server/go2w_web.py --port "$PORT"
