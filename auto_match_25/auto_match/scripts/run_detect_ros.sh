#!/usr/bin/env bash
# detect_ros 启动包装器：负责挑一个「装了推理后端」的 python 再 exec 节点。
#
# 解释器优先级：
#   1. $AUTO_MATCH_VENV_PYTHON（显式指定，便于 A/B 环境切换）
#   2. ~/yolo_venv/bin/python（若存在）
#   3. 系统 python3
#
# 10.24.170.0 实测：系统 python3(3.8.10) 已装 ultralytics 8.0.143 + yolov5 7.0.13 + torch 2.0.1，
# 并不需要 yolo_venv。以前这里写死 ~/yolo_venv 且不存在就 exit 2，导致视觉链路整体起不来。
set -euo pipefail

if [[ -n "${AUTO_MATCH_VENV_PYTHON:-}" ]]; then
  PYTHON="${AUTO_MATCH_VENV_PYTHON}"
elif [[ -x "${HOME}/yolo_venv/bin/python" ]]; then
  PYTHON="${HOME}/yolo_venv/bin/python"
else
  PYTHON="$(command -v python3 || true)"
fi

if [[ -z "${PYTHON}" || ! -x "${PYTHON}" ]]; then
  echo "[detect_ros] 找不到可用的 python3（已尝试 \$AUTO_MATCH_VENV_PYTHON、~/yolo_venv、系统 python3）" >&2
  exit 2
fi

# 后端自检：至少要有一个能 import，否则节点起来也是白起
if ! "${PYTHON}" - <<'PY'
import importlib.util
import sys

found = [m for m in ("ultralytics", "yolov5") if importlib.util.find_spec(m) is not None]
sys.stderr.write("[detect_ros] 解释器: %s\n" % sys.executable)
sys.stderr.write("[detect_ros] 可用后端: %s\n" % (found or "无"))
sys.exit(0 if found else 3)
PY
then
  echo "[detect_ros] ${PYTHON} 里 ultralytics 和 yolov5 都没有，请先安装 requirements-vision.txt" >&2
  exit 3
fi

PACKAGE_PATH="$(rospack find auto_match)"
# 关掉 Python 输出缓冲：roslaunch 把 stdout 重定向到文件时，日志默认是块缓冲，
# 现场排查会看到"日志晚到"甚至丢失，这里强制行缓冲。
export PYTHONUNBUFFERED=1
exec "${PYTHON}" "${PACKAGE_PATH}/nodes/detect_ros.py" "$@"
