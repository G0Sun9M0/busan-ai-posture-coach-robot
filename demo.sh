#!/bin/bash
# ===== 바른자세 코치봇 데모 실행 =====
# 펌웨어는 이미 보드에 구워져 있음 → 모니터만 띄워 로그 보며 시연.
# 사용법:  bash ~/Desktop/0629_busan_ai/demo.sh     (종료: Ctrl + ])

source /Users/kosungmo/.espressif/tools/activate_idf_v5.5.4.sh >/dev/null
unset IDF_TARGET
cd /Users/kosungmo/Desktop/0629_busan_ai/firmware

PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "❌ 보드를 못 찾음 — USB 연결 확인"
  exit 1
fi
echo "✅ 포트: $PORT"
echo "▶ 모니터 시작. 센서를 앞으로 숙이면 [자세]/[AI 전송] 로그 + 음성이 나옵니다."
echo "  (종료: Ctrl + ])"
idf.py -p "$PORT" monitor
