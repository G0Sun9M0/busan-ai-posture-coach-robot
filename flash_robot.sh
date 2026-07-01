#!/bin/bash
# ESP32-S3 Xiaozhi 로봇 - 빌드된 펌웨어 플래시 + 모니터 런처 (macOS)
# 사용법:  bash ~/Desktop/0629_busan_ai/flash_robot.sh
#          (모니터 종료: Ctrl + ])

set -e

# 1) ESP-IDF 환경 활성화
source /Users/kosungmo/.espressif/tools/activate_idf_v5.5.4.sh >/dev/null

# 2) 충돌 유발 환경변수 제거 (esp32 잔재)
unset IDF_TARGET

# 3) 프로젝트 폴더로 이동
cd /Users/kosungmo/Desktop/0629_busan_ai/firmware

# 4) 연결된 ESP32-S3 포트 자동 탐지
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "❌ 보드를 못 찾았습니다. USB 연결을 확인하세요. (/dev/cu.usbmodem* 없음)"
  exit 1
fi
echo "✅ 포트 감지: $PORT"

# 5) 플래시 + 모니터
idf.py -p "$PORT" flash monitor
