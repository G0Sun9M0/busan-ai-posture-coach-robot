#!/bin/bash
# 바른자세 코치봇 - 기울기 센싱 유닛 (ZY-ESP32E / 클래식 ESP32) 플래시 런처
# 사용법:  bash ~/Desktop/0629_busan_ai/flash_sensor.sh
#          (모니터 종료: Ctrl + ])
# ※ 로봇(S3)은 /dev/cu.usbmodem*, 이 센서보드는 /dev/cu.usbserial* (또는 wchusbserial)

set -e
source /Users/kosungmo/.espressif/tools/activate_idf_v5.5.4.sh >/dev/null
unset IDF_TARGET
cd /Users/kosungmo/Desktop/0629_busan_ai/posture_sensor

# ZY-ESP32E는 CP2102/CH340 칩 → usbserial/wchusbserial 로 잡힘 (로봇 usbmodem 제외)
PORT=$(ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "❌ 센서 보드(ZY-ESP32E)를 못 찾았습니다."
  echo "   - USB 연결 확인"
  echo "   - 포트가 안 보이면 CP2102/CH340 드라이버 설치 필요할 수 있음"
  echo "   - 현재 보이는 포트: $(ls /dev/cu.* 2>/dev/null | tr '\n' ' ')"
  exit 1
fi
echo "✅ 센서 보드 포트 감지: $PORT"
idf.py -p "$PORT" flash monitor
