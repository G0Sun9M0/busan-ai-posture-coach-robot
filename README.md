# 바른자세 코치봇 — ESP32-S3 · Xiaozhi AI 4족 로봇

2026년 **부산 AI 교육**에서 만든 임베디드 AI 로봇 프로젝트.
센서가 사용자의 **숙인 자세**를 감지하면 **AI 음성**으로 자세를 교정해 주는 4족 로봇이다.

## 무엇을 배웠나
보드 점등부터 서보 보행·오디오·AI 연동까지 **하드웨어를 바닥부터 한 단계씩(bring-up)** 직접 올렸다.

- **ESP-IDF v5.5.4 · FreeRTOS** — `idf.py build/flash/monitor`, `app_main` 구조
- **I2C** — `driver/i2c`로 PCA9685(16채널 PWM) 직접 제어 → 서보 각도 명령
- **I2S** — 마이크 입력 / 스피커 출력
- **4족 보행** — 서보 시퀀스로 게이트 구현
- **Xiaozhi AI** — 자세 감지 → AI 음성 반응 (Windows 가이드를 macOS로 이식)

> 📓 **자세한 학습 기록**: [학습일지_바른자세코치봇.md](학습일지_바른자세코치봇.md)

## 폴더 구조
| 경로 | 내용 |
|---|---|
| `00_board_test` ~ `06_mic_reaction_test` | 단계별 하드웨어 브링업 (독립 ESP-IDF 프로젝트) |
| `firmware/` | Xiaozhi 오픈소스 펌웨어 이식본 |
| `demo.sh` · `flash_robot.sh` · `flash_sensor.sh` | 포트 자동 탐색 → 플래시 → 모니터 스크립트 |

## 빌드 / 실행
```bash
source ~/.espressif/tools/activate_idf_v5.5.4.sh
unset IDF_TARGET
cd 02_servo_test          # 예: 서보 테스트
idf.py -p /dev/cu.usbmodem* flash monitor
```

> `build/`·`managed_components/`는 저장소에서 제외했다. `idf.py reconfigure`로 의존성을 다시 받은 뒤 빌드하면 된다.

---
🤖 개발 방식: 이 프로젝트는 Claude Code와 협업(바이브 코딩)하며 만들었고, 그 과정과 배움을 학습일지에 정직하게 기록했다.
