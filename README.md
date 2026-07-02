# 바른자세 코치봇 — ESP32-S3 · Xiaozhi AI 자세 코치 로봇

2026년 **부산 AI 교육**에서 만든 임베디드 AI 로봇 프로젝트.
**기울기 센서(MPU6050)**로 사용자의 **숙인 자세**를 감지하면 **AI 음성**으로 자세를 교정해 준다.

> ⚠️ **보행 기능은 없다.** 원래 실습 키트는 4족 보행 로봇(서보 기반)이었지만,
> "자세 코치"에는 걷는 기능이 불필요해서 **서보모터를 제거하고 보행 하드웨어·코드를 데드로 정리**했다.
> (남길 것과 버릴 것을 구분한 설계 판단이 이 프로젝트의 핵심.)

## 무엇을 배웠나
하드웨어를 바닥부터 한 단계씩(bring-up) 올리고, 목적에 맞게 불필요한 부분을 걷어냈다.

- **ESP-IDF v5.5.4 · FreeRTOS** — `idf.py build/flash/monitor`, `app_main` 구조
- **I2S 오디오** — INMP441 마이크 입력 · MAX98357A 스피커 출력 (`I2S_NUM_0` 번갈아 사용)
- **I2C · MPU6050** — 자이로/가속도 IMU를 I2C로 읽어 자세(앞으로 숙임) 감지
- **Xiaozhi AI** — 자세 이벤트 → AI 음성 반응 (Windows 가이드를 macOS로 이식)
- **서보(PCA9685) · 4족 보행** — 개념 학습·실습(`02`,`03`)했으나 **최종 제거**(보행 불필요)

> 📓 **자세한 학습 기록**: [학습일지_바른자세코치봇.md](학습일지_바른자세코치봇.md)

## 폴더 구조
| 경로 | 내용 | 최종 사용 |
|---|---|---|
| `00_board_test` | FreeRTOS 보드 생존 확인 | ✅ |
| `01_oled_test` | I2C OLED 출력 | 선택 |
| `02_servo_test` · `03_walk_test` | 서보·4족 보행 (키트 원본) | ❌ 제거 |
| `04_speaker_test` · `05_mic_test` · `06_mic_reaction_test` | I2S 스피커·마이크·반응 | ✅ |
| `firmware/` | Xiaozhi 오픈소스 펌웨어 이식본 | |
| `demo.sh` · `flash_*.sh` | 포트 자동 탐색 → 플래시 → 모니터 | |

## 빌드 / 실행
```bash
source ~/.espressif/tools/activate_idf_v5.5.4.sh
unset IDF_TARGET
cd 06_mic_reaction_test          # 예: 마이크 반응 테스트
idf.py -p /dev/cu.usbmodem* flash monitor
```

> `build/`·`managed_components/`는 저장소에서 제외했다. `idf.py reconfigure`로 의존성을 다시 받은 뒤 빌드하면 된다.

---
🤖 개발 방식: 이 프로젝트는 Claude Code와 협업(바이브 코딩)하며 만들었고, 그 과정과 배움을 학습일지에 정직하게 기록했다.
