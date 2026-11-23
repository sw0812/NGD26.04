# QGroundControl + 조종기(라디오마스터 박서) 설정 가이드

> **중요**: 이 가이드는 원래 Jetson(aarch64, Ubuntu 20.04) 전용으로 만들었던 걸
> **x86_64 새 컴퓨터용으로 고친 버전**임. 이 USB/`~/.local/bin/`에 있는
> `qgc-docker.sh`는 **aarch64 전용**(AppImage 파일명이 `QGroundControl-aarch64.AppImage`로
> 고정됨)이라 x86_64에서는 그대로 못 씀 — 아래 순서대로 새로 확인할 것.

## 0단계: 조종기가 OS에 보이는지부터 확인 (가장 먼저, 매번)

```bash
ls /dev/input/by-id/ | grep -i radiomaster   # 이게 나와야 함 (예: ...-event-joystick)
lsusb -t                                      # Human Interface Device 쪽에 조종기가 걸려있는지
```

안 보이면:
- 조종기(박서) **본체 전원이 켜져 있는지** 확인 — USB 꽂아도 기기 자체가 꺼져있으면 충전만 되고
  호스트엔 아예 안 잡힘.
- 전원 켜져 있으면, **본체 USB 모드가 Joystick/HID로 선택돼 있는지** 확인 — EdgeTX 계열 기기는
  USB 연결 시 모드 선택 메뉴(조이스틱 / USB 저장소 / 펌웨어 업데이트)가 뜨는 경우가 있음.
- 케이블이 충전전용(데이터선 없음)인 경우도 있음 — 다른 케이블/포트로도 시도.

여기서 안 나오면 아래 1단계부터는 의미 없음.

## 1단계 (x86_64 새 컴퓨터): 먼저 도커 없이 네이티브로 시도

x86_64는 Jetson과 달리 **glibc 버전 문제가 없을 가능성이 높음** — 새 컴퓨터가 비교적
최근 Ubuntu/Debian이면 QGC AppImage가 그냥 네이티브로 뜰 수 있음. 도커부터 쓰지 말고
먼저 이것부터 확인:

```bash
ldd --version | head -1          # glibc 버전 확인
```

- **glibc 2.38 이상**이면 → QGC 공식 다운로드 페이지에서 **x86_64용** AppImage
  (`QGroundControl.AppImage`, aarch64용 아님)를 받아서 그냥 실행:
  ```bash
  chmod +x QGroundControl.AppImage
  ./QGroundControl.AppImage
  ```
  이게 되면 도커는 아예 필요 없음 — 2단계(SDL 매핑)만 적용하면 됨.

- **glibc 2.38 미만**(오래된 배포판)이면 → `qgc-docker.sh`를 참고해서 x86_64 버전으로
  바꿔서 써야 함: 스크립트 안의 `ubuntu:24.04` 베이스 이미지는 그대로 쓰면 되고,
  `APPIMAGE` 변수만 x86_64용 AppImage 경로로 바꾸면 됨. `--device-cgroup-rule='c 13:* rmw'`
  (조이스틱 `/dev/input` 권한) 부분은 아키텍처 상관없이 동일하게 필요함.

## 2단계: SDL 조종기 매핑 문제 (아키텍처 무관, x86에서도 재발 가능)

라디오마스터 박서가 **SDL3 게임패드 매핑 DB 자체에 없어서** 생기는 문제라 x86_64에서도
똑같이 재발할 수 있음:

**증상**: QGC에서 조종기는 잡히는데 스로틀 캘리브레이션이 끝까지 안 움직이고 중간값
근처에 고정됨 → SDL이 스로틀 축(axis2)을 "트리거"(0~1 편측 정규화)로 잘못 추측해서 생김.

**해결**: QGC 실행 전에 환경변수로 축 매핑을 직접 강제 (leftx/lefty/rightx/righty =
대칭 -1~1 정규화, 핵심은 axis2→lefty).

조종기 채널은 **AETR, Mode 2** (CH1 에일러론=a0, CH2 엘리베이터=a1, CH3 스로틀=a2,
CH4 러더=a3) 이므로 스틱 위치 기준으로 아래처럼 매핑해야 함:

```bash
export SDL_GAMECONTROLLERCONFIG="0300c19b09120000544f000011010000,OpenTX Radiomaster Boxer Joystick,leftx:a3,lefty:a2,rightx:a0,righty:a1,platform:Linux,"
```

> ⚠ 예전(Jetson) 문서 값 `leftx:a0,...,rightx:a3`는 롤(a0)이 왼쪽 스틱으로 가서
> 롤↔요가 뒤바뀜. **2026-11-10 머신 B(x86_64)에서 위 값으로 수정·검증함.**

QGC 축 번호는 게임패드 순서(0=leftx, 1=lefty, 2=rightx, 3=righty)이고,
`~/.config/QGroundControl/QGroundControl.ini`의 축 기능은
0→요, 1→스로틀, 2→롤, 3→피치 (`throttleModeCenterZero=false`).

## 2-1단계: 스로틀 맨 아래가 슬라이더 끝까지 안 내려가는 문제 (머신 B에서 확인)

**증상**: 매핑을 고친 뒤에도 Joystick 탭에서 스로틀을 맨 아래로 내려도 슬라이더가
왼쪽 끝까지 안 감 (예전 증상: 바닥 50 / 중앙 75 / 맨위 100).

**원인**: 박서가 스로틀 맨 아래를 0이 아니라 **256**(범위 0~2047)으로 보냄.
QGC Joystick 탭 슬라이더는 보정값이 아니라 **원시값**을 그대로 그리므로
QGC ini의 min/max를 아무리 고쳐도 안 바뀜.

**해결**: `boxer_abs_fix.py`가 evdev(EVIOCSABS)로 스로틀 축(ABS_Z) 최솟값을 256으로
바꿔줌. 조종기를 다시 꽂으면 초기화되므로 **QGC 실행할 때마다** 먼저 돌려야 함.

## 실행 방법 (머신 B, x86_64): 반드시 `qgc.sh`로

위 2단계 + 2-1단계를 묶은 스크립트. 이 폴더의 `qgc-native-machineB.sh`가
`~/QGroundControl/qgc.sh` 사본이고, `boxer_abs_fix.py`를 같은 폴더(`~/QGroundControl/`)에 둘 것.

```bash
~/QGroundControl/qgc.sh
```

AppImage를 직접 실행하면 매핑/스로틀 보정이 둘 다 빠짐.

**확인 방법**: 스로틀 맨 아래·나머지 중앙 상태에서 QGC Joystick 탭이
Throttle 슬라이더 왼쪽 끝, Pitch/Roll/Yaw 중앙이면 정상
(`qgc_joystick_tab_verified_20261110.png` 참고).

**주의**:
- QGC 설정에서 가상 조이스틱(`virtualJoystick`)은 끄고, 박서를 기체 1번에 활성화해 둘 것.
- **Joystick 설정 화면이 열려 있는 동안엔 QGC가 기체로 조종 입력을 안 보냄** (정상).
  Fly 화면으로 나가야 PX4에 들어감.
- 스로틀 맨 위(2047 도달 여부)와 각 채널 방향(반전)은 아직 미확인.

GUID(`0300c19b...`)는 디바이스 고유값이라 **새 컴퓨터에서도 같은 라디오마스터 박서를
쓰면 그대로 유효함**. 다른 조종기로 바꾸면 GUID가 달라지므로, 먼저 환경변수 없이
떠보고 캘리브레이션이 중간값에 고정될 때만 새 GUID로 다시 매핑할 것
(GUID 확인: `sdl2-jstest --list` 또는 QGC 로그에 찍히는 조종기 이름으로 유추).

## 도커를 쓰게 될 경우 (x86_64 기준)

```bash
docker run -d --name qgc_container --network host \
    --device-cgroup-rule='c 13:* rmw' \
    -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev \
    -v /path/to/QGroundControl.AppImage:/QGroundControl.AppImage:ro \
    ubuntu:24.04 sleep infinity

docker exec -u root $CONTAINER bash -c '
    apt-get update -qq && apt-get install -y -qq \
        libfuse2t64 libxcb-cursor0 libxkbcommon-x11-0 libgl1 libegl1 \
        libsm6 libice6 libfontconfig1 libnss3 libasound2t64
'

docker exec -e DISPLAY=$DISPLAY -e QT_QPA_PLATFORM=xcb \
    -e SDL_GAMECONTROLLERCONFIG="$SDL_GAMECONTROLLERCONFIG" \
    $CONTAINER /QGroundControl.AppImage --appimage-extract-and-run
```

`--device-cgroup-rule='c 13:* rmw'`가 없으면 컨테이너 안에서 조이스틱이 `EPERM`으로
전혀 안 읽힘 — 이건 Jetson에서 실제로 겪었던 문제라 x86에서도 도커 쓸 거면 꼭 넣을 것.

## PX4 쪽 설정 (QGC/조이스틱과는 별개)

QGC에서 조종기가 잡혀도, PX4가 그 입력을 실제로 쓰는지는 airframe 파라미터
`COM_RC_IN_MODE`가 결정함:
- `0` = RC만, `1` = 조이스틱만, `3` = RC+조이스틱 둘 다, `4` = 둘 다 무시(오프보드 전용)
- `tarot680pro` gz-sim 포팅(`6022_gz_tarot680pro`)은 현재 `1`(조이스틱 전용)로 설정돼 있음
  (2026-11-10 사용자 확인). Classic(`6021`)은 따로 설정 안 돼 있으니 필요하면 추가.

## 원본(Jetson/aarch64) 참고용 스크립트: `qgc-docker.sh`

이 파일 옆에 있는 `qgc-docker.sh`는 Jetson 전용으로 짰던 원본 그대로 둠 (AppImage
경로 고정, GUID는 동일, SDL 매핑은 예전 값 `leftx:a0/rightx:a3`이라 롤↔요 뒤바뀜) — x86 새 컴퓨터에서 그대로 복사해 쓰지 말고,
위 가이드대로 `APPIMAGE` 경로만 x86_64 버전으로 바꿔서 참고용으로만 쓸 것.
