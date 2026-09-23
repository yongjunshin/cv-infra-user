# cv-infra-user (carter)

cv-infra **소비자 예시**. "로봇 SW 프로젝트가 cv-infra를 실제로 쓰면 저장소가 어떻게 생기나"를
보여주는 최소 형태 — 파일 4개와 워크플로 1개가 전부다.

검증하는 질문은 하나다: **"창고 통로에 잡동사니가 놓였을 때, 로봇이 아무것도 건드리지 않고
목표점까지 가는가?"** 케이스마다 로봇을 스폰 포즈로 텔레포트하고, 스폰→목표 직선 위에 창고
소품을 `obstacle_count`개 세운 뒤, 주행시키고, **닿았는지 / 도착했는지**를 기록한다.

PR을 열면 GitHub Actions가 플랫폼의 재사용 워크플로를 호출하고, 플랫폼은 GPU 워크스테이션의
Isaac Sim에서 아래 `verify/sim.py`를 **입력 조합마다 한 번씩** 돌린 뒤 `verify/oracle.py`의 판정을
PR에 Check · sticky 코멘트 · 아티팩트(케이스별 출력 zip + 시뮬 로그)로 돌려준다.

## 파일

| 파일 | 무엇 |
|---|---|
| `verify/sim.py` | **표준 Isaac standalone 실행 entrypoint.** 창고 씬(공식 ROS 2 내비게이션 샘플)을 열고, 로봇을 `--spawn_*`로 텔레포트하고, `--obstacle_*`대로 소품을 스폰하고, 파일 안의 ~60줄 컨트롤러로 `--goal_*`까지 몰면서 `verify/out/`에 **trajectory.csv · contacts.json · run.json** 세 개를 쓴다. 플랫폼은 이 파일을 import하지 않고 실행만 한다. |
| `verify/param_space.pict` | **입력 공간**(Microsoft PICT 문법). 축 이름 = argv 플래그: `goal_y: 3.0` → `--goal_y=3.0`. 플랫폼이 여기서 페어와이즈(k=2) 커버링 배열을 만들어 **15 케이스**를 뽑는다. |
| `verify/oracle.py` | **판정.** 시뮬 직후 같은 이미지·같은 argv로(GPU 없이) 돌아 위 세 파일을 읽고 평평한 JSON dict 한 줄을 stdout에 낸다. stdlib만 쓴다. |
| `verify/out/.gitkeep` | 출력 디렉토리를 **커밋된 상태로** 두기 위한 파일 — 아래 참고. |
| `.github/workflows/verify.yml` | 잡 하나(`uses: …@…`)와 `with:` 입력 8개. 이 저장소가 유지하는 통합 표면 전부. |

## 판정은 타입으로 말한다

`oracle.py`가 내는 dict는 **예약 키가 없다.** 값의 타입이 뜻을 정한다.

| 타입 | 뜻 |
|---|---|
| `bool` | **체크** — 케이스는 모든 bool이 true일 때 pass |
| 숫자 | **지표** — 커밋 간 baseline과 비교해 변화만 보고(게이트하지 않음) |
| `null` | 판정 불가 — 실패가 아니라 비율에서 제외 |
| 문자열 | 메모 |

| 키 | 타입 | 뜻 |
|---|---|---|
| `reached_goal` | 체크 | 예산 안에 목표점 0.3 m 이내로 들어왔나 |
| `collision_free` | 체크 | 로봇의 **어떤 바디도** 소품에 닿지 않았나 |
| `time_to_goal_s` | 지표/`null` | 도착까지 sim 초. 못 갔으면 `null`(실패는 `reached_goal`이 말한다) |
| `final_dist_m` | 지표 | 종료 시점의 목표점까지 거리 |
| `min_clearance_m` | 지표/`null` | 주행 중 가장 가까웠던 소품 표면까지 거리. 소품이 없으면 `null` |
| `path_len_m` | 지표/`null` | 실제 주행 경로 길이. `trajectory.csv`가 없으면 `null` |
| `note` | 메모 | 첫 접촉("wheel_left ↔ cardbox #0 at t=4.08s")과 그 케이스의 소품 목록 |

> ⚠ **체크 키는 "좋은 쪽"으로 이름 짓는다.** 플랫폼은 **모든 bool이 true**여야 pass로 본다. 그래서
> `collided`(닿았다)가 아니라 `collision_free`(안 닿았다)다 — 전자로 쓰면 로봇이 **박았을 때만
> 초록**인 게이트가 된다.

> ⚠ **exit code는 판정이 아니다.** `SimulationApp.close()`는 무슨 일이 있었든 프로세스를 status 0으로
> 끝내고, stock `python.sh`는 비0을 1로 뭉갠다. 그래서 `sim.py`의 종료 코드는 pass/fail을 실을 수
> 없고, 오직 "이 케이스가 ERROR였다"만 의미한다. pass/fail은 전부 `oracle.py`가 결정한다. 판정에
> 쓸 산출물은 전부 `close()` **이전에** 쓴다.

## 시뮬 이미지는 다이제스트로 고정한다

`sim_image`는 **필수 입력**이고, 값은 태그가 아니라 **다이제스트**여야 한다(플랫폼이 태그를 거부한다).
Isaac Sim은 메이저 사이에 Python API가 깨지고 `5.1.0` 태그는 같은 이름으로 다시 푸시될 수 있으니,
`verify/sim.py`가 실제로 맞춰 쓰인 그 이미지를 다이제스트로 못 박아야 CI가 "로컬에서 테스트한 그
환경"이 된다. 다이제스트를 읽는 법:

```bash
docker inspect --format '{{index .RepoDigests 0}}' nvcr.io/nvidia/isaac-sim:5.1.0
```

아래 로컬 `docker run`도 **워크플로의 `sim_image`와 같은 다이제스트**를 쓴다. 이미지를 올릴 때는
`.github/workflows/verify.yml`과 이 README를 **같이** 고친다 — 한쪽만 바뀌면 로컬에서 재현한 게
CI가 돌린 것과 다른 이미지가 된다.

## 로컬에서 같은 케이스 돌리기

CI가 하는 일과 같다 — 저장소 루트에서. 아래 아홉 개 플래그는 `verify/param_space.pict`가 펼쳐지는
15행 중 **한 행 그대로**다(통로를 6 m 올라가며 플라스틱 배럴 하나를 피하는 케이스):

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e CV_SEED=7 \
  -v "$PWD":/cv/checkout -w /cv/checkout \
  --entrypoint /bin/sh \
  nvcr.io/nvidia/isaac-sim:5.1.0@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54 \
  -lc 'exec "$0" "$@"' verify/sim.py \
  --sim_time_max=40 --spawn_x=-5.7 --spawn_y=-1.0 --spawn_yaw=1.5708 \
  --goal_x=-6.0 --goal_y=5.0 --obstacle_count=1 --obstacle_kind=barrel --obstacle_scale=1.0

python3 verify/oracle.py \
  --sim_time_max=40 --spawn_x=-5.7 --spawn_y=-1.0 --spawn_yaw=1.5708 \
  --goal_x=-6.0 --goal_y=5.0 --obstacle_count=1 --obstacle_kind=barrel --obstacle_scale=1.0
```

`ACCEPT_EULA`가 없으면 `sim.py`는 부팅 전에 거부한다(exit 3). 컨테이너에는 디스플레이가 없으므로
기본은 headless고, `--gui`는 데스크톱에 설치된 Isaac에서 볼 때만 쓴다(`./python.sh verify/sim.py
--gui --sim_time_max=40 …`). CI에서 GUI로 부팅하면 행이나 크래시로 끝난다.

## `verify/out/.gitkeep`이 필요한 이유

플랫폼은 체크아웃을 **읽기 전용**으로 마운트하고, 그 위 `sim_output_dir` 경로에만 케이스별 호스트
디렉토리를 읽기·쓰기로 덮어 씌운다. 그래서 그 경로는 **체크아웃에 디렉토리로 존재해야** 한다 —
git은 빈 디렉토리를 추적하지 않으므로 `.gitkeep`을 커밋해 둔다. 산출물 자체는 `.gitignore`가
막는다.

## 구동 경로와 "로봇 SW"에 대한 정직한 메모

**ROS 2 경로는 없다.** `sim.py`는 휠 조인트 속도를 직접 명령한다(`set_joint_velocities`). 이전 버전은
번들 `rclpy`로 `/cmd_vel`을 발행하는 경로를 먼저 시도했지만, CI 실측에서 **stock 이미지의 번들
브리지는 `setup_ros_env.sh` 없이는 뜨지 않는다**는 것이 확인됐고, 이 예시에는 ROS가 필요하지 않다.
씬에 들어 있는 ROS 2 OmniGraph들은 그대로 두되 아무도 구독하지 않은 채 논다.

휠 반지름 `0.14 m`와 트레드 `0.413 m`는 **공표값 추정이 아니라 씬 자신의 authored 값**이다 —
`carter_warehouse_navigation.usd` 안의 `DifferentialController` 노드에서 읽었다(실측 2026-09-23).
`(v, w)`를 좌·우 휠 속도로 바꾸는 데 이 두 값만 쓴다.

여기서 **검증 대상("로봇 SW")은 `sim.py` 안의 ~60줄 컨트롤러**다. 목표 방위각 P 제어 +
**소품 위치를 이미 아는 상태**에서의 반발 조향(지도 기반, 인식 없음)이 전부인 대역(stand-in)이다.
그래서:

- 케이스가 빨간 건 **그 컨트롤러가 실패한 것**이지 플랫폼이나 Isaac의 문제가 아니다. 1.0배 박스
  하나는 비켜 가도 1.5배 배럴 두 개는 못 피할 수 있고, 그 편차를 보는 게 이 검증의 목적이다.
- 실제 로봇 SW(nav2 등)를 붙이는 건 이 컨트롤러 블록을 들어내고 그 자리에 붙이는 일이다.
  입력 공간·오라클·수집 계약은 그대로 쓴다.
- 접촉 판정은 `ContactSensor`가 아니라 소품 프림에 건 **PhysX contact report**다 — 섀시보다 **휠이
  먼저 닿기** 때문에 섀시 센서는 박스를 놓쳤다(실측). 어떤 바디가 무엇에 닿았는지는
  `contacts.json`과 `note`에 이름으로 남는다.
