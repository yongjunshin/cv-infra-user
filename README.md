# cv-infra-user (carter)

cv-infra **소비자 예시**. "로봇 SW 프로젝트가 cv-infra를 실제로 쓰면 저장소가 어떻게 생기나"를
보여주는 최소 형태 — 파일 4개와 워크플로 1개가 전부다.

PR을 열면 GitHub Actions가 플랫폼의 재사용 워크플로를 호출하고, 플랫폼은 GPU 워크스테이션의
Isaac Sim에서 아래 `verify/sim.py`를 **입력 조합마다 한 번씩** 돌린 뒤 `verify/oracle.py`의 판정을
PR에 Check · sticky 코멘트 · 아티팩트(케이스별 출력 zip + 시뮬 로그)로 돌려준다.

## 파일

| 파일 | 무엇 |
|---|---|
| `verify/sim.py` | **표준 Isaac standalone 스크립트.** 창고 씬(공식 ROS 2 내비게이션 샘플)을 열고 Nova Carter를 `--drive_v` / `--drive_t` / `--yaw_rate` 대로 몰면서 섀시의 **GT 포즈**를 `verify/out/trajectory.csv`에 기록한다. 플랫폼은 이 파일을 import하지 않는다 — 컨테이너 안에서 `python.sh`로 실행할 뿐이다. |
| `verify/param_space.pict` | **입력 공간**(Microsoft PICT 문법). 축 이름 = argv 플래그: `drive_v: 0.2` → `--drive_v=0.2`. 플랫폼이 여기서 페어와이즈(k=2) 커버링 배열을 만들어 케이스를 뽑는다. |
| `verify/oracle.py` | **판정.** 시뮬 직후 같은 이미지·같은 argv로(GPU 없이) 돌아 `trajectory.csv`를 읽고 평평한 JSON dict 한 줄을 stdout에 낸다. |
| `verify/out/.gitkeep` | 출력 디렉토리를 **커밋된 상태로** 두기 위한 파일 — 아래 참고. |
| `.github/workflows/verify.yml` | 잡 하나(`uses: …@…`)와 `with:` 입력 7개. 이 저장소가 유지하는 통합 표면 전부. |

## 판정은 타입으로 말한다

`oracle.py`가 내는 dict는 **예약 키가 없다.** 값의 타입이 뜻을 정한다.

| 타입 | 뜻 |
|---|---|
| `bool` | **체크** — 케이스는 모든 bool이 true일 때 pass |
| 숫자 | **지표** — 커밋 간 baseline과 비교해 변화만 보고(게이트하지 않음) |
| `null` | 판정 불가 — 실패가 아니라 비율에서 제외 |
| 문자열 | 메모 |

여기서는 `moved`/`upright`가 체크, `displacement_m`/`final_z_m`이 지표, `note`가 메모다.

> ⚠ **exit code는 판정이 아니다.** `SimulationApp.close()`는 무슨 일이 있었든 프로세스를 status 0으로
>끝내고, stock `python.sh`는 비0을 1로 뭉갠다. 그래서 `sim.py`의 종료 코드는 pass/fail을 실을 수
> 없고, 오직 "이 케이스가 ERROR였다"만 의미한다. pass/fail은 전부 `oracle.py`가 결정한다.

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

CI가 하는 일과 같다 — 저장소 루트에서:

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v "$PWD":/cv/checkout -w /cv/checkout \
  --entrypoint /isaac-sim/python.sh \
  nvcr.io/nvidia/isaac-sim:5.1.0@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54 \
  verify/sim.py --drive_v=0.2 --drive_t=5 --yaw_rate=0.0

python3 verify/oracle.py --drive_v=0.2 --drive_t=5 --yaw_rate=0.0
```

`ACCEPT_EULA`가 없으면 `sim.py`는 부팅 전에 거부한다(exit 3). 컨테이너에는 디스플레이가 없으므로
기본은 headless고, `--gui`는 데스크톱에 설치된 Isaac에서 볼 때만 쓴다(`./python.sh verify/sim.py
--gui --drive_v=0.2 --drive_t=5 --yaw_rate=0.0`). CI에서 GUI로 부팅하면 행이나 크래시로 끝난다.

## `verify/out/.gitkeep`이 필요한 이유

플랫폼은 체크아웃을 **읽기 전용**으로 마운트하고, 그 위 `sim_output_dir` 경로에만 케이스별 호스트
디렉토리를 읽기·쓰기로 덮어 씌운다. 그래서 그 경로는 **체크아웃에 디렉토리로 존재해야** 한다 —
git은 빈 디렉토리를 추적하지 않으므로 `.gitkeep`을 커밋해 둔다. 산출물 자체는 `.gitignore`가
막는다.

## 구동 경로에 대한 정직한 메모

`sim.py`는 ROS 2 브리지를 켜고 번들 `rclpy`로 `/cmd_vel`을 발행하는 경로를 **먼저 시도**하고, 그게
안 되면 같은 파일의 두 번째 경로(아티큘레이션 휠 조인트 속도 직접 명령)로 내려간다. **어느 쪽도 아직
GPU에서 실측되지 않았다** — stock 이미지의 `python.sh` 안에서 `rclpy` import가 되는지, 브리지 공유
라이브러리가 로더에 보이는지가 가정으로 남아 있다(파일 상단 주석에 명시). 실제로 어느 경로가 쓰였는지는
케이스 로그의 `drive path:` 줄에 찍히고, 그 로그는 아티팩트로 함께 내려온다. 폴백 경로의 휠 반지름·
트레드 상수 역시 실측이 아닌 공표값 가정이다.
