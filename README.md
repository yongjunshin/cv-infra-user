# cv-infra-user

CV-Infra **소비자 / E2E 픽스처** (public). 로봇 SW 개발자 입장에서 CV-Infra를 *실제로 사용*하는 데모 프로젝트 — **carter 데모 로봇 SW(SUT)** + 시나리오 + GitHub Actions로 `push → 검증 요청 → PR에 pass/fail·회귀 결과 수신` 전반을 **End-to-End 검증**한다.

> **현 상태**: **배선 완료.** PR을 열면 `.github/workflows/verify.yml`이 ① SUT 이미지를 빌드해 GHCR에 push하고 ② 그 **다이제스트 ref만** 플랫폼 reusable workflow(`@v1`)에 넘겨 GPU 워크스테이션에서 검증을 돌린 뒤 ③ PR에 **Check + sticky 코멘트 + artifact**를 게시한다. carter SUT·시나리오(pass/fail/변형/커스텀 oracle)·커스텀 oracle 플러그인 예시도 그대로 살아 있다.

## 저장소 레이아웃
- `robot_sw/Dockerfile` — carter 데모 SUT 이미지. Nav2 · nav2_bringup · pointcloud_to_laserscan · carter_navigation(핀 커밋)을 **조립 완료**(베이스 `@sha256` 다이제스트 핀 · headless 런치). 상세·SUT 계약은 `robot_sw/README.md`.
- `scenarios/` — 표준 입력 인스턴스(`*.yaml`) + 커스텀 oracle 플러그인(`*.py`). 정본(pass) · goal 변형 · 의도적 fail · 커스텀 oracle 시나리오가 실측값으로 채워져 있다(스키마는 플랫폼 소유, 본 저장소는 인스턴스화만).
- `.github/workflows/verify.yml` — 소비자 검증 워크플로(2잡). `build-sut`(GitHub-hosted, GPU 무관 — fork PR을 GPU 호스트에서 빌드하지 않는다) → `verify`(**잡-레벨 `uses:`** 로 플랫폼 reusable workflow `@v1` 호출; 그 잡엔 `runs-on`/`steps`가 없다). 상세는 파일 상단 주석.

## 쓰는 법 (로봇 SW 개발자 관점)
1. 로봇 코드(`robot_sw/`)나 시나리오(`scenarios/`)를 고쳐 브랜치에 push하고 **PR을 연다**.
2. CI가 SUT 이미지를 빌드→GHCR push하고, cv-infra가 **그 이미지 ref**로 창고 미션을 시뮬레이션한다.
3. PR에서 결과를 받는다 — Check 결론(pass/fail/**인프라 문제**는 서로 다른 신호), 매트릭스+회귀 sticky 코멘트, `result.json`·MCAP·영상 artifact.
4. YAML이 틀리면 검증이 돌기 전에 **PR diff 위 inline annotation**으로 알려준다(exit 2). 로봇이 실패한 것과 구분된다.

CI 게이트에 올라가는 시나리오는 **pass 기대** 2종(`nova_carter_warehouse_goal{,_b}.yaml`)이다. `nova_carter_warehouse_obstacle_fail.yaml`은 **의도적 FAIL 픽스처**라 게이트에 넣으면 모든 PR이 빨개진다 — 필요할 때 수동으로 돌린다.

## 개요
- **담당 팀**: CV-User 팀 (E2E 픽스처·소비자). PM 지휘 하에 **플랫폼 계약의 타당성을 소비자 관점에서 구현·테스트**하고 마찰을 피드백한다.
- **경계 규칙**: 플랫폼을 *계약 + 릴리즈 이미지 + `verify.yml@vN`* 으로만 소비한다(상대경로 소스 참조 금지 — `uses: <ORG>/<PLATFORM_REPO>/...@vN` 형태로만 호출). carter SUT는 cv-infra가 내부를 수정하지 않는 **블랙박스 검증 대상**.
- E2E green = "요구사항이 사용자 손에서 실제로 충족됨"의 acceptance 증거.

## 재현성 (환경 핀)
- `robot_sw/Dockerfile` 베이스 이미지는 **정확한 태그로 핀**한다(floating `ros:jazzy` 금지). 현재 핀: `ros:jazzy-ros-base-noble`(ROS 2 Jazzy · ros-base · Ubuntu Noble — headless) + **불변 `@sha256` 다이제스트 핀 완료**(워크스테이션 최초 빌드에서 관측된 다이제스트로 고정).
- GitHub Action 버전도 핀(floating `@main` 금지). 빌드: `docker build -t carter-sut robot_sw/` — 본 이미지는 cv-infra가 *블랙박스*로 소비한다(레지스트리 ref로 시나리오에서 참조).

## 커스텀 oracle (시나리오-인접 플러그인)

내장 MVP oracle(`reached_goal`·`no_collision`) 외의 판정이 필요하면, oracle `.py` 모듈을 **시나리오 YAML과 같은 디렉토리**(`scenarios/`)에 두고 `acceptance_criteria`에서 `oracle: "<모듈>:<클래스>"`로 참조한다(플랫폼 결정 D-1(a), 2026-07-11 — 무설치·소비자 파생 이미지 불요·러너 이미지 불변).

- **살아있는 예시**: [`scenarios/max_time_to_goal.py`](scenarios/max_time_to_goal.py)(플러그인) + [`scenarios/nova_carter_warehouse_custom_oracle.yaml`](scenarios/nova_carter_warehouse_custom_oracle.yaml)(그 플러그인을 `oracle: "max_time_to_goal:MaxTimeToGoalOracle"` criterion으로 참조).
- **동작 방식**: 플랫폼이 요청 admit 시 시나리오 디렉토리를 sys.path에 올려 플러그인을 로드·검증하고(잘못된 참조는 실행 전 친절 에러 + exit 2 거부 — REQ-INTAKE-007/008), 실행 시 같은 디렉토리를 러너 컨테이너에 **동일 절대경로 read-only 마운트**해 평가 엔진이 균일하게 로드한다. 소비자는 파일을 두고 참조만 하면 된다.
- **작성 규칙**: `cv_infra.oracles.base.OracleBase` 서브클래스(클래스 속성 `name`/`version` + `validate_params`/`evaluate` 구현). `evaluate(telemetry, criteria)`는 GT 텔레메트리와 병합 criteria view(시나리오 goal/timeout + 각 criterion `params:`의 평탄 병합)를 읽어 `OracleOutcome`을 반환한다. **결정적·순수 파이썬**으로 쓴다(시계·랜덤·네트워크 금지). 모듈 스코프 import는 **stdlib + `cv_infra.*`**(러너 이미지에 설치됨)만.
- **모듈명은 고유하게**: 플러그인 파일명(=모듈명)은 러너 이미지에 이미 설치된 패키지명(`yaml`·`pydantic`·`cv_infra` 등)과 **비충돌**하는 고유한 이름으로 짓는다 — 이미 import된 이름과 겹치면 플러그인 대신 설치 패키지가 우선해(sys.modules 캐시) loud 에러("no attribute")로 실패한다(플랫폼 실측, 2026-07-11).
- ⚠️ **oracle 모듈 스코프에서 `omni.*`/`isaacsim.*` import 금지.** 러너는 평가 엔진을 **시뮬레이터(엔진) 부팅 전에** 구성한다 — 모듈 스코프 Isaac import는 부팅 전에 크래시한다(플랫폼 실측 근거). Isaac 상태가 필요한 판정은 커스텀 oracle의 몫이 아니다(텔레메트리는 플랫폼이 수집해 전달).
