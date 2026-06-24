# cv-infra-user

CV-Infra **소비자 / E2E 픽스처** (public). 로봇 SW 개발자 입장에서 CV-Infra를 *실제로 사용*하는 데모 프로젝트 — **carter 데모 로봇 SW(SUT)** + 시나리오 + GitHub Actions로 `push → 검증 요청 → PR에 pass/fail·회귀 결과 수신` 전반을 **End-to-End 검증**한다.

> ⚠️ **Phase 0 스캐폴드** — 디렉토리 골격과 핀된 베이스만 자리잡은 상태다. 실제 항법 스택(Nav2·carter_navigation)은 **Phase 2**, `verify.yml` 실배선은 **Phase 5**에서 채워진다.

## 저장소 레이아웃 (P0 스캐폴드)
- `robot_sw/Dockerfile` — carter 데모 SUT 이미지. **현재는 핀된 베이스 레이어만**(`FROM ros:jazzy-ros-base-noble`, headless). Nav2 · carter_navigation · pointcloud_to_laserscan 조립은 P2.
- `scenarios/` — 표준 입력 인스턴스(`*.yaml`) 자리. YAML은 P2~P3에 추가(스키마는 플랫폼 소유, 본 저장소는 인스턴스화만).
- `.github/workflows/verify.yml` — 소비자 검증 워크플로 **placeholder**. P5에서 플랫폼 reusable workflow를 `workflow_call`로 `@vN` 호출하도록 배선.

## 개요
- **담당 팀**: CV-User 팀 (E2E 픽스처·소비자). PM 지휘 하에 **플랫폼 계약의 타당성을 소비자 관점에서 구현·테스트**하고 마찰을 피드백한다.
- **경계 규칙**: 플랫폼을 *계약 + 릴리즈 이미지 + `verify.yml@vN`* 으로만 소비한다(상대경로 소스 참조 금지 — `uses: <ORG>/<PLATFORM_REPO>/...@vN` 형태로만 호출). carter SUT는 cv-infra가 내부를 수정하지 않는 **블랙박스 검증 대상**.
- E2E green = "요구사항이 사용자 손에서 실제로 충족됨"의 acceptance 증거.

## 재현성 (환경 핀)
- `robot_sw/Dockerfile` 베이스 이미지는 **정확한 태그로 핀**한다(floating `ros:jazzy` 금지). 현재 핀: `ros:jazzy-ros-base-noble`(ROS 2 Jazzy · ros-base · Ubuntu Noble — headless). **불변 `@sha256` 다이제스트 핀은 P2**(워크스테이션 최초 pull 시 관측된 다이제스트로 고정).
- GitHub Action 버전도 핀(floating `@main` 금지). 빌드(예정, P2~): `docker build -t carter-sut robot_sw/` — 본 이미지는 cv-infra가 *블랙박스*로 소비한다(레지스트리 ref로 시나리오에서 참조).
