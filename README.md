# cv-infra-user

CV-Infra **소비자 / E2E 픽스처** (public). 로봇 SW 개발자 입장에서 CV-Infra를 *실제로 사용*하는 데모 프로젝트 — **carter 데모 로봇 SW(SUT)** + 시나리오 + GitHub Actions로 `push → 검증 요청 → PR에 pass/fail·회귀 결과 수신` 전반을 **End-to-End 검증**한다.

> ⚠️ **스캐폴드 대기 (구현 계획 Phase 0~)** — 현재는 3-저장소 토폴로지만 수립된 상태입니다. `robot_sw/`(carter SUT, `FROM ros:jazzy`+Nav2) · `scenarios/*.yaml` · `.github/workflows/verify.yml`은 구현 계획에 따라 채워집니다.

## 개요
- **담당 팀**: CV-User 팀 (E2E 픽스처·소비자). PM 지휘 하에 **플랫폼 계약의 타당성을 소비자 관점에서 구현·테스트**하고 마찰을 피드백한다.
- **경계 규칙**: 플랫폼(`cv-infra-workspace`)을 *계약 + 릴리즈 이미지 + `verify.yml@vN`* 으로만 소비한다(상대경로 소스 참조 금지). carter SUT는 cv-infra가 내부를 수정하지 않는 **블랙박스 검증 대상**.
- E2E green = "요구사항이 사용자 손에서 실제로 충족됨"의 acceptance 증거.

## 설계·계획 문서
상위 **private 메타 저장소 `cv-infra-project`** 의 `implementation-plan/07-repository-and-environments.md`(§8 CV-User E2E 워크스트림) 참조.
