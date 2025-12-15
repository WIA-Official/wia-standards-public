# WIA Biotech Research Report - Phase 3

**Communication Protocol Research**

---

## 1. Executive Summary

바이오테크 분야에서 데이터 교환과 통신 프로토콜은 연구소, 바이오파운드리, 의료 시스템 간의 상호운용성을 위해 핵심적인 역할을 합니다. 본 조사에서는 HL7 FHIR Genomics, GA4GH htsget, LIMS 통합 프로토콜, 실시간 시퀀싱 스트리밍 등 주요 표준과 기술을 분석합니다.

---

## 2. 전송 프로토콜 비교

### 2.1 WebSocket

**장점:**
- 양방향 실시간 통신 지원
- 낮은 오버헤드 (HTTP 대비)
- 지속적 연결로 실시간 데이터 스트리밍 적합
- PBrowse 등 협업 게놈 브라우저에서 활용

**단점:**
- 방화벽 이슈 가능
- Stateful 연결 관리 필요
- 로드 밸런싱 복잡

**바이오테크 적용:**
- 실시간 시퀀싱 데이터 스트리밍
- 나노포어 시퀀싱의 Read Until 기능
- 협업 데이터 분석 플랫폼
- LIMS 실시간 알림

### 2.2 REST API

**장점:**
- 널리 채택된 표준
- Stateless 설계로 확장성 우수
- 캐싱 용이
- 방화벽 친화적

**단점:**
- 폴링 필요 (실시간성 제한)
- 요청-응답 오버헤드

**바이오테크 적용:**
- LIMS 통합 표준 방식
- FHIR Genomics API
- 데이터 CRUD 작업
- 배치 분석 결과 조회

### 2.3 htsget Protocol (GA4GH)

**장점:**
- GA4GH 표준 프로토콜
- 보안 스트리밍 지원
- 게놈 인터벌 기반 데이터 접근
- SAM/BAM/CRAM/VCF/BCF 형식 지원

**단점:**
- 특화된 사용 사례
- 구현 복잡도

**바이오테크 적용:**
- 대용량 게놈 데이터 스트리밍
- 보안 데이터 공유
- 연구 데이터 페더레이션

### 2.4 MQTT

**장점:**
- 경량 프로토콜
- IoT 장비에 적합
- 발행-구독 모델
- QoS 레벨 지원

**단점:**
- 대용량 데이터에 제한적
- 추가 브로커 필요

**바이오테크 적용:**
- 센서 데이터 수집
- 장비 상태 모니터링
- 배양기/인큐베이터 IoT

---

## 3. 기존 바이오테크 통신 표준

### 3.1 HL7 FHIR Genomics

**개요:**
- HL7 Clinical Genomics Work Group 개발
- 의료 시스템과 게놈 데이터 통합
- Genomics Reporting IG 2.0.0 (14개 FHIR 프로필)

**주요 리소스:**
- `GenomicStudy`: 게놈 연구 메타데이터
- `MolecularDefinition`: 분자 엔티티 정의
- `Observation`: 변이 리포팅

**통신 방식:**
- RESTful API
- FHIR Bundle 트랜잭션
- 비동기 작업 지원

**2025년 동향:**
- genomDE (독일 국가 게놈 의료 전략) 매핑
- GA4GH Phenopacket Schema 통합
- 6천만 명 이상 헬스케어 게놈 시퀀싱 예상

### 3.2 GA4GH Standards

**htsget:**
- 보안 게놈 데이터 스트리밍
- 게놈 인터벌 지정 쿼리
- 4개 독립 클라이언트/서버 구현

**Data Repository Service (DRS):**
- 데이터 객체 발견 및 접근
- 클라우드 중립적 설계

**Workflow Execution Service (WES):**
- 분석 워크플로우 실행
- CWL/WDL 지원

### 3.3 LIMS Integration

**통합 방식:**
- REST API (현대 표준)
- HL7 v2.x (레거시)
- 파일 기반 (공유 디렉토리)
- 미들웨어 계층

**보안 요구사항:**
- HIPAA 준수
- 역할 기반 접근 제어 (RBAC)
- 데이터 암호화 (전송/저장)
- 감사 로깅

**규제 준수:**
- CLIA (임상 실험실)
- GLP (우수 실험실 관행)
- FDA 21 CFR Part 11

---

## 4. 실시간 시퀀싱 데이터 스트리밍

### 4.1 나노포어 시퀀싱

**실시간 분석:**
- 시퀀싱 시작과 동시에 분석 가능
- Read Until 선택적 시퀀싱
- 저지연 분석 필수

**데이터 스트리밍:**
- MinKNOW 소프트웨어
- 실시간 베이스콜링
- 클라우드 업로드 (BaseSpace 등)

### 4.2 스트림 프로세싱

**SeQual-Stream:**
- Apache Spark 기반
- HDFS 분산 처리
- 2.5억 DNA 서열 처리 시 2.7x 속도 향상

**클라우드 스트리밍:**
- 전송 중 처리로 지연 최소화
- NGS 데이터 대용량 처리

---

## 5. 메시지 형식 분석

### 5.1 JSON 기반

**장점:**
- 인간 가독성
- 언어 중립적
- FHIR 표준 형식
- 웹 친화적

**단점:**
- 바이너리 대비 큰 크기
- 파싱 오버헤드

### 5.2 Protocol Buffers / MessagePack

**장점:**
- 컴팩트한 바이너리 형식
- 빠른 직렬화/역직렬화
- 스키마 기반 검증

**단점:**
- 가독성 낮음
- 추가 도구 필요

### 5.3 권장 접근법

```
메시지 헤더: JSON (메타데이터, 라우팅)
메시지 페이로드:
  - 제어 메시지: JSON
  - 대용량 데이터: 바이너리 (참조 방식)
  - 시퀀싱 데이터: htsget/GA4GH 표준
```

---

## 6. 보안 고려사항

### 6.1 인증/인가

- OAuth 2.0 / OpenID Connect
- API Key (간단한 통합)
- mTLS (기기 인증)

### 6.2 데이터 보호

- TLS 1.3 전송 암호화
- 저장 시 AES-256 암호화
- 데이터 익명화/가명화

### 6.3 규제 준수

- HIPAA (미국 의료 정보)
- GDPR (EU 개인 정보)
- GxP (제약 산업)

---

## 7. 결론 및 권장사항

### 7.1 권장 통신 방식

| 사용 사례 | 권장 프로토콜 | 이유 |
|----------|--------------|------|
| 실시간 스트리밍 | WebSocket | 양방향, 저지연 |
| API 통합 | REST | 표준, 확장성 |
| 게놈 데이터 | htsget | GA4GH 표준 |
| 의료 시스템 | FHIR | HL7 표준 |
| IoT/센서 | MQTT | 경량, 효율적 |

### 7.2 WIA Biotech Protocol 설계 방향

1. **다중 전송 계층 지원**
   - WebSocket (실시간)
   - REST (요청-응답)
   - 추상화된 Transport 인터페이스

2. **메시지 형식**
   - JSON 기반 (FHIR 호환)
   - Phase 1 데이터 형식을 페이로드로 활용
   - 확장 가능한 메시지 타입

3. **상호운용성**
   - HL7 FHIR Genomics 호환
   - GA4GH 표준 참조
   - LIMS 통합 지원

4. **보안**
   - TLS 필수
   - 토큰 기반 인증
   - 감사 로깅

---

## 8. References

### Standards & Specifications
- [HL7 FHIR Genomics](https://www.hl7.org/fhir/genomics.html)
- [GA4GH htsget Protocol](https://academic.oup.com/bioinformatics/article/35/1/119/5040320)
- [GenomeX - Genomics Data Exchange](https://confluence.hl7.org/spaces/COD/pages/90361435/GenomeX+-+Genomics+Data+Exchange)

### Research Papers
- [Genomics on FHIR – National Strategy for Genomic Medicine (2025)](https://www.nature.com/articles/s41525-025-00516-1)
- [PBrowse: Real-time Collaborative Genome Browser](https://academic.oup.com/nar/article/45/9/e67/2918642)
- [SeQual-Stream: NGS Quality Control](https://bmcbioinformatics.biomedcentral.com/articles/10.1186/s12859-023-05530-7)

### Industry Resources
- [LIMS API Integration Guide](https://www.onqsoft.com.au/the-benefits-of-apis-in-lims-applications/)
- [Oxford Nanopore Real-time Sequencing](https://nanoporetech.com/platform/technology/advantages-of-real-time-sequencing)
- [Illumina LIMS Integration](https://emea.illumina.com/informatics/infrastructure-pipeline-setup/lims.html)

---

**Document Version**: 1.0
**Last Updated**: 2025-12
**Author**: WIA Biotech Working Group

---

弘益人間 - *Benefit All Humanity*
