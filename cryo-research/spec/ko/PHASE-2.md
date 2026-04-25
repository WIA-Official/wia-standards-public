# WIA-CRYO-010 단계 2

**표준**: WIA-CRYO-010  
**단계**: 2  
**버전**: 1.0.0  
**날짜**: 2025년 1월  
**상태**: 활성  

## 개요

WIA-CRYO-010 표준의 단계 2는 냉동보존 연구 데이터 관리의 핵심 구성 요소를 정의합니다.
이 문서는 영문 PHASE-2.md의 한국어 번역본입니다.

弘益人間 (홍익인간) - 널리 인간을 이롭게 하라

## 핵심 내용

이 단계에서는 다음을 다룹니다:

### 표준화된 접근 방식
- 데이터 형식 및 구조
- 알고리즘 및 계산 방법
- API 및 통신 프로토콜
- 시스템 통합 지침

### 상호 운용성
WIA-CRYO-010은 다양한 시스템과 플랫폼 간의 원활한 데이터 교환을 보장합니다:

+--------------------------------------------------------------------------+
| 시스템 유형           | 통합 방법            | 지원 형식                |
+--------------------------------------------------------------------------+
| LIMS                 | 어댑터/플러그인        | JSON-LD, CSV            |
| ELN                  | 내보내기/가져오기      | JSON-LD                 |
| 데이터베이스          | 네이티브 스키마        | PostgreSQL, MongoDB     |
| 분석 도구            | REST API             | JSON, CSV               |
| 저장소               | 파일 기반            | JSON-LD, HDF5           |
+--------------------------------------------------------------------------+

## JSON-LD 데이터 형식

모든 WIA-CRYO-010 데이터는 JSON-LD 형식을 사용합니다:



## 필수 필드

모든 데이터 레코드는 다음 필드를 포함해야 합니다:

1. **@context**: 스키마 컨텍스트 URL
2. **@type**: 데이터 유형 (예: CryoResearchExperiment)
3. **@id**: 고유 식별자 (UUID 권장)
4. **version**: 스키마 버전 (시맨틱 버전 관리)
5. **created**: 생성 타임스탬프 (ISO-8601)
6. **modified**: 수정 타임스탬프 (ISO-8601)

## 실험 기록 스키마



## 생존력 계산

### 기본 생존력 백분율
```
생존력 (%) = (생존 세포 / 전체 세포) × 100
```

### 회복률
```
회복률 (%) = (해동 후 세포 / 동결 전 세포) × 100
```

### 유효 생존력
```
유효 생존력 = 생존력 × 회복률 / 100
```

### 신뢰 구간 (95%)
```
CI = p ± 1.96 × √(p(1-p)/n)
여기서 p = 생존력 (소수점), n = 표본 크기
```

## API 엔드포인트

### 기본 URL
```
https://api.wia.org/cryo-research/v1
```

### 인증
모든 요청은 OAuth 2.0 Bearer 토큰이 필요합니다:
```
Authorization: Bearer {access_token}
```

### 실험 조회
```http
GET /experiments?cell_type=hepatocyte&viability_min=80
```

### 실험 제출
```http
POST /experiments
Content-Type: application/ld+json

{실험 JSON-LD 데이터}
```

## 품질 관리

모든 데이터는 다음 품질 기준을 충족해야 합니다:

### 완전성
- 모든 필수 필드 채워짐
- 누락 값 < 5%
- 메타데이터 포함

### 일관성
- 날짜 순서 검증
- 범위 검사 통과
- 교차 필드 유효성

### 정확성
- 교정된 장비 사용
- 표준 방법 적용
- 품질 점수 ≥ 0.8

## 통계 분석

### 표본 크기 계산
```
n = [(Zα + Zβ)² × (p₁(1-p₁) + p₂(1-p₂))] / (p₁ - p₂)²
여기서:
Zα = 1.96 (α=0.05, 양측)
Zβ = 0.84 (검정력=0.80)
p₁ = 예상 비율 그룹 1
p₂ = 예상 비율 그룹 2
```

### t-검정 (두 표본)
```
t = (x̄₁ - x̄₂) / √(s₁²/n₁ + s₂²/n₂)
```

### 효과 크기 (Cohen's d)
```
d = (x̄₁ - x̄₂) / s_pooled
```

## 시스템 통합

### LIMS 통합
LIMS 시스템과의 통합을 위한 어댑터:

```python
def lims_to_wia(lims_data):
    """LIMS 데이터를 WIA-CRYO-010 형식으로 변환"""
    return {
        "@context": "https://wia.org/standards/cryo-research/v1",
        "@type": "CryoResearchExperiment",
        "experimentId": lims_data['sample_id'],
        "title": lims_data['test_name'],
        ...
    }
```

### 데이터베이스 스키마

#### PostgreSQL
```sql
CREATE TABLE experiments (
    experiment_id VARCHAR(255) PRIMARY KEY,
    json_ld JSONB NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);
```

#### MongoDB
```javascript
db.createCollection("experiments", {
   validator: {
      $jsonSchema: {
         required: ["@context", "@type", "experimentId"]
      }
   }
})
```

## 보안

### 암호화
- 저장 데이터: AES-256
- 전송 데이터: TLS 1.3
- API 키: bcrypt 해시

### 접근 제어
역할 기반 권한:
- researcher: 읽기, 본인 작성
- data_manager: 읽기, 모든 작성, 본인 삭제
- admin: 모든 권한

## 백업 및 복구

### 3-2-1 규칙
- 데이터 3개 사본
- 2가지 다른 미디어 유형
- 1개 오프사이트 백업

### 자동 백업
```bash
#!/bin/bash
DATE=$(date +%Y%m%d)
pg_dump cryo_research > /backup/cryo_${DATE}.sql
gzip /backup/cryo_${DATE}.sql
```

## 성능 최적화

### 캐싱
- Redis를 통한 자주 액세스하는 데이터
- CDN을 통한 정적 스키마
- 데이터베이스 인덱스 최적화

### 대량 작업
```http
POST /bulk-import
Content-Type: application/x-ndjson

{"@type": "CryoResearchExperiment", ...}
{"@type": "CryoResearchExperiment", ...}
```

## 모니터링

### 추적 지표
- API 응답 시간 (p50, p95, p99)
- 오류율
- 데이터 수집 속도
- 저장 공간 사용량

### 경고 규칙
- 높은 오류율 (>5%)
- 낮은 생존력 감지 (<70%)
- 시스템 과부하

## 배포

### Docker 컨테이너화
```dockerfile
FROM python:3.10
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
CMD ["python", "app.py"]
```

### Kubernetes
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-research-api
spec:
  replicas: 3
```

## 테스트

### 단위 테스트
```python
def test_viability_calculation():
    result = calculate_viability(87, 100)
    assert result['viability'] == 87.0
```

### 통합 테스트
```python
def test_api_submit():
    response = client.post('/experiments', json=data)
    assert response.status_code == 201
```

## 참고 자료

- JSON-LD: https://www.w3.org/TR/json-ld/
- JSON Schema: https://json-schema.org/
- ISO 8601: https://www.iso.org/iso-8601-date-and-time-format.html
- OAuth 2.0: https://oauth.net/2/
- PostgreSQL: https://www.postgresql.org/
- MongoDB: https://www.mongodb.com/

## 추가 리소스

### 도구 및 라이브러리
- Python: `jsonschema`, `requests`, `pandas`
- R: `jsonlite`, `httr`, `dplyr`
- JavaScript: `ajv`, `axios`, `json-ld`

### 커뮤니티
- GitHub: https://github.com/WIA-Official/wia-standards
- 토론: https://github.com/WIA-Official/wia-standards/discussions
- 이슈: https://github.com/WIA-Official/wia-standards/issues

---

영문 원본 문서는 더 상세한 내용을 포함합니다.  
완전한 기술 사양은 영문 PHASE-2.md를 참조하십시오.

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
