# 제5장: 2단계 - API 인터페이스

## 단백질 동역학을 위한 RESTful API

**弘益人間 (홍익인간)**

---

## 5.1 API 설계 원칙

### RESTful 아키텍처

WIA-PROTEIN-DYNAMICS API는 REST 원칙을 따릅니다:

- **리소스 지향**: URL이 리소스(단백질, 앙상블, 시뮬레이션)를 식별
- **무상태**: 각 요청이 필요한 모든 정보를 포함
- **표준 메서드**: 정의된 의미를 가진 GET, POST, PUT, DELETE
- **JSON 응답**: 전체에 걸쳐 일관된 데이터 형식

### 기본 URL

```
프로덕션: https://api.wia.live/protein-dynamics/v1
스테이징: https://staging-api.wia.live/protein-dynamics/v1
```

### 인증

```http
Authorization: Bearer <api_key>
X-WIA-Client-ID: <client_id>
```

---

## 5.2 핵심 엔드포인트

### 단백질 구조 및 동역학

#### 단백질 동역학 프로파일 조회

```http
GET /api/v1/protein/{protein_id}/dynamics
```

**경로 매개변수:**
| 매개변수 | 유형 | 설명 |
|---------|------|------|
| protein_id | string | UniProt ID, PDB ID, 또는 WIA ID |

**응답:**
```json
{
  "status": "success",
  "data": {
    "protein_id": "P00533",
    "name": "표피 성장 인자 수용체",
    "dynamics_profile": {
      "conformational_ensemble": { ... },
      "dynamics_metrics": { ... },
      "allosteric_network": { ... }
    }
  }
}
```

---

## 5.3 형태 앙상블 엔드포인트

### 앙상블 조회

```http
GET /api/v1/protein/{protein_id}/ensemble
```

**쿼리 매개변수:**
| 매개변수 | 유형 | 기본값 | 설명 |
|---------|------|-------|------|
| method | string | any | 생성 방법: md, alphaflow, boltzmann |
| max_states | int | 10 | 반환할 최대 상태 수 |
| min_population | float | 0.01 | 최소 집단 임계값 |

### 자유 에너지 지형 조회

```http
GET /api/v1/protein/{protein_id}/ensemble/landscape
```

---

## 5.4 동역학 예측 엔드포인트

### 동역학 프로파일 예측

```http
POST /api/v1/protein/dynamics/predict
```

**요청 본문:**
```json
{
  "sequence": "MRPSGTAGAALLALLAALCPASRALEEKK...",
  "options": {
    "ensemble_method": "alphaflow",
    "num_samples": 100,
    "include_flexibility": true,
    "include_disorder": true
  }
}
```

**응답:**
```json
{
  "status": "success",
  "job_id": "JOB-12345",
  "estimated_time_seconds": 60
}
```

### 작업 상태 조회

```http
GET /api/v1/jobs/{job_id}/status
```

---

## 5.5 약물 결합 엔드포인트

### 결합 동역학 예측

```http
POST /api/v1/binding/predict
```

**요청 본문:**
```json
{
  "protein_id": "P00533",
  "ligand": {
    "format": "smiles",
    "structure": "COc1cc2ncnc(Nc3cccc(c3)C#C)c2cc1OCCOC"
  },
  "options": {
    "predict_kinetics": true,
    "predict_pathway": true
  }
}
```

### 결합 시뮬레이션

```http
POST /api/v1/binding/simulate
```

---

## 5.6 시뮬레이션 엔드포인트

### MD 시뮬레이션 제출

```http
POST /api/v1/simulation/submit
```

**요청 본문:**
```json
{
  "protein_id": "P00533",
  "protocol": {
    "engine": "gromacs",
    "force_field": "amber14sb",
    "water_model": "tip3p",
    "temperature_K": 300,
    "duration_ns": 100
  }
}
```

### 시뮬레이션 결과 조회

```http
GET /api/v1/simulation/{simulation_id}/results
```

---

## 5.7 TypeScript SDK

### 설치

```bash
npm install @wia/protein-dynamics
```

### 사용 예제

```typescript
import { WIAProteinDynamics } from '@wia/protein-dynamics';

// 클라이언트 초기화
const client = new WIAProteinDynamics({
  apiKey: process.env.WIA_API_KEY,
  clientId: 'my-application'
});

// 동역학 프로파일 조회
async function getDynamicsProfile(proteinId: string) {
  const response = await client.protein.getDynamics(proteinId, {
    include: ['ensemble', 'metrics', 'allosteric'],
    detail: 'full'
  });

  console.log(`단백질: ${response.data.name}`);
  console.log(`상태 수: ${response.data.dynamics_profile.conformational_ensemble.num_states}`);

  return response.data;
}

// 결합 예측
async function predictBinding(proteinId: string, ligandSmiles: string) {
  const job = await client.binding.predict({
    proteinId,
    ligand: { format: 'smiles', structure: ligandSmiles },
    options: { predictKinetics: true }
  });

  const result = await client.jobs.waitForCompletion(job.jobId);

  console.log(`결합 친화도: ${result.thermodynamics.deltaG} kcal/mol`);
  console.log(`체류 시간: ${result.kinetics.residenceTime} s`);

  return result;
}
```

---

## 5.8 속도 제한 및 할당량

### 속도 제한

| 티어 | 요청/분 | 동시 작업 | 저장소 |
|-----|---------|----------|--------|
| 무료 | 60 | 1 | 1 GB |
| 기본 | 300 | 5 | 10 GB |
| 프로 | 1000 | 20 | 100 GB |
| 엔터프라이즈 | 무제한 | 맞춤 | 맞춤 |

### 속도 제한 헤더

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1705312260
```

### 오류 처리

```json
{
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "속도 제한 초과. 30초 후 재시도하세요.",
    "retry_after": 30
  }
}
```

---

## 요약

WIA-PROTEIN-DYNAMICS API는 다음을 제공합니다:
- 모든 동역학 작업을 위한 RESTful 엔드포인트
- 계산을 위한 비동기 작업 처리
- 쉬운 통합을 위한 TypeScript SDK
- 포괄적인 속도 제한 및 오류 처리
- 외부 데이터베이스와의 통합

---

**다음 장:** [3단계: 프로토콜](./07-phase3-protocols.md)

弘益人間 - 널리 인간을 이롭게 하라
