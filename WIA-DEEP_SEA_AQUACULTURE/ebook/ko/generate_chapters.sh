#!/bin/bash

# Chapter 01 생성
cat > chapter-01.html << 'CH01'
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Chapter 01: 개요 및 아키텍처 - WIA-DEEP_SEA_AQUACULTURE</title>
    <style>:root{--primary:#0ea5e9;--primary-dark:#0284c7}*{margin:0;padding:0;box-sizing:border-box}body{font-family:-apple-system,BlinkMacSystemFont,sans-serif;line-height:1.6;color:#334155;background:linear-gradient(135deg,#0ea5e9 0%,#0284c7 100%);min-height:100vh;padding:20px}.container{max-width:900px;margin:0 auto;background:white;border-radius:16px;box-shadow:0 20px 60px rgba(0,0,0,.3);overflow:hidden}.header{background:linear-gradient(135deg,var(--primary-dark),var(--primary));color:white;padding:40px;text-align:center}.header h1{font-size:2.5em;margin-bottom:10px;text-shadow:2px 2px 4px rgba(0,0,0,.2)}.header p{font-size:1.2em;opacity:.95}.content{padding:40px}.content h2{color:var(--primary-dark);margin:30px 0 20px;padding-bottom:10px;border-bottom:3px solid var(--primary)}.content h3{color:var(--primary);margin:25px 0 15px}.content p{margin:15px 0;text-align:justify}.content ul,.content ol{margin:15px 0 15px 30px}.content li{margin:8px 0}.info-box{background:linear-gradient(135deg,#dbeafe,#bfdbfe);border-left:5px solid var(--primary);padding:20px;margin:25px 0;border-radius:8px}.code-block{background:#1e293b;color:#e2e8f0;padding:20px;border-radius:8px;overflow-x:auto;margin:20px 0;border-left:4px solid var(--primary)}.code-block code{font-family:"Courier New",monospace;font-size:.9em;line-height:1.5}table{width:100%;border-collapse:collapse;margin:20px 0;box-shadow:0 2px 8px rgba(0,0,0,.1)}table th{background:var(--primary);color:white;padding:15px;text-align:left;font-weight:600}table td{padding:12px 15px;border-bottom:1px solid #e2e8f0}table tr:hover{background:#f8fafc}.key-takeaways{background:linear-gradient(135deg,#d1fae5,#6ee7b7);padding:30px;margin:40px 0;border-radius:12px;border:2px solid #10b981}.key-takeaways h3{color:#065f46;margin-bottom:20px}.key-takeaways ul{margin-left:25px}.key-takeaways li{margin:12px 0;color:#064e3b}.navigation{display:flex;justify-content:space-between;margin-top:40px;padding-top:30px;border-top:2px solid #e2e8f0}.nav-button{background:var(--primary);color:white;padding:12px 24px;border-radius:8px;text-decoration:none;transition:all .3s;display:inline-block}.nav-button:hover{background:var(--primary-dark);transform:translateY(-2px);box-shadow:0 4px 12px rgba(14,165,233,.4)}.footer{background:#0f172a;color:white;text-align:center;padding:30px;font-size:.9em}.footer p{margin:5px 0}</style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🌊 Chapter 01</h1>
            <p>개요 및 아키텍처</p>
        </div>
        <div class="content">
            <h2>1. WIA-DEEP_SEA_AQUACULTURE 소개</h2>
            <p><strong>WIA-DEEP_SEA_AQUACULTURE</strong>는 심해 양식 시스템의 데이터 교환, API 인터페이스, 통신 프로토콜, 시스템 통합을 위한 국제 표준입니다.</p>
            <p>이 표준은 다양한 심해 양식 플랫폼, 센서 네트워크, 모니터링 시스템, 관리 소프트웨어 간의 상호운용성을 보장합니다.</p>

            <div class="info-box">
                <h4>🎯 표준의 목적</h4>
                <ul>
                    <li><strong>상호운용성</strong>: 서로 다른 제조사의 시스템 간 데이터 교환</li>
                    <li><strong>표준화</strong>: 일관된 데이터 포맷 및 API 규격</li>
                    <li><strong>확장성</strong>: 미래 기술 및 요구사항 수용</li>
                    <li><strong>보안성</strong>: 강력한 암호화 및 인증 체계</li>
                    <li><strong>철학</strong>: 弘益人間 (널리 인간을 이롭게 하라)</li>
                </ul>
            </div>

            <h2>2. 4단계 구현 방법론</h2>
            <p>WIA-DEEP_SEA_AQUACULTURE는 4단계(Phase 1-4)로 구성된 점진적 구현 방법론을 제공합니다.</p>

            <table>
                <thead>
                    <tr>
                        <th>단계</th>
                        <th>주제</th>
                        <th>핵심 내용</th>
                        <th>구현 난이도</th>
                    </tr>
                </thead>
                <tbody>
                    <tr>
                        <td><strong>Phase 1</strong></td>
                        <td>데이터 포맷</td>
                        <td>JSON 스키마, 타입 정의, 유효성 검증</td>
                        <td>기초</td>
                    </tr>
                    <tr>
                        <td><strong>Phase 2</strong></td>
                        <td>API 인터페이스</td>
                        <td>REST API, TypeScript SDK, 인증</td>
                        <td>중급</td>
                    </tr>
                    <tr>
                        <td><strong>Phase 3</strong></td>
                        <td>통신 프로토콜</td>
                        <td>메시지 구조, 운영 절차, 보안</td>
                        <td>중급</td>
                    </tr>
                    <tr>
                        <td><strong>Phase 4</strong></td>
                        <td>시스템 통합</td>
                        <td>WIA 표준 연동, 배포 아키텍처</td>
                        <td>고급</td>
                    </tr>
                </tbody>
            </table>

            <h2>3. 전체 아키텍처</h2>
            <div class="code-block">
<code>┌─────────────────────────────────────────────────────┐
│           Client Applications (Web, Mobile)         │
├─────────────────────────────────────────────────────┤
│                   API Gateway                       │
│              (WIA-OMNI-API 통합)                     │
├─────────────────────────────────────────────────────┤
│              Authentication Layer                   │
│               (WIA-AUTH 통합)                        │
├─────────────────────────────────────────────────────┤
│          WIA-DEEP_SEA_AQUACULTURE Core               │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐      │
│  │ Records   │  │ Validation│  │ Processing│      │
│  │ Manager   │  │  Engine   │  │  Pipeline │      │
│  └───────────┘  └───────────┘  └───────────┘      │
├─────────────────────────────────────────────────────┤
│              Data Access Layer                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐         │
│  │PostgreSQL│  │  Redis   │  │  S3      │         │
│  │(Primary) │  │ (Cache)  │  │(Storage) │         │
│  └──────────┘  └──────────┘  └──────────┘         │
├─────────────────────────────────────────────────────┤
│             Integration Layer                       │
│  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐  │
│  │Webhook │  │  MQTT  │  │GraphQL │  │ Event  │  │
│  │        │  │        │  │        │  │  Bus   │  │
│  └────────┘  └────────┘  └────────┘  └────────┘  │
└─────────────────────────────────────────────────────┘</code>
            </div>

            <h2>4. 핵심 구성 요소</h2>
            <h3>4.1 데이터 레이어</h3>
            <p>모든 레코드는 표준화된 JSON 포맷을 따르며, UUID v4 식별자와 ISO 8601 타임스탬프를 포함합니다.</p>
            <div class="code-block">
<code>{
  "type": "WIA-DEEP_SEA_AQUACULTURERecord",
  "version": "1.0",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-01T00:00:00Z",
  "data": {
    "category": "sensor_data",
    "value": {
      "temperature": 4.2,
      "depth": 1200,
      "salinity": 35.5
    },
    "metadata": {
      "source": "sensor-node-42",
      "location": {
        "latitude": 35.6762,
        "longitude": 139.6503
      }
    }
  },
  "signature": "ed25519-signature-here"
}</code>
            </div>

            <h3>4.2 API 레이어</h3>
            <p>RESTful API 엔드포인트를 통해 CRUD 작업을 수행합니다.</p>
            <div class="code-block">
<code># TypeScript SDK 사용 예제
import { createClient } from '@wia/deep-sea-aquaculture-sdk';

const client = createClient({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia.org/v1'
});

// 새 레코드 생성
const record = await client.createRecord({
  category: 'sensor_data',
  value: { temperature: 4.2, depth: 1200 },
  metadata: { source: 'sensor-node-42' }
});

console.log('Created record:', record.id);</code>
            </div>

            <h3>4.3 프로토콜 레이어</h3>
            <p>메시지 기반 통신 프로토콜을 정의하여 시스템 간 안전한 데이터 교환을 보장합니다.</p>
            <table>
                <thead>
                    <tr>
                        <th>메시지 타입</th>
                        <th>코드</th>
                        <th>용도</th>
                    </tr>
                </thead>
                <tbody>
                    <tr><td>REQUEST</td><td>0x01</td><td>작업 요청</td></tr>
                    <tr><td>RESPONSE</td><td>0x02</td><td>응답 반환</td></tr>
                    <tr><td>BROADCAST</td><td>0x03</td><td>시스템 전체 알림</td></tr>
                    <tr><td>ALERT</td><td>0x04</td><td>긴급 알림</td></tr>
                    <tr><td>SYNC</td><td>0x05</td><td>동기화</td></tr>
                    <tr><td>HEARTBEAT</td><td>0x06</td><td>헬스 체크</td></tr>
                </tbody>
            </table>

            <h3>4.4 통합 레이어</h3>
            <p>다른 WIA 표준 및 외부 시스템과의 통합을 지원합니다.</p>
            <table>
                <thead>
                    <tr>
                        <th>통합 대상</th>
                        <th>목적</th>
                        <th>필수 여부</th>
                    </tr>
                </thead>
                <tbody>
                    <tr><td>WIA-INTENT</td><td>의도 처리</td><td>필수</td></tr>
                    <tr><td>WIA-OMNI-API</td><td>API 게이트웨이</td><td>필수</td></tr>
                    <tr><td>WIA-AUTH</td><td>인증</td><td>필수</td></tr>
                    <tr><td>WIA-AUDIT</td><td>감사 로깅</td><td>필수</td></tr>
                    <tr><td>WIA-MONITOR</td><td>모니터링</td><td>권장</td></tr>
                </tbody>
            </table>

            <h2>5. 기술 스택</h2>
            <p>권장하는 기술 스택은 다음과 같습니다:</p>
            <ul>
                <li><strong>언어</strong>: TypeScript, Python, Go</li>
                <li><strong>데이터베이스</strong>: PostgreSQL (주), Redis (캐시)</li>
                <li><strong>메시지 큐</strong>: RabbitMQ, Apache Kafka</li>
                <li><strong>API 프레임워크</strong>: Express.js, FastAPI, Gin</li>
                <li><strong>보안</strong>: TLS 1.3, AES-256-GCM, Ed25519</li>
                <li><strong>배포</strong>: Docker, Kubernetes</li>
            </ul>

            <div class="key-takeaways">
                <h3>🎯 핵심 요점</h3>
                <ul>
                    <li><strong>4단계 방법론</strong>: Phase 1-4로 점진적 구현</li>
                    <li><strong>표준 레코드</strong>: JSON 포맷, UUID, ISO 8601</li>
                    <li><strong>RESTful API</strong>: CRUD 작업 지원</li>
                    <li><strong>TypeScript SDK</strong>: 간편한 클라이언트 구현</li>
                    <li><strong>메시지 프로토콜</strong>: 6가지 메시지 타입</li>
                    <li><strong>WIA 통합</strong>: 5개 주요 WIA 표준 연동</li>
                    <li><strong>보안</strong>: TLS 1.3, AES-256, Ed25519</li>
                    <li>弘益人間: 심해 양식 기술로 인류 식량 문제 해결 🌊</li>
                </ul>
            </div>

            <div class="navigation">
                <a href="index.html" class="nav-button">← 목차로</a>
                <a href="chapter-02.html" class="nav-button">02. 데이터 포맷 →</a>
            </div>
        </div>
        <div class="footer">
            <p><strong>WIA-DEEP_SEA_AQUACULTURE</strong> | Chapter 01: 개요 및 아키텍처</p>
            <p>© 2025 WIA - World Certification Industry Association</p>
            <p>弘益人間 (홍익인간) · Benefit All Humanity</p>
        </div>
    </div>
</body>
</html>
CH01

echo "Chapter 01 created"
