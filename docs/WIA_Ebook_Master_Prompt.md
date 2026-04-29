# WIA Ebook 마스터 프롬프트 v1.0

> **목적:** Claude Code가 WIA 표준 Ebook을 생성할 때 사용하는 품질 기준
> **참조 템플릿:** emotion-ai/ebook/en/ (832KB, 8챕터, 평균 23KB/챕터)
> **작성일:** 2025-12-24

---

## 📊 품질 기준 요약

| 항목 | 최소 기준 | 권장 기준 |
|------|----------|----------|
| **챕터당 크기** | 15KB | 20-25KB |
| **섹션 수 (h2)** | 6개 | 8-10개 |
| **전체 Ebook** | 120KB | 160-200KB |
| **테이블** | 챕터당 2개 | 3-5개 |
| **코드 블록** | 기술 챕터 2개 | 3-5개 |
| **필수 섹션** | 요약, 복습문제 | + 학습목표, 다음장 예고 |

---

## 🎨 HTML 템플릿 구조

### 1. HEAD 섹션 (필수 CSS)

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="icon" type="image/x-icon" href="/favicon.ico">
    <title>Chapter N: 제목 - WIA {표준명} Standard</title>
    <style>
        :root{--primary:#EC4899;--primary-dark:#DB2777;--bg:#0f172a;--bg-card:#1e293b;--text:#f8fafc;--text-muted:#94a3b8;--border:#334155;--gold:#ffd700}
        *{margin:0;padding:0;box-sizing:border-box}
        body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:var(--bg);color:var(--text);min-height:100vh;line-height:1.8}
        .container{max-width:900px;margin:0 auto;padding:20px}
        
        /* Header */
        .header{display:flex;justify-content:space-between;align-items:center;padding:15px 0;border-bottom:1px solid var(--border);margin-bottom:30px;flex-wrap:wrap;gap:15px}
        .header-left{display:flex;align-items:center;gap:15px;flex-wrap:wrap}
        .header-logo{color:var(--text);text-decoration:none;font-weight:bold}
        .header-logo:hover{color:var(--primary)}
        .header-title{color:var(--primary);font-size:.95rem}
        .header-right{display:flex;align-items:center;gap:15px}
        .lang-switch{display:flex;gap:5px}
        .lang-btn{padding:6px 12px;border:1px solid var(--border);background:transparent;color:var(--text-muted);border-radius:6px;text-decoration:none;font-size:.85rem}
        .lang-btn.active{background:var(--primary);color:white;border-color:var(--primary)}
        
        /* Chapter Navigation */
        .chapter-nav{display:flex;justify-content:space-between;align-items:center;background:var(--bg-card);border:1px solid var(--border);border-radius:10px;padding:15px 20px;margin-bottom:30px;flex-wrap:wrap;gap:10px}
        .nav-btn{color:var(--text-muted);text-decoration:none;padding:8px 16px;border:1px solid var(--border);border-radius:6px;font-size:.9rem;transition:all .2s}
        .nav-btn:hover{border-color:var(--primary);color:var(--primary)}
        .nav-btn.disabled{opacity:.5;pointer-events:none}
        .nav-current{color:var(--primary);font-weight:bold}
        
        /* Content */
        .content{padding:20px 0}
        .content h1{font-size:2rem;margin-bottom:20px;color:var(--primary);border-bottom:2px solid var(--primary);padding-bottom:15px}
        .content h2{font-size:1.5rem;margin:40px 0 20px;color:var(--text);border-left:4px solid var(--primary);padding-left:15px}
        .content h3{font-size:1.2rem;margin:30px 0 15px;color:var(--text-muted)}
        .content p{margin:15px 0;color:var(--text)}
        .content ul,.content ol{margin:15px 0 15px 30px}
        .content li{margin:8px 0}
        
        /* Blockquote (철학 인용) */
        .content blockquote{background:linear-gradient(135deg,var(--bg-card),#2d1f3d);border-left:4px solid var(--gold);padding:20px;margin:25px 0;border-radius:0 10px 10px 0}
        .content blockquote p{color:var(--gold);margin:5px 0}
        
        /* Table */
        .content table{width:100%;border-collapse:collapse;margin:25px 0;background:var(--bg-card);border-radius:10px;overflow:hidden}
        .content th,.content td{padding:15px;text-align:left;border:1px solid var(--border)}
        .content th{background:var(--primary);color:white}
        .content tr:hover{background:rgba(236,72,153,.1)}
        
        /* Code Block */
        .content pre{background:#000;padding:20px;border-radius:10px;overflow-x:auto;margin:20px 0;border:1px solid var(--border)}
        .content code{font-family:'Fira Code',monospace;font-size:.9rem}
        
        /* HR */
        .content hr{border:none;border-top:1px solid var(--border);margin:40px 0}
        
        /* CTA Box */
        .cta-box{background:linear-gradient(135deg,var(--bg-card),#2d1f3d);border:2px solid var(--primary);border-radius:12px;padding:30px;text-align:center;margin:40px 0}
        .cta-box h3{color:var(--primary);margin-bottom:15px}
        .cta-box p{color:var(--text-muted);margin-bottom:20px}
        .cta-btn{display:inline-block;background:linear-gradient(135deg,var(--primary),var(--primary-dark));color:white;padding:14px 30px;border-radius:8px;text-decoration:none;font-weight:bold;transition:all .2s}
        .cta-btn:hover{transform:translateY(-2px);box-shadow:0 10px 30px rgba(236,72,153,.3)}
        
        /* Footer */
        footer{margin-top:60px;padding:30px 0;border-top:1px solid var(--border);text-align:center}
        footer .philosophy{color:var(--gold);font-size:1.1rem;margin-bottom:10px}
        footer p{color:var(--text-muted);margin:5px 0}
        footer .footer-links{margin-top:15px}
        footer .footer-links a{color:var(--text-muted);text-decoration:none;margin:0 10px}
        footer .footer-links a:hover{color:var(--primary)}
        
        /* Responsive */
        @media(max-width:768px){.header{flex-direction:column}.chapter-nav{flex-direction:column;text-align:center}.content h1{font-size:1.5rem}}
    </style>
</head>
```

### 2. BODY 구조 (필수 요소)

```html
<body>
    <div class="container">
        <!-- 1. Header (WIA 로고 + 언어 토글) -->
        <header class="header">
            <div class="header-left">
                <a href="https://wiastandards.com" class="header-logo" target="_blank">🤟 WIA</a>
                <span class="header-title">{이모지} {표준명} Ebook</span>
            </div>
            <div class="header-right">
                <div class="lang-switch">
                    <a href="../en/chapter-0N.html" class="lang-btn active">EN</a>
                    <a href="../ko/chapter-0N.html" class="lang-btn">KO</a>
                </div>
            </div>
        </header>
        
        <!-- 2. Chapter Navigation -->
        <nav class="chapter-nav">
            <a href="chapter-0(N-1).html" class="nav-btn">← Previous</a>
            <span class="nav-current">Ch.N / 8</span>
            <a href="index.html" class="nav-btn">📚 Contents</a>
            <a href="chapter-0(N+1).html" class="nav-btn">Next →</a>
        </nav>
        
        <!-- 3. Content (본문) -->
        <article class="content">
            <h1>{이모지} Chapter N: 제목</h1>
            
            <!-- 섹션들 (8-10개) -->
            <h2>N.1 첫 번째 섹션</h2>
            ...
            
            <!-- 마지막 필수 섹션들 -->
            <h2>N.8 Chapter Summary</h2>
            <h2>N.9 Review Questions</h2>
            <h2>N.10 Looking Ahead</h2>
        </article>
        
        <!-- 4. Chapter Navigation (하단) -->
        <nav class="chapter-nav">...</nav>
        
        <!-- 5. CTA Box -->
        <div class="cta-box">
            <h3>📚 Get the Full Ebook</h3>
            <p>EN $99 | KO $99 | Bundle $159</p>
            <a href="https://wiabook.com" target="_blank" class="cta-btn">🛒 WIA Book</a>
        </div>
        
        <!-- 6. Footer (弘益人間 필수!) -->
        <footer>
            <p class="philosophy">弘益人間 · Benefit All Humanity</p>
            <p>© 2025 WIA Standards | MIT License</p>
            <div class="footer-links">
                <a href="https://{표준명}.wiastandards.com/" target="_blank">🏠 Landing</a>
                <a href="https://{표준명}.wiastandards.com/simulator/" target="_blank">🎮 Simulator</a>
                <a href="https://wiabook.com" target="_blank">📚 Ebook Store</a>
            </div>
        </footer>
    </div>
</body>
</html>
```

---

## 📚 8챕터 표준 구조

### Chapter 1: Introduction (소개)
**목표:** 표준의 필요성과 배경 설명
| 섹션 | 내용 |
|------|------|
| 1.1 | What is {표준}? (정의) |
| 1.2 | Market Size and Growth (시장 규모) |
| 1.3 | Key Concepts (핵심 개념) |
| 1.4 | Classification/Categories (분류) |
| 1.5 | History/Origin (역사) |
| 1.6 | Major Use Cases (주요 활용) |
| 1.7 | WIA Philosophy: Hongik Ingan (철학) |
| 1.8 | Chapter Summary |
| 1.9 | Review Questions (6개) |
| 1.10 | Looking Ahead |

### Chapter 2: Current Challenges (현재 과제)
**목표:** 현재 문제점과 왜 표준이 필요한지
| 섹션 | 내용 |
|------|------|
| 2.1 | Industry Pain Points (산업 문제점) |
| 2.2 | Interoperability Issues (상호운용 문제) |
| 2.3 | Data Format Fragmentation (데이터 파편화) |
| 2.4 | Privacy & Security Concerns (보안 우려) |
| 2.5 | Regulatory Challenges (규제 과제) |
| 2.6 | Market Barriers (시장 장벽) |
| 2.7 | Case Studies (사례 연구) |
| 2.8 | Chapter Summary |
| 2.9 | Review Questions |
| 2.10 | Looking Ahead |

### Chapter 3: Standard Overview (표준 개요)
**목표:** WIA 표준의 전체 구조 소개
| 섹션 | 내용 |
|------|------|
| 3.1 | WIA Standard Architecture (아키텍처) |
| 3.2 | Four-Phase Approach (4단계 접근) |
| 3.3 | Design Principles (설계 원칙) |
| 3.4 | Compatibility & Compliance (호환성) |
| 3.5 | Versioning Strategy (버전 전략) |
| 3.6 | Ecosystem Overview (에코시스템) |
| 3.7 | Comparison with Alternatives (대안 비교) |
| 3.8 | Chapter Summary |
| 3.9 | Review Questions |
| 3.10 | Looking Ahead |

### Chapter 4: Phase 1 - Data Format (데이터 형식)
**목표:** 데이터 구조와 JSON 스키마 상세
| 섹션 | 내용 |
|------|------|
| 4.1 | Data Model Overview (데이터 모델) |
| 4.2 | JSON Schema Definition (스키마 정의) |
| 4.3 | Core Data Types (핵심 타입) |
| 4.4 | Metadata Structure (메타데이터) |
| 4.5 | Validation Rules (검증 규칙) |
| 4.6 | Example Payloads (예제) |
| 4.7 | Migration from Legacy (마이그레이션) |
| 4.8 | Chapter Summary |
| 4.9 | Review Questions |

### Chapter 5: Phase 2 - API Interface (API 인터페이스)
**목표:** REST API 설계와 엔드포인트
| 섹션 | 내용 |
|------|------|
| 5.1 | API Design Philosophy (설계 철학) |
| 5.2 | RESTful Endpoints (엔드포인트) |
| 5.3 | Authentication & Authorization (인증) |
| 5.4 | Request/Response Format (요청/응답) |
| 5.5 | Error Handling (오류 처리) |
| 5.6 | Rate Limiting & Quotas (제한) |
| 5.7 | SDK Overview (SDK 개요) |
| 5.8 | API Examples (예제 코드) |
| 5.9 | Chapter Summary |
| 5.10 | Review Questions |

### Chapter 6: Phase 3 - Protocol (프로토콜)
**목표:** 실시간 통신과 스트리밍
| 섹션 | 내용 |
|------|------|
| 6.1 | Protocol Architecture (프로토콜 아키텍처) |
| 6.2 | Transport Layer (전송 계층) |
| 6.3 | Message Format (메시지 형식) |
| 6.4 | Real-time Streaming (실시간 스트리밍) |
| 6.5 | Connection Management (연결 관리) |
| 6.6 | Security & Encryption (보안) |
| 6.7 | Chapter Summary |
| 6.8 | Review Questions |

### Chapter 7: Phase 4 - Integration (통합)
**목표:** 시스템 통합과 상호운용
| 섹션 | 내용 |
|------|------|
| 7.1 | Integration Patterns (통합 패턴) |
| 7.2 | Domain-Specific Adapters (도메인 어댑터) |
| 7.3 | Third-party Integration (서드파티 연동) |
| 7.4 | Legacy System Bridge (레거시 연결) |
| 7.5 | Cloud Deployment (클라우드 배포) |
| 7.6 | Monitoring & Analytics (모니터링) |
| 7.7 | Chapter Summary |
| 7.8 | Review Questions |

### Chapter 8: Implementation & Certification (구현 및 인증)
**목표:** 실제 구현 가이드와 WIA 인증
| 섹션 | 내용 |
|------|------|
| 8.1 | Getting Started (시작하기) |
| 8.2 | Implementation Checklist (체크리스트) |
| 8.3 | Sample Implementation (샘플 구현) |
| 8.4 | Testing & Validation (테스트) |
| 8.5 | WIA Certification Process (인증 절차) |
| 8.6 | Certification Levels (인증 등급) |
| 8.7 | Case Studies (성공 사례) |
| 8.8 | Future Roadmap (로드맵) |
| 8.9 | Chapter Summary |
| 8.10 | Review Questions |

---

## 📋 필수 요소 체크리스트

### 각 챕터 시작 (선택적)
```html
<div class="learning-objectives">
    <h3>📎 Learning Objectives</h3>
    <p>After completing this chapter, you will be able to:</p>
    <ul>
        <li>Objective 1 (동사로 시작: Define, Explain, Implement...)</li>
        <li>Objective 2</li>
        <li>Objective 3</li>
    </ul>
</div>
```

### Chapter Summary (필수!)
```html
<h2>N.8 Chapter Summary</h2>
<p>[OK] <strong>Key Takeaways:</strong></p>
<ol>
    <li><strong>Point 1:</strong> 설명</li>
    <li><strong>Point 2:</strong> 설명</li>
    <li><strong>Point 3:</strong> 설명</li>
    <li><strong>Point 4:</strong> 설명</li>
    <li><strong>Point 5:</strong> 설명</li>
</ol>
```

### Review Questions (필수! 6개)
```html
<h2>N.9 Review Questions</h2>
<ol>
    <li>Question 1 (정의/개념)</li>
    <li>Question 2 (비교/분류)</li>
    <li>Question 3 (적용/예시)</li>
    <li>Question 4 (분석/설명)</li>
    <li>Question 5 (평가/판단)</li>
    <li>Question 6 (종합/설계)</li>
</ol>
```

### Looking Ahead (필수!)
```html
<h2>N.10 Looking Ahead</h2>
<p>In Chapter (N+1), we will explore {다음 챕터 주제}...</p>
<hr>
<p><strong>Chapter N Complete</strong> | Approximate pages: 16</p>
<p><a href="chapter-0(N+1).html">Next: Chapter (N+1) - {다음 제목}</a></p>
```

### 弘益人間 Quote (Chapter 1 또는 7에 필수)
```html
<blockquote>
    <p><strong>弘益人間 (Hongik Ingan)</strong></p>
    <p>"Benefit All Humanity"</p>
    <p>This ancient Korean philosophy guides the WIA {표준명} Standard...</p>
</blockquote>
```

---

## 📊 시각적 요소 (Pattern Interrupts)

### 테이블 (챕터당 2-5개)
```html
<table border="1" cellpadding="10">
    <thead>
        <tr>
            <th>Column 1</th>
            <th>Column 2</th>
            <th>Column 3</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>Data</td>
            <td>Data</td>
            <td>Data</td>
        </tr>
    </tbody>
</table>
```

### 코드 블록 (기술 챕터에 필수)
```html
<pre>
{
    "format": "WIA-{표준}-v1.0",
    "timestamp": "2025-01-01T00:00:00Z",
    "data": {
        "field1": "value1",
        "field2": 123
    }
}
</pre>
```

### 리스트 (중요 포인트)
```html
<ul>
    <li><strong>Point 1:</strong> 설명</li>
    <li><strong>Point 2:</strong> 설명</li>
    <li><strong>Point 3:</strong> 설명</li>
</ul>
```

### Info Box
```html
<p>[i] <strong>Note:</strong> 중요한 정보나 팁</p>
```

---

## ❌ 하지 말 것!

```
❌ 챕터당 15KB 미만 (너무 짧음)
❌ 섹션 5개 이하 (내용 빈약)
❌ Summary/Review Questions 누락
❌ 테이블/코드 없는 기술 챕터
❌ 밝은 배경 (white, #ffffff)
❌ 弘益人間 푸터 누락
❌ 언어 토글 없음
❌ 외부 링크 target="_blank" 누락
❌ CTA Box 누락
```

---

## ✅ 생성 프로세스

1. **참조 템플릿 확인**
   ```bash
   cat emotion-ai/ebook/en/chapter-01.html  # 구조 참고
   ```

2. **8챕터 생성 (EN 먼저)**
   - chapter-01.html ~ chapter-08.html
   - 각 20-25KB 목표

3. **index.html (목차) 생성**
   - 8챕터 링크
   - 도서 정보 박스

4. **KO 버전 생성**
   - EN 번역
   - 언어 토글 링크 수정

5. **품질 검증**
   ```bash
   for f in ebook/en/chapter-*.html; do
     echo "$f: $(wc -c < $f) bytes, $(grep -c '<h2>' $f) sections"
   done
   ```

---

## 📁 참조 템플릿 경로

| 항목 | 경로 |
|------|------|
| **최고 품질 템플릿** | `emotion-ai/ebook/en/` |
| 대안 템플릿 | `pet-health-passport/ebook/en/` |
| 대안 템플릿 | `battery-passport/ebook/en/` |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

---

*WIA Ebook Master Prompt v1.0 | 2025-12-24 | Claude (동생)*
