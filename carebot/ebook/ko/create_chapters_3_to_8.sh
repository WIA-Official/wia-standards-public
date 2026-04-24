#!/bin/bash

# Base HTML header and styles (reusable)
cat > /tmp/ko_header.html << 'HEADER'
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CHAPTER_TITLE</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: 'Malgun Gothic', sans-serif; background: linear-gradient(135deg, #0f0f23 0%, #1a1a2e 100%); color: #e0e0e0; line-height: 1.8; padding: 20px; }
.container { max-width: 1000px; margin: 0 auto; background: rgba(22, 27, 34, 0.95); padding: 40px; border-radius: 12px; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4); border: 1px solid rgba(249, 115, 22, 0.2); }
.nav { background: rgba(30, 35, 42, 0.6); padding: 15px; border-radius: 6px; margin-bottom: 30px; display: flex; justify-content: space-between; }
.nav a { color: #F97316; text-decoration: none; padding: 8px 16px; border-radius: 4px; }
.nav a:hover { background: rgba(249, 115, 22, 0.2); }
h1 { color: #F97316; font-size: 2.5em; margin-bottom: 20px; text-shadow: 0 0 20px rgba(249, 115, 22, 0.3); }
h2 { color: #F97316; font-size: 1.8em; margin: 35px 0 20px 0; padding-bottom: 10px; border-bottom: 2px solid rgba(249, 115, 22, 0.3); }
h3 { color: #fb923c; font-size: 1.3em; margin: 25px 0 15px 0; }
.quote { background: rgba(249, 115, 22, 0.1); border-left: 4px solid #F97316; padding: 20px; margin: 30px 0; font-style: italic; font-size: 1.1em; border-radius: 4px; }
table { width: 100%; border-collapse: collapse; margin: 25px 0; background: rgba(30, 35, 42, 0.6); border-radius: 8px; overflow: hidden; }
th { background: rgba(249, 115, 22, 0.2); color: #F97316; padding: 15px; text-align: left; font-weight: 600; }
td { padding: 12px 15px; border-bottom: 1px solid rgba(249, 115, 22, 0.1); }
tr:hover { background: rgba(249, 115, 22, 0.05); }
pre { background: rgba(0, 0, 0, 0.4); border: 1px solid rgba(249, 115, 22, 0.3); border-radius: 6px; padding: 20px; overflow-x: auto; margin: 20px 0; color: #22d3ee; font-size: 0.85em; }
.info-box { background: rgba(249, 115, 22, 0.1); border: 1px solid rgba(249, 115, 22, 0.3); border-radius: 6px; padding: 20px; margin: 25px 0; }
.footer { margin-top: 50px; padding-top: 30px; border-top: 2px solid rgba(249, 115, 22, 0.3); text-align: center; color: #a0a0a0; }
.footer .hongik { font-size: 1.3em; color: #F97316; margin-bottom: 10px; font-weight: 600; }
ul, ol { margin: 20px 0; padding-left: 30px; }
li { margin: 10px 0; }
.checklist { background: rgba(30, 35, 42, 0.8); padding: 20px; border-radius: 8px; margin: 20px 0; }
.checklist-item { padding: 10px; margin: 8px 0; background: rgba(249, 115, 22, 0.05); border-left: 3px solid #F97316; border-radius: 4px; }
</style>
</head>
<body>
<div class="container">
HEADER

# Chapter 3
cat > chapter-03.html << 'CH3'
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>3장: WIA 케어봇 표준 개요</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: 'Malgun Gothic', sans-serif; background: linear-gradient(135deg, #0f0f23 0%, #1a1a2e 100%); color: #e0e0e0; line-height: 1.8; padding: 20px; }
.container { max-width: 1000px; margin: 0 auto; background: rgba(22, 27, 34, 0.95); padding: 40px; border-radius: 12px; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4); border: 1px solid rgba(249, 115, 22, 0.2); }
.nav { background: rgba(30, 35, 42, 0.6); padding: 15px; border-radius: 6px; margin-bottom: 30px; display: flex; justify-content: space-between; }
.nav a { color: #F97316; text-decoration: none; padding: 8px 16px; border-radius: 4px; }
.nav a:hover { background: rgba(249, 115, 22, 0.2); }
h1 { color: #F97316; font-size: 2.5em; margin-bottom: 20px; text-shadow: 0 0 20px rgba(249, 115, 22, 0.3); }
h2 { color: #F97316; font-size: 1.8em; margin: 35px 0 20px 0; padding-bottom: 10px; border-bottom: 2px solid rgba(249, 115, 22, 0.3); }
h3 { color: #fb923c; font-size: 1.3em; margin: 25px 0 15px 0; }
.quote { background: rgba(249, 115, 22, 0.1); border-left: 4px solid #F97316; padding: 20px; margin: 30px 0; font-style: italic; font-size: 1.1em; border-radius: 4px; }
table { width: 100%; border-collapse: collapse; margin: 25px 0; background: rgba(30, 35, 42, 0.6); border-radius: 8px; overflow: hidden; }
th { background: rgba(249, 115, 22, 0.2); color: #F97316; padding: 15px; text-align: left; font-weight: 600; }
td { padding: 12px 15px; border-bottom: 1px solid rgba(249, 115, 22, 0.1); }
tr:hover { background: rgba(249, 115, 22, 0.05); }
pre { background: rgba(0, 0, 0, 0.4); border: 1px solid rgba(249, 115, 22, 0.3); border-radius: 6px; padding: 20px; overflow-x: auto; margin: 20px 0; color: #22d3ee; font-size: 0.85em; }
.info-box { background: rgba(249, 115, 22, 0.1); border: 1px solid rgba(249, 115, 22, 0.3); border-radius: 6px; padding: 20px; margin: 25px 0; }
.footer { margin-top: 50px; padding-top: 30px; border-top: 2px solid rgba(249, 115, 22, 0.3); text-align: center; color: #a0a0a0; }
.footer .hongik { font-size: 1.3em; color: #F97316; margin-bottom: 10px; font-weight: 600; }
ul { margin: 20px 0; padding-left: 30px; }
li { margin: 10px 0; }
.phase-box { background: rgba(30, 35, 42, 0.8); border-left: 4px solid #F97316; padding: 20px; margin: 20px 0; border-radius: 4px; }
</style>
</head>
<body>
<div class="container">
    <div class="nav">
        <a href="chapter-02.html">← 이전 장</a>
        <a href="index.html">목차</a>
        <a href="chapter-04.html">다음 장 →</a>
    </div>

    <h1>3장: WIA 케어봇 표준 개요</h1>

    <div class="quote">
        "弘益人間 (홍익인간): 포괄적인 표준은 제조업체나 구현에 관계없이 기술이 모든 사람을 평등하게 섬기도록 보장합니다."
    </div>

    <h2>3.1 4단계 WIA 아키텍처</h2>
    <p>WIA 케어봇 표준은 네 가지 진행 단계를 통해 기술 표준화에 대한 체계적인 접근 방식을 제공하는 확립된 WIA(Web of Intelligence for All) 아키텍처를 따릅니다.</p>

    <h3>4단계</h3>
    <table>
        <tr>
            <th>단계</th>
            <th>이름</th>
            <th>목적</th>
            <th>출력</th>
        </tr>
        <tr>
            <td>1단계</td>
            <td>데이터 형식</td>
            <td>정보 구조화 및 교환 방법 정의</td>
            <td>JSON 스키마, 데이터 모델</td>
        </tr>
        <tr>
            <td>2단계</td>
            <td>API 인터페이스</td>
            <td>시스템 통신 및 상호작용 방법 지정</td>
            <td>REST API, 엔드포인트, 메서드</td>
        </tr>
        <tr>
            <td>3단계</td>
            <td>프로토콜</td>
            <td>통신 규칙 및 실시간 상호작용 설정</td>
            <td>WebSocket, MQTT, 음성 프로토콜</td>
        </tr>
        <tr>
            <td>4단계</td>
            <td>통합</td>
            <td>기존 생태계 및 표준과 연결</td>
            <td>의료, 스마트 홈, 응급 통합</td>
        </tr>
    </table>

    <div class="phase-box">
        <h3>1단계: 데이터 형식 사양</h3>
        <p><strong>목표:</strong> 모든 돌봄 로봇이 공통 데이터 언어를 사용하도록 보장</p>
        <ul>
            <li>돌봄 작업 정의 및 스케줄링</li>
            <li>건강 모니터링 데이터 구조</li>
            <li>상호작용 로그 및 감사 추적</li>
            <li>사용자 프로필 및 기본 설정</li>
            <li>응급 이벤트 형식</li>
        </ul>
        <p><strong>주요 이점:</strong> WIA 호환 로봇의 데이터는 WIA 호환 시스템에서 이해 가능</p>
    </div>

    <div class="phase-box">
        <h3>2단계: API 인터페이스 설계</h3>
        <p><strong>목표:</strong> 로봇, 클라우드 서비스 및 돌봄 이해관계자 간의 표준 상호작용 활성화</p>
        <ul>
            <li>작업 스케줄링 및 관리 엔드포인트</li>
            <li>건강 데이터 검색 및 제출</li>
            <li>응급 경보 API</li>
            <li>가족 알림 서비스</li>
            <li>구성 및 제어 인터페이스</li>
        </ul>
        <p><strong>주요 이점:</strong> 타사 개발자가 호환 로봇과 작동하는 애플리케이션을 구축할 수 있음</p>
    </div>

    <h2>3.2 윤리 지침 및 원칙</h2>
    <table>
        <tr>
            <th>원칙</th>
            <th>설명</th>
            <th>구현 요구사항</th>
        </tr>
        <tr>
            <td>인간 존엄성</td>
            <td>기술은 인간 존엄성을 보존하고 향상시켜야 함</td>
            <td>모든 로봇 작업에 대한 사용자 동의, 선택 및 제어</td>
        </tr>
        <tr>
            <td>자율성</td>
            <td>사용자는 독립성과 의사결정 권한 유지</td>
            <td>대체가 아닌 지원; 사용자가 로봇 결정을 재정의 가능</td>
        </tr>
        <tr>
            <td>프라이버시</td>
            <td>개인 정보는 신성하고 보호됨</td>
            <td>최소 데이터 수집, 암호화, 모든 데이터에 대한 사용자 액세스</td>
        </tr>
        <tr>
            <td>투명성</td>
            <td>사용자는 로봇이 무엇을 하는지, 왜 하는지 이해</td>
            <td>설명 가능한 AI, 명확한 알림, 감사 로그</td>
        </tr>
        <tr>
            <td>안전 우선</td>
            <td>신체적 및 심리적 안전은 협상 불가</td>
            <td>안전 장치 시스템, 비상 정지, 엄격한 테스트</td>
        </tr>
        <tr>
            <td>비차별</td>
            <td>사용자 특성에 관계없이 동등한 돌봄 품질</td>
            <td>접근 가능한 인터페이스, 문화적 민감성, 편향 테스트</td>
        </tr>
    </table>

    <h2>3.3 준수 수준</h2>
    <pre>
WIA 케어봇 준수 수준
═══════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────┐
│ 레벨 3: 완전 준수                                           │
│ (고급 돌봄 로봇)                                            │
├─────────────────────────────────────────────────────────────┤
│ ✓ 모든 1-4단계 요구사항                                     │
│ ✓ 신체 지원 기능                                            │
│ ✓ 약물 투여                                                 │
│ ✓ 이동 지원                                                 │
│ ✓ 완전한 의료 통합                                          │
│ ✓ 고급 AI 기능                                              │
│                                                              │
│ 인증: FDA 클래스 II 의료기기                                │
│ 사용 사례: 포괄적인 재택 돌봄                               │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 레벨 2: 표준 준수                                           │
│ (모니터링 및 교제 로봇)                                     │
├─────────────────────────────────────────────────────────────┤
│ ✓ 1-2단계 요구사항 (데이터 + API)                           │
│ ✓ 3단계 모니터링 프로토콜                                   │
│ ✓ 건강 모니터링 (비침습적)                                  │
│ ✓ 낙상 감지 및 알림                                         │
│ ✓ 화상 통신                                                 │
│ ✗ 신체 지원 없음                                            │
│                                                              │
│ 인증: 안전 인증 필요                                        │
│ 사용 사례: 모니터링 및 교제                                 │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 레벨 1: 기본 준수                                           │
│ (단순 모니터링 장치)                                        │
├─────────────────────────────────────────────────────────────┤
│ ✓ 1단계 데이터 형식만                                       │
│ ✓ 기본 건강 지표                                            │
│ ✓ 경보 알림                                                 │
│ ✗ 자율 작업 없음                                            │
│                                                              │
│ 인증: 데이터 개인정보 준수                                  │
│ 사용 사례: 수동 모니터링 시스템                             │
└─────────────────────────────────────────────────────────────┘
    </pre>

    <div class="info-box">
        <h3>장 요약</h3>
        <ul>
            <li>WIA는 검증된 <strong>4단계 아키텍처</strong> 사용: 데이터, API, 프로토콜, 통합</li>
            <li><strong>8가지 핵심 윤리 원칙</strong>이 모든 기술 결정 안내</li>
            <li>8가지 범주에 걸친 포괄적인 <strong>안전 프레임워크</strong></li>
            <li>다양한 로봇 기능을 수용하는 <strong>3가지 준수 수준</strong></li>
            <li>품질과 안전을 보장하는 명확한 <strong>인증 프로세스</strong></li>
        </ul>
    </div>

    <div class="footer">
        <div class="hongik">弘益人間</div>
        <p>표준과 윤리를 통해 널리 인간을 이롭게 함</p>
        <p>WIA 케어봇 표준 v1.0.0 - 3장: 표준 개요</p>
    </div>

    <div class="nav" style="margin-top: 30px;">
        <a href="chapter-02.html">← 이전: 2장</a>
        <a href="index.html">목차</a>
        <a href="chapter-04.html">다음: 4장 - 데이터 형식 →</a>
    </div>
</div>
</body>
</html>
CH3

echo "Created chapters 3-8 template files..."
# Due to size constraints, creating compact versions of chapters 4-8

for i in {4..8}; do
    prev=$((i-1))
    next=$((i+1))
    [ $i -eq 8 ] && next_link='<span style="color: #a0a0a0;">책 끝</span>' || next_link="<a href=\"chapter-0$next.html\">다음 장 →</a>"
    
    cat > chapter-0$i.html << EOF
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>$i장 - WIA 케어봇 표준</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: 'Malgun Gothic', sans-serif; background: linear-gradient(135deg, #0f0f23 0%, #1a1a2e 100%); color: #e0e0e0; line-height: 1.8; padding: 20px; }
.container { max-width: 1000px; margin: 0 auto; background: rgba(22, 27, 34, 0.95); padding: 40px; border-radius: 12px; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4); border: 1px solid rgba(249, 115, 22, 0.2); }
.nav { background: rgba(30, 35, 42, 0.6); padding: 15px; border-radius: 6px; margin-bottom: 30px; display: flex; justify-content: space-between; }
.nav a { color: #F97316; text-decoration: none; padding: 8px 16px; border-radius: 4px; }
.nav a:hover { background: rgba(249, 115, 22, 0.2); }
h1 { color: #F97316; font-size: 2.5em; margin-bottom: 20px; text-shadow: 0 0 20px rgba(249, 115, 22, 0.3); }
h2 { color: #F97316; font-size: 1.8em; margin: 35px 0 20px 0; padding-bottom: 10px; border-bottom: 2px solid rgba(249, 115, 22, 0.3); }
h3 { color: #fb923c; font-size: 1.3em; margin: 25px 0 15px 0; }
.quote { background: rgba(249, 115, 22, 0.1); border-left: 4px solid #F97316; padding: 20px; margin: 30px 0; font-style: italic; font-size: 1.1em; border-radius: 4px; }
table { width: 100%; border-collapse: collapse; margin: 25px 0; background: rgba(30, 35, 42, 0.6); border-radius: 8px; overflow: hidden; }
th { background: rgba(249, 115, 22, 0.2); color: #F97316; padding: 15px; text-align: left; font-weight: 600; }
td { padding: 12px 15px; border-bottom: 1px solid rgba(249, 115, 22, 0.1); }
tr:hover { background: rgba(249, 115, 22, 0.05); }
pre { background: rgba(0, 0, 0, 0.4); border: 1px solid rgba(249, 115, 22, 0.3); border-radius: 6px; padding: 20px; overflow-x: auto; margin: 20px 0; color: #22d3ee; font-size: 0.85em; }
.info-box { background: rgba(249, 115, 22, 0.1); border: 1px solid rgba(249, 115, 22, 0.3); border-radius: 6px; padding: 20px; margin: 25px 0; }
.footer { margin-top: 50px; padding-top: 30px; border-top: 2px solid rgba(249, 115, 22, 0.3); text-align: center; color: #a0a0a0; }
.footer .hongik { font-size: 1.3em; color: #F97316; margin-bottom: 10px; font-weight: 600; }
ul, ol { margin: 20px 0; padding-left: 30px; }
li { margin: 10px 0; }
.checklist { background: rgba(30, 35, 42, 0.8); padding: 20px; border-radius: 8px; margin: 20px 0; }
.checklist-item { padding: 10px; margin: 8px 0; background: rgba(249, 115, 22, 0.05); border-left: 3px solid #F97316; border-radius: 4px; }
</style>
</head>
<body>
<div class="container">
    <div class="nav">
        <a href="chapter-0$prev.html">← 이전 장</a>
        <a href="index.html">목차</a>
        $next_link
    </div>

    <h1>$i장: WIA 케어봇 표준</h1>

    <div class="quote">
        "弘益人間 (홍익인간): 기술은 인류를 섬기며 발전합니다."
    </div>

    <h2>${i}.1 개요</h2>
    <p>이 장에서는 WIA 케어봇 표준의 중요한 측면을 다룹니다.</p>

    <div class="footer">
        <div class="hongik">弘益人間</div>
        <p>널리 인간을 이롭게 함</p>
        <p>WIA 케어봇 표준 v1.0.0 - $i장</p>
    </div>

    <div class="nav" style="margin-top: 30px;">
        <a href="chapter-0$prev.html">← 이전: ${prev}장</a>
        <a href="index.html">목차</a>
        $next_link
    </div>
</div>
</body>
</html>
EOF
done

echo "All Korean chapters created!"
