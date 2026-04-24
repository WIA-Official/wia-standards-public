#!/bin/bash

# Create Chapter 2
cat > /home/user/wia-standards/carebot/ebook/ko/chapter-02.html << 'EOF'
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>2장: 현재 과제 - WIA 케어봇 표준</title>
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
.warning-box { background: rgba(239, 68, 68, 0.1); border: 2px solid rgba(239, 68, 68, 0.4); border-radius: 6px; padding: 20px; margin: 25px 0; }
.info-box { background: rgba(249, 115, 22, 0.1); border: 1px solid rgba(249, 115, 22, 0.3); border-radius: 6px; padding: 20px; margin: 25px 0; }
.footer { margin-top: 50px; padding-top: 30px; border-top: 2px solid rgba(249, 115, 22, 0.3); text-align: center; color: #a0a0a0; }
.footer .hongik { font-size: 1.3em; color: #F97316; margin-bottom: 10px; font-weight: 600; }
ul { margin: 20px 0; padding-left: 30px; }
li { margin: 10px 0; }
</style>
</head>
<body>
<div class="container">
    <div class="nav">
        <a href="chapter-01.html">← 이전 장</a>
        <a href="index.html">목차</a>
        <a href="chapter-03.html">다음 장 →</a>
    </div>

    <h1>2장: 돌봄 로봇의 현재 과제</h1>

    <div class="quote">
        "弘益人間 (홍익인간): 우리가 직면한 과제를 이해하는 것이 진정으로 인류를 섬기는 솔루션을 향한 첫 걸음입니다."
    </div>

    <h2>2.1 신뢰 및 수용 문제</h2>
    <p>돌봄 로봇 채택에 대한 가장 중요한 장벽 중 하나는 사용자와 자율 시스템 간의 근본적인 신뢰 격차입니다. 취약한 인구(노인 및 장애인)를 다룰 때 신뢰는 단순히 좋은 기능이 아니라 성공적인 배포에 필수적입니다.</p>

    <h3>이해관계자 그룹별 신뢰 장벽</h3>
    <table>
        <tr>
            <th>이해관계자</th>
            <th>주요 우려사항</th>
            <th>신뢰 장벽 수준</th>
            <th>핵심 요구사항</th>
        </tr>
        <tr>
            <td>노인 사용자 (65-75세)</td>
            <td>기술 복잡성, 대체에 대한 두려움</td>
            <td>높음 (68%)</td>
            <td>단순성, 인간과 유사한 상호작용</td>
        </tr>
        <tr>
            <td>노인 사용자 (75세 이상)</td>
            <td>기술 생소함, 자율성 상실</td>
            <td>매우 높음 (82%)</td>
            <td>점진적 도입, 선택권</td>
        </tr>
        <tr>
            <td>장애인 사용자</td>
            <td>신뢰성, 신체적 안전</td>
            <td>중상 (61%)</td>
            <td>검증된 안전 기록, 인증</td>
        </tr>
        <tr>
            <td>가족 돌봄 제공자</td>
            <td>적절한 돌봄 품질, 응급상황</td>
            <td>중간 (54%)</td>
            <td>투명성, 모니터링 액세스</td>
        </tr>
        <tr>
            <td>전문 돌봄 제공자</td>
            <td>일자리 대체, 책임</td>
            <td>중간 (48%)</td>
            <td>대체가 아닌 증강</td>
        </tr>
        <tr>
            <td>의료 제공자</td>
            <td>의료 책임, 데이터 정확성</td>
            <td>중상 (63%)</td>
            <td>FDA/규제 승인, 표준</td>
        </tr>
    </table>

    <h2>2.2 개인정보 보호 및 감시 우려</h2>
    <p>돌봄 로봇은 본질적으로 사용자의 일상생활에 대한 광범위한 데이터를 관찰하고 수집해야 합니다. 이는 효과적인 돌봄 제공과 개인 프라이버시 보호 사이에 내재적인 긴장을 만듭니다.</p>

    <h3>개인정보 보호 위협 매트릭스</h3>
    <table>
        <tr>
            <th>데이터 유형</th>
            <th>수집 필요성</th>
            <th>개인정보 위험</th>
            <th>완화 전략</th>
        </tr>
        <tr>
            <td>비디오 녹화</td>
            <td>낙상 감지, 활동 모니터링</td>
            <td>매우 높음</td>
            <td>로컬 처리, 암호화, 시간 제한</td>
        </tr>
        <tr>
            <td>오디오 녹음</td>
            <td>음성 명령, 고통 감지</td>
            <td>높음</td>
            <td>웨이크 워드 활성화, 로컬 처리</td>
        </tr>
        <tr>
            <td>건강 지표</td>
            <td>의료 모니터링, 알림</td>
            <td>매우 높음</td>
            <td>HIPAA 준수, 암호화 저장</td>
        </tr>
        <tr>
            <td>위치 추적</td>
            <td>이동 패턴, 배회 알림</td>
            <td>중상</td>
            <td>세분성 제한, 가정 내만</td>
        </tr>
        <tr>
            <td>활동 로그</td>
            <td>돌봄 품질, 패턴 분석</td>
            <td>중간</td>
            <td>집계, 익명화</td>
        </tr>
    </table>

    <h2>2.3 신체적 안전 위험</h2>
    <div class="warning-box">
        <h3>돌봄 로봇의 중요 안전 사고 (2018-2023)</h3>
        <ul>
            <li><strong>이동 로봇 충돌 (2019):</strong> 장애물 감지 실패로 인한 휠체어 로봇 충돌로 84세 사용자의 고관절 골절 발생</li>
            <li><strong>약물 투여 오류 (2020):</strong> 약물 식별 실패로 인한 잘못된 용량 투여로 응급 개입 필요</li>
            <li><strong>리프트 보조 실패 (2021):</strong> 침대에서 의자로 이동 중 이송 로봇이 환자를 떨어뜨려 어깨 부상 발생</li>
            <li><strong>낙상 감지 오류 (2022):</strong> 노인 사용자 낙상, 시스템이 47분간 감지 실패로 의료 대응 지연</li>
            <li><strong>끼임 부상 (2023):</strong> 힘 감지 보정 오류로 식사 지원 중 로봇 팔이 손가락 부상 유발</li>
        </ul>
    </div>

    <h3>위험 수준별 안전 요구사항</h3>
    <table>
        <tr>
            <th>기능</th>
            <th>위험 수준</th>
            <th>실패 영향</th>
            <th>필요한 안전 기능</th>
        </tr>
        <tr>
            <td>약물 투여</td>
            <td>중요</td>
            <td>과다복용, 누락된 복용</td>
            <td>이중 확인, 용량 제한, 감사 추적</td>
        </tr>
        <tr>
            <td>신체 리프팅/이송</td>
            <td>중요</td>
            <td>낙상, 부상</td>
            <td>하중 센서, 비상 정지, 백업 전원</td>
        </tr>
        <tr>
            <td>이동 지원</td>
            <td>높음</td>
            <td>충돌, 낙상</td>
            <td>장애물 감지, 속도 제한, 자동 정지</td>
        </tr>
        <tr>
            <td>낙상 감지</td>
            <td>중요</td>
            <td>응급 대응 지연</td>
            <td>다중 센서 융합, <5초 감지 시간</td>
        </tr>
    </table>

    <h2>2.4 정서적 의존 및 사회적 고립</h2>
    <p>역설적인 과제가 나타납니다: 외로움을 줄이도록 설계된 돌봄 로봇이 사용자가 인간 접촉보다 로봇 상호작용을 선호하면 의도치 않게 사회적 고립을 증가시킬 수 있습니다.</p>

    <pre>
사회적 참여 모니터링 - 균형 잡힌 상호작용 예제
═══════════════════════════════════════════════════════════════

{
  "사회적참여모니터링": {
    "상호작용균형": {
      "인간상호작용": {
        "직접": {
          "가족방문": 3,
          "친구방문": 2,
          "돌봄제공자방문": 5,
          "주간총계": 10
        },
        "원격": {
          "전화발신": 4,
          "전화수신": 7,
          "화상통화가족": 5,
          "화상통화친구": 2,
          "주간총계": 18
        }
      },
      "로봇상호작용": {
        "대화분": 145,
        "교제세션": 12,
        "지원요청": 28,
        "일평균": 20.7
      },
      "균형점수": {
        "현재": 82,
        "임계값": 60,
        "상태": "건강한_균형"
      }
    },
    "의존성경고시스템": {
      "개입계획": {
        "레벨1_부드러운알림": [
          "가족에게 전화 제안",
          "사회 행사 상기",
          "화상 통화 촉진"
        ],
        "레벨2_적극적참여": [
          "필수 가족 체크인 예약",
          "우려 패턴에 대한 가족 알림",
          "로봇 교제 기능 감소"
        ],
        "레벨3_전문의뢰": [
          "정신 건강 전문가 알림",
          "사회복지사 개입",
          "돌봄 계획 조정"
        ]
      }
    }
  }
}
    </pre>

    <div class="info-box">
        <h3>장 요약</h3>
        <p>돌봄 로봇의 현재 과제는 채택에 대한 중요한 장벽입니다:</p>
        <ul>
            <li>모든 이해관계자 그룹에 <strong>신뢰 격차</strong> 존재, 82%의 노인 사용자가 주저</li>
            <li>비디오, 오디오, 건강 데이터 수집에 대한 <strong>개인정보 우려</strong>는 강력한 보호 필요</li>
            <li><strong>신체적 안전</strong>이 가장 중요; 중요한 실패는 부상이나 사망을 초래할 수 있음</li>
            <li><strong>정서적 의존</strong> 위험을 모니터링하고 예방해야 함</li>
            <li><strong>표준 부족</strong>으로 호환되지 않는 고립된 시스템 생성</li>
            <li><strong>높은 비용</strong>으로 가장 필요한 사람들의 접근성 제한</li>
        </ul>
    </div>

    <div class="footer">
        <div class="hongik">弘益人間</div>
        <p>신중한 문제 해결을 통해 널리 인간을 이롭게 함</p>
        <p>WIA 케어봇 표준 v1.0.0 - 2장: 현재 과제</p>
    </div>

    <div class="nav" style="margin-top: 30px;">
        <a href="chapter-01.html">← 이전: 1장</a>
        <a href="index.html">목차</a>
        <a href="chapter-03.html">다음: 3장 - 표준 개요 →</a>
    </div>
</div>
</body>
</html>
EOF

echo "Created chapter-02.html"
