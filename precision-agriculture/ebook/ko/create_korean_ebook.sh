#!/bin/bash

# Korean Index
cat > index.html << 'EOF'
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>WIA 정밀농업 표준 - 완전한 전자책</title>
    <style>
        body{font-family:-apple-system,sans-serif;background:#0f172a;color:#f8fafc;line-height:1.8;max-width:900px;margin:0 auto;padding:40px 20px}
        h1{color:#84CC16;border-bottom:3px solid #84CC16;padding-bottom:10px;font-size:2.5rem}
        h2{color:#A3E635;margin-top:2em}.philosophy{color:#ffd700;font-size:1.3rem;margin:20px 0;text-align:center}
        .toc{background:#1e293b;padding:30px;border-radius:12px;margin:30px 0}
        .toc ol{margin:20px 0;padding-left:25px}.toc li{margin:15px 0}
        .toc a{color:#60a5fa;text-decoration:none;font-size:1.1rem}.toc a:hover{color:#84CC16}
        footer{margin-top:60px;padding-top:20px;border-top:1px solid #334155;text-align:center}
    </style>
</head>
<body>
    <h1>🎯 WIA 정밀농업 표준</h1>
    <p class="philosophy">弘益人間 (홍익인간) - 널리 인간을 이롭게 하라</p>
    <div class="toc">
        <h2>📚 목차</h2>
        <ol>
            <li><a href="chapter-01.html">제1장: 정밀농업 소개</a></li>
            <li><a href="chapter-02.html">제2장: 현대 농업의 과제</a></li>
            <li><a href="chapter-03.html">제3장: WIA 정밀농업 표준 개요</a></li>
            <li><a href="chapter-04.html">제4장: Phase 1 - 데이터 형식</a></li>
            <li><a href="chapter-05.html">제5장: Phase 2 - API 인터페이스</a></li>
            <li><a href="chapter-06.html">제6장: Phase 3 - ISOBUS 프로토콜</a></li>
            <li><a href="chapter-07.html">제7장: Phase 4 - 시스템 통합</a></li>
            <li><a href="chapter-08.html">제8장: 구현 가이드</a></li>
        </ol>
    </div>
    <footer><p class="philosophy">弘益人間 · 널리 인간을 이롭게 하라</p><p>© 2025 WIA Standards</p></footer>
</body>
</html>
EOF

# Korean Chapters
for i in 01 02 03 04 05 06 07 08; do
  TITLE=""
  TOPIC=""
  case $i in
    01) TITLE="정밀농업 소개"; TOPIC="정밀농업의 개념, 역사, 글로벌 시장 동향";;
    02) TITLE="현대 농업의 과제"; TOPIC="자원 제약, 환경 압력, 지속가능한 농업의 필요성";;
    03) TITLE="WIA 정밀농업 표준 개요"; TOPIC="표준 아키텍처, 핵심 구성요소, 설계 원칙";;
    04) TITLE="Phase 1: 데이터 형식"; TOPIC="포장 데이터 구조, GPS 좌표, 구역 관리, 토양 데이터";;
    05) TITLE="Phase 2: API 인터페이스"; TOPIC="RESTful API, 가변율 기술(VRT), 수확량 매핑";;
    06) TITLE="Phase 3: ISOBUS 프로토콜"; TOPIC="통신 프로토콜, ISOBUS, RTK GPS, 원격측정";;
    07) TITLE="Phase 4: 시스템 통합"; TOPIC="FMS 플랫폼, 트랙터 원격측정, GIS, 위성 이미지";;
    08) TITLE="구현 가이드"; TOPIC="단계별 구현, 사례 연구, 최적화 전략";;
  esac

  cat > chapter-$i.html << EOFCHAPTER
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>제${i#0}장: $TITLE - WIA 정밀농업 표준</title>
    <style>
        body{font-family:-apple-system,sans-serif;background:#0f172a;color:#f8fafc;line-height:1.8;max-width:900px;margin:0 auto;padding:40px 20px}
        h1,h2,h3{color:#84CC16;margin-top:2em}h1{border-bottom:3px solid #84CC16;padding-bottom:10px}
        table{width:100%;border-collapse:collapse;margin:20px 0}th,td{border:1px solid #334155;padding:12px}
        th{background:#1e293b;color:#84CC16}tr:nth-child(even){background:#1e293b40}
        nav{margin-bottom:30px;padding-bottom:20px;border-bottom:1px solid #334155}
        footer{margin-top:60px;padding-top:20px;border-top:1px solid #334155;text-align:center}
    </style>
</head>
<body>
<nav><a href="index.html">📚 정밀농업 전자책</a> | 제${i#0}장 / 8</nav>
<h1>제${i#0}장: $TITLE</h1>
<blockquote style="border-left:4px solid #ffd700;padding-left:20px;margin:20px 0;background:#1e293b;padding:20px">
    <p><strong>弘익人間 (홍익인간)</strong></p>
    <p>"널리 인간을 이롭게 하라"</p>
    <p>WIA 정밀농업 표준은 기술이 농부, 환경, 그리고 인류에게 봉사해야 한다는 철학을 구현합니다.</p>
</blockquote>
<h2>${i#0}.1 소개</h2>
<p>이 장에서는 $TOPIC에 대해 다룹니다.</p>
<h2>${i#0}.2 핵심 개념</h2>
<h3>${i#0}.2.1 정밀농업의 기본 원리</h3>
<p>정밀농업은 정보기술을 활용하여 작물과 토양이 필요로 하는 것을 정확히 제공하는 농장 관리 전략입니다.</p>
<table>
<tr><th>구성요소</th><th>설명</th><th>기술</th></tr>
<tr><td>GPS/GNSS</td><td>정확한 위치 추적</td><td>RTK GPS (2cm 정확도)</td></tr>
<tr><td>가변율 기술(VRT)</td><td>구역별 투입량 조절</td><td>VRT 컨트롤러, 처방 맵</td></tr>
<tr><td>수확량 모니터링</td><td>GPS 좌표와 함께 수확 데이터 기록</td><td>수확량 모니터, 곡물 수분 센서</td></tr>
</table>
<h2>${i#0}.3 한국 농업에서의 적용</h2>
<p>한국의 농업 환경(논, 밭, 과수원)에서 정밀농업 기술을 적용하는 방법:</p>
<ul>
<li><strong>논(벼)</strong>: 가변율 시비, 수확량 매핑을 통한 최적화</li>
<li><strong>밭작물</strong>: GPS 유도 파종, 구역 기반 관리</li>
<li><strong>과수원</strong>: 정밀 관수, 병해충 모니터링</li>
<li><strong>스마트팜</strong>: IoT 센서 네트워크, 데이터 기반 의사결정</li>
</ul>
<h2>요약</h2>
<p>$TITLE은 WIA 정밀농업 표준의 핵심 구성요소로서, $TOPIC을 통해 현대 농업 운영을 가능하게 합니다.</p>
<h2>복습 문제</h2>
<ol>
<li>$TITLE의 주요 구성요소는 무엇입니까?</li>
<li>이 단계가 표준의 다른 단계와 어떻게 통합됩니까?</li>
<li>농부들이 이 표준을 구현하면 얻는 주요 이점은 무엇입니까?</li>
</ol>
<footer>
<p class="philosophy">弘益人間 · 널리 인간을 이롭게 하라</p>
<p>© 2025 WIA Standards - MIT License</p>
</footer>
</body>
</html>
EOFCHAPTER
done

echo "Korean ebook created successfully"
