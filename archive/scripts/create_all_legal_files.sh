#!/bin/bash

# Create all remaining files for WIA-LEGAL standards

STANDARDS_INFO=(
    "WIA-LEGAL-002-online-dispute:🤝:Online Dispute Resolution:온라인 분쟁해결:ODR platform for alternative dispute resolution"
    "WIA-LEGAL-003-smart-legal-contract:📜:Smart Legal Contracts:스마트 법률계약:Blockchain-based self-executing legal agreements"
    "WIA-LEGAL-004-digital-evidence:🔍:Digital Evidence:디지털 증거:Digital evidence authentication and chain of custody"
    "WIA-LEGAL-005-legal-ai:🤖:Legal AI:법률 AI:AI-powered legal research and case analysis"
    "WIA-LEGAL-006-ip-digital:©️:Digital IP Rights:디지털 지식재산권:Digital intellectual property protection and licensing"
    "WIA-LEGAL-007-digital-forensics:🔬:Digital Forensics:디지털 포렌식:Digital forensics standards for legal proceedings"
    "WIA-LEGAL-008-e-notary:✍️:E-Notary:전자 공증:Electronic notarization and apostille services"
    "WIA-LEGAL-009-legal-data-exchange:🔄:Legal Data Exchange:법률 데이터 교환:Standardized legal data exchange between systems"
    "WIA-LEGAL-010-international-law-digital:🌐:International Law Digital:국제법 디지털화:Digital transformation of international legal frameworks"
)

create_korean_ebook_index() {
    local dir=$1
    local emoji=$2
    local title_ko=$3
    
    cat > "$dir/ebook/ko/index.html" << 'EOFKO'
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>TITLE_KO | WIA-LEGAL</title>
    <style>
        :root { --bg: #0f172a; --bg-card: #1e293b; --text: #f8fafc; --legal-color: #7C3AED; }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Noto Sans KR', sans-serif; background: var(--bg); color: var(--text); padding: 40px 20px; }
        .container { max-width: 900px; margin: 0 auto; background: var(--bg-card); padding: 60px; border-radius: 16px; }
        h1 { font-size: 2.5rem; color: var(--legal-color); margin-bottom: 20px; }
        .toc { list-style: none; margin: 30px 0; }
        .toc li { margin: 15px 0; padding: 20px; background: var(--bg); border-radius: 8px; border-left: 4px solid var(--legal-color); }
        .toc a { color: var(--text); text-decoration: none; display: block; }
    </style>
</head>
<body>
    <div class="container">
        <div style="text-align: center; margin-bottom: 40px;">
            <div style="font-size: 100px;">EMOJI</div>
            <h1>TITLE_KO</h1>
            <p>완전한 구현 가이드</p>
        </div>
        <h2 style="color: var(--legal-color); margin: 40px 0 20px;">목차</h2>
        <ul class="toc">
            <li><a href="chapter-01.html">챕터 1: 소개</a></li>
            <li><a href="chapter-02.html">챕터 2: 현재 과제</a></li>
            <li><a href="chapter-03.html">챕터 3: 표준 개요</a></li>
            <li><a href="chapter-04.html">챕터 4: 데이터 형식</a></li>
            <li><a href="chapter-05.html">챕터 5: API 인터페이스</a></li>
            <li><a href="chapter-06.html">챕터 6: 프로토콜</a></li>
            <li><a href="chapter-07.html">챕터 7: 통합</a></li>
            <li><a href="chapter-08.html">챕터 8: 구현 및 인증</a></li>
        </ul>
        <div style="text-align: center; margin-top: 60px; color: var(--legal-color);">
            <p><strong>弘益人間 - 널리 인간을 이롭게 하라</strong></p>
        </div>
    </div>
</body>
</html>
EOFKO

    sed -i "s/EMOJI/$emoji/g" "$dir/ebook/ko/index.html"
    sed -i "s/TITLE_KO/$title_ko/g" "$dir/ebook/ko/index.html"
}

# Create Korean index for WIA-LEGAL-001
create_korean_ebook_index "WIA-LEGAL-001-digital-court" "⚖️" "디지털 법원 표준"

# Create Korean indexes for all other standards
for info in "${STANDARDS_INFO[@]}"; do
    IFS=':' read -r dir emoji title_en title_ko desc <<< "$info"
    create_korean_ebook_index "$dir" "$emoji" "$title_ko 표준"
    echo "✓ Created Korean ebook index for $dir"
done

echo "✓ All Korean ebook indexes created!"
