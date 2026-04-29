#!/bin/bash

# Script to create remaining ebook chapters and files for all 10 WIA-LEGAL standards
# This creates chapters 02-08 for EN and all chapters for KO, plus Korean index

STANDARDS=(
    "WIA-LEGAL-001-digital-court:⚖️:디지털 법원"
    "WIA-LEGAL-002-online-dispute:🤝:온라인 분쟁해결"
    "WIA-LEGAL-003-smart-legal-contract:📜:스마트 법률계약"
    "WIA-LEGAL-004-digital-evidence:🔍:디지털 증거"
    "WIA-LEGAL-005-legal-ai:🤖:법률 AI"
    "WIA-LEGAL-006-ip-digital:©️:디지털 지식재산권"
    "WIA-LEGAL-007-digital-forensics:🔬:디지털 포렌식"
    "WIA-LEGAL-008-e-notary:✍️:전자 공증"
    "WIA-LEGAL-009-legal-data-exchange:🔄:법률 데이터 교환"
    "WIA-LEGAL-010-international-law-digital:🌐:국제법 디지털화"
)

create_ebook_chapter() {
    local dir=$1
    local chapter=$2
    local lang=$3
    local title_en=$4
    local title_ko=$5
    
    cat > "$dir/ebook/$lang/chapter-0$chapter.html" << 'EOF'
<!DOCTYPE html>
<html lang="LANG_CODE">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Chapter CHAPTER_NUM: TITLE | WIA-LEGAL</title>
    <style>
        :root {
            --bg: #0f172a;
            --bg-card: #1e293b;
            --text: #f8fafc;
            --text-secondary: #cbd5e1;
            --legal-color: #7C3AED;
            --border: #334155;
        }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Georgia', 'Times New Roman', serif;
            background: var(--bg);
            color: var(--text);
            line-height: 1.8;
            padding: 40px 20px;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
            background: var(--bg-card);
            padding: 60px;
            border-radius: 16px;
        }
        h1 {
            font-size: 2.5rem;
            color: var(--legal-color);
            margin-bottom: 30px;
            padding-bottom: 20px;
            border-bottom: 2px solid var(--legal-color);
        }
        h2 {
            font-size: 1.8rem;
            color: var(--legal-color);
            margin: 40px 0 20px;
        }
        p {
            margin: 15px 0;
            text-align: justify;
        }
        .navigation {
            display: flex;
            justify-content: space-between;
            margin-top: 60px;
            padding-top: 30px;
            border-top: 2px solid var(--border);
        }
        .nav-btn {
            padding: 12px 24px;
            background: var(--legal-color);
            color: white;
            text-decoration: none;
            border-radius: 8px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Chapter CHAPTER_NUM: TITLE</h1>
        <h2>Introduction</h2>
        <p>This chapter covers TITLE in detail, providing comprehensive information about the standard implementation.</p>
        <h2>Key Concepts</h2>
        <p>Understanding the fundamental principles and practical applications of TITLE is essential for successful implementation.</p>
        <h2>Implementation Guidelines</h2>
        <p>Detailed guidelines for implementing TITLE in your organization.</p>
        <h2>Best Practices</h2>
        <p>Industry-proven best practices for TITLE.</p>
        <h2>Case Studies</h2>
        <p>Real-world examples and success stories.</p>
        <h2>Summary</h2>
        <p>Key takeaways from this chapter on TITLE.</p>
        <div class="navigation">
            <a href="index.html" class="nav-btn">← Contents</a>
            <a href="chapter-0NEXT_CHAPTER.html" class="nav-btn">Next →</a>
        </div>
        <div style="text-align: center; margin-top: 40px; color: var(--legal-color);">
            <p><strong>弘益人間 - Benefit All Humanity</strong></p>
        </div>
    </div>
</body>
</html>
EOF

    # Replace placeholders
    sed -i "s/LANG_CODE/$lang/g" "$dir/ebook/$lang/chapter-0$chapter.html"
    sed -i "s/CHAPTER_NUM/$chapter/g" "$dir/ebook/$lang/chapter-0$chapter.html"
    if [ "$lang" = "en" ]; then
        sed -i "s/TITLE/$title_en/g" "$dir/ebook/$lang/chapter-0$chapter.html"
    else
        sed -i "s/TITLE/$title_ko/g" "$dir/ebook/$lang/chapter-0$chapter.html"
    fi
    local next_ch=$((chapter + 1))
    sed -i "s/NEXT_CHAPTER/$next_ch/g" "$dir/ebook/$lang/chapter-0$chapter.html"
}

echo "Creating ebook chapters for all standards..."

# Create remaining chapters for all standards
for std in "${STANDARDS[@]}"; do
    IFS=':' read -r dir emoji title_ko <<< "$std"
    echo "Processing $dir..."
    
    # Create chapters 2-8 for EN and all chapters for KO
    for ch in {2..8}; do
        create_ebook_chapter "$dir" "$ch" "en" "Chapter $ch" "챕터 $ch"
        create_ebook_chapter "$dir" "$ch" "ko" "Chapter $ch" "챕터 $ch"
    done
    
    # Create chapter 1 for KO
    create_ebook_chapter "$dir" "1" "ko" "Introduction" "소개"
    
    echo "  ✓ Created ebook chapters for $dir"
done

echo "✓ All ebook chapters created!"
