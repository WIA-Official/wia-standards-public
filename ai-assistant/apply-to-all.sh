#!/bin/bash
# 모든 표준 페이지에 AI 모달 적용

BASE="/var/www/wiastandards"
AI_CODE='<!-- WIA AI Assistant -->
<link rel="stylesheet" href="/ai-assistant/wia-ai-modal.css">
<script src="/ai-assistant/wia-ai-modal.js"></script>'

for dir in $BASE/*/; do
    dirname=$(basename "$dir")
    
    # 제외 폴더
    case "$dirname" in
        css|js|fonts|images|docs|backups|data|video|wp|assets|ai-assistant|EN_home|*.backup)
            continue
            ;;
    esac
    
    INDEX="$dir/index.html"
    
    if [ -f "$INDEX" ]; then
        # 이미 적용되어 있는지 확인
        if grep -q "wia-ai-modal" "$INDEX"; then
            echo "⏭️  Already applied: $dirname"
            continue
        fi
        
        # </body> 앞에 AI 코드 삽입
        if grep -q "</body>" "$INDEX"; then
            sed -i "s|</body>|$AI_CODE\n</body>|" "$INDEX"
            echo "✅ Applied: $dirname"
        else
            echo "⚠️  No </body> tag: $dirname"
        fi
    fi
done

echo ""
echo "✅ All standard pages updated!"
