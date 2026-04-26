#!/bin/bash
# WIA Standards JSON 생성 스크립트
# 새 표준 추가 시 실행: bash generate-standards.sh

OUTPUT="/var/www/wiastandards/ai-assistant/standards.json"
BASE="/var/www/wiastandards"

echo '{"standards":[' > $OUTPUT

FIRST=true
for dir in $BASE/*/; do
    dirname=$(basename "$dir")
    
    # 제외 폴더
    case "$dirname" in
        css|js|fonts|images|docs|backups|data|video|wp|assets|ai-assistant|EN_home|*.backup)
            continue
            ;;
    esac
    
    # index.html 확인
    if [ -f "$dir/index.html" ]; then
        # 제목 추출
        title=$(grep -o "<title>[^<]*</title>" "$dir/index.html" 2>/dev/null | sed 's/<[^>]*>//g' | head -1)
        
        if [ -z "$title" ]; then
            title="WIA $dirname Standard"
        fi
        
        # 설명 추출 (meta description)
        desc=$(grep -o 'content="[^"]*"' "$dir/index.html" 2>/dev/null | grep -i "description\|content=" | head -1 | sed 's/content="//;s/"$//' | cut -c1-150)
        
        if [ -z "$desc" ]; then
            desc="WIA ${dirname^^} 표준 - World Industry Association"
        fi
        
        if [ "$FIRST" = true ]; then
            FIRST=false
        else
            echo ',' >> $OUTPUT
        fi
        
        # JSON 출력 (특수문자 이스케이프)
        title_escaped=$(echo "$title" | sed 's/"/\\"/g')
        desc_escaped=$(echo "$desc" | sed 's/"/\\"/g')
        
        echo "{\"id\":\"$dirname\",\"title\":\"$title_escaped\",\"desc\":\"$desc_escaped\",\"url\":\"/$dirname/\"}" >> $OUTPUT
    fi
done

echo ']}' >> $OUTPUT

echo "✅ 생성 완료: $OUTPUT"
echo "표준 개수: $(grep -o '"id"' $OUTPUT | wc -l)"
