#!/bin/bash
# WIA Pet Health Passport Ebook Uploader
# 경고 숨김 버전

EBOOK_DIR="/var/www/wiastandards/pet-health-passport/ebook"
WP_PATH="/var/www/wiabooks"
BOOK_TAG="WIA Pet Health Passport"

cd "$WP_PATH"

echo "=== WIA Ebook Uploader ==="
echo "Book: $BOOK_TAG"
echo ""

# 1. 태그 생성 (에러 출력만 숨김)
echo "[1/3] 태그 생성..."
sudo -u apache wp term create post_tag "$BOOK_TAG" --porcelain --allow-root 2>/dev/null || true
TAG_ID=$(sudo -u apache wp term list post_tag --name="$BOOK_TAG" --field=term_id --allow-root 2>/dev/null)
echo "태그 ID: $TAG_ID"

# 2. 챕터 파일 처리
echo ""
echo "[2/3] 챕터 업로드..."

for chapter_file in "$EBOOK_DIR"/chapter-*.html; do
    [ -f "$chapter_file" ] || continue
    
    filename=$(basename "$chapter_file")
    chapter_num=$(echo "$filename" | sed 's/chapter-0*\([0-9]*\)\.html/\1/')
    
    # 제목 추출 (간단한 방식)
    title=$(grep '<h1 class="chapter-title"' "$chapter_file" | sed 's/.*>\([^<]*\)<.*/\1/' | head -1)
    [ -z "$title" ] && title="Chapter $chapter_num"
    
    echo ""
    echo "처리 중: $filename"
    echo "  제목: $title"
    
    # article.content 부분 추출
    content=$(awk '/<article class="content">/,/<\/article>/' "$chapter_file" | sed '1d;$d')
    
    # 임시 파일에 저장
    temp_content="/tmp/chapter_${chapter_num}.html"
    echo "$content" > "$temp_content"
    
    # 포스트 생성 (경고 숨김)
    POST_ID=$(sudo -u apache wp post create "$temp_content" \
        --post_title="$title" \
        --post_status=private \
        --post_type=post \
        --porcelain \
        --allow-root 2>/dev/null)
    
    if [ -n "$POST_ID" ] && [ "$POST_ID" -gt 0 ] 2>/dev/null; then
        # 태그 할당
        sudo -u apache wp post term set "$POST_ID" post_tag "$TAG_ID" --allow-root 2>/dev/null
        echo "  [OK] 생성됨: ID=$POST_ID"
    else
        echo "  [ERROR] 생성 실패"
    fi
    
    rm -f "$temp_content"
done

# 3. 결과 확인
echo ""
echo "[3/3] 결과 확인..."
echo ""
sudo -u apache wp post list --post_status=any --tag="$BOOK_TAG" --fields=ID,post_title,post_status --allow-root 2>/dev/null

echo ""
echo "완료!"
