#!/bin/bash

echo "=========================================="
echo "WIA Standards Quality Audit Report"
echo "Date: $(date)"
echo "=========================================="
echo ""

STANDARDS=(
  "WIA-DEFI"
  "WIA-ESPORTS"
  "WIA-EVTOL"
  "WIA-EXOSKELETON"
  "WIA-FINTECH"
  "WIA-GAME"
  "WIA-HAPTIC"
  "WIA-HEALTH"
  "WIA-HOME"
  "WIA-HYDROPONICS"
  "WIA-INSURTECH"
)

TOTAL_PASS=0
TOTAL_FAIL=0

for STD in "${STANDARDS[@]}"; do
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "📋 Standard: $STD"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  
  PASS=0
  FAIL=0
  
  # 1. File Count Check (19 files required)
  echo ""
  echo "1️⃣  File Count Check (Required: 19 files)"
  FILE_COUNT=$(find "$STD" -type f | wc -l)
  echo "   Found: $FILE_COUNT files"
  
  # Spec files (4)
  SPEC_COUNT=$(find "$STD/spec" -name "*.md" 2>/dev/null | wc -l)
  if [ $SPEC_COUNT -eq 4 ]; then
    echo "   ✅ Spec: 4/4 files"
    ((PASS++))
  else
    echo "   ❌ Spec: $SPEC_COUNT/4 files"
    ((FAIL++))
  fi
  
  # API files (7: types, index, validators, utils, package.json, tsconfig.json, README)
  API_TS_COUNT=$(find "$STD/api/typescript/src" -name "*.ts" 2>/dev/null | wc -l)
  API_JSON_COUNT=$(find "$STD/api/typescript" -maxdepth 1 -name "*.json" 2>/dev/null | wc -l)
  API_README=$([ -f "$STD/api/typescript/README.md" ] && echo 1 || echo 0)
  API_TOTAL=$((API_TS_COUNT + API_JSON_COUNT + API_README))
  
  if [ $API_TOTAL -ge 7 ]; then
    echo "   ✅ API: $API_TOTAL/7 files (${API_TS_COUNT} TS + ${API_JSON_COUNT} JSON + ${API_README} README)"
    ((PASS++))
  else
    echo "   ❌ API: $API_TOTAL/7 files"
    ((FAIL++))
  fi
  
  # CLI (1)
  CLI_COUNT=$(find "$STD/cli" -name "*.sh" 2>/dev/null | wc -l)
  if [ $CLI_COUNT -ge 1 ]; then
    echo "   ✅ CLI: $CLI_COUNT file(s)"
    ((PASS++))
  else
    echo "   ❌ CLI: $CLI_COUNT files"
    ((FAIL++))
  fi
  
  # Ebook EN (9)
  EBOOK_COUNT=$(find "$STD/ebook/en" -name "*.html" 2>/dev/null | wc -l)
  if [ $EBOOK_COUNT -eq 9 ]; then
    echo "   ✅ Ebook EN: 9/9 files"
    ((PASS++))
  else
    echo "   ❌ Ebook EN: $EBOOK_COUNT/9 files"
    ((FAIL++))
  fi
  
  # Root files (README + install.sh = 2)
  ROOT_FILES=0
  [ -f "$STD/README.md" ] && ((ROOT_FILES++))
  [ -f "$STD/install.sh" ] && ((ROOT_FILES++))
  
  if [ $ROOT_FILES -eq 2 ]; then
    echo "   ✅ Root: 2/2 files (README.md, install.sh)"
    ((PASS++))
  else
    echo "   ❌ Root: $ROOT_FILES/2 files"
    ((FAIL++))
  fi
  
  # 2. File Size Check
  echo ""
  echo "2️⃣  File Size Check"
  
  # Spec PHASE1 (10KB+)
  if [ -f "$STD/spec/$STD-PHASE1.md" ]; then
    SIZE=$(stat -f%z "$STD/spec/$STD-PHASE1.md" 2>/dev/null || stat -c%s "$STD/spec/$STD-PHASE1.md" 2>/dev/null)
    if [ $SIZE -ge 10240 ]; then
      echo "   ✅ PHASE1: ${SIZE} bytes (≥10KB)"
      ((PASS++))
    else
      echo "   ⚠️  PHASE1: ${SIZE} bytes (<10KB)"
      ((FAIL++))
    fi
  else
    echo "   ❌ PHASE1: File not found"
    ((FAIL++))
  fi
  
  # Spec PHASE2 (15KB+)
  if [ -f "$STD/spec/$STD-PHASE2.md" ]; then
    SIZE=$(stat -f%z "$STD/spec/$STD-PHASE2.md" 2>/dev/null || stat -c%s "$STD/spec/$STD-PHASE2.md" 2>/dev/null)
    if [ $SIZE -ge 15360 ]; then
      echo "   ✅ PHASE2: ${SIZE} bytes (≥15KB)"
      ((PASS++))
    else
      echo "   ⚠️  PHASE2: ${SIZE} bytes (<15KB)"
      ((FAIL++))
    fi
  else
    echo "   ❌ PHASE2: File not found"
    ((FAIL++))
  fi
  
  # Ebook index (10KB+)
  if [ -f "$STD/ebook/en/index.html" ]; then
    SIZE=$(stat -f%z "$STD/ebook/en/index.html" 2>/dev/null || stat -c%s "$STD/ebook/en/index.html" 2>/dev/null)
    if [ $SIZE -ge 10240 ]; then
      echo "   ✅ Ebook index: ${SIZE} bytes (≥10KB)"
      ((PASS++))
    else
      echo "   ⚠️  Ebook index: ${SIZE} bytes (<10KB)"
      ((FAIL++))
    fi
  else
    echo "   ❌ Ebook index: File not found"
    ((FAIL++))
  fi
  
  # Ebook chapters (15KB+ each)
  CHAPTER_PASS=0
  CHAPTER_FAIL=0
  for i in {01..08}; do
    if [ -f "$STD/ebook/en/chapter-$i.html" ]; then
      SIZE=$(stat -f%z "$STD/ebook/en/chapter-$i.html" 2>/dev/null || stat -c%s "$STD/ebook/en/chapter-$i.html" 2>/dev/null)
      if [ $SIZE -ge 15360 ]; then
        ((CHAPTER_PASS++))
      else
        ((CHAPTER_FAIL++))
      fi
    fi
  done
  
  if [ $CHAPTER_PASS -eq 8 ]; then
    echo "   ✅ Ebook chapters: 8/8 chapters ≥15KB"
    ((PASS++))
  else
    echo "   ⚠️  Ebook chapters: $CHAPTER_PASS/8 chapters ≥15KB"
    ((FAIL++))
  fi
  
  # 3. Content Quality Check
  echo ""
  echo "3️⃣  Content Quality Check"
  
  # 弘익人間 philosophy
  PHILOSOPHY_COUNT=$(grep -r "弘益人間" "$STD" 2>/dev/null | wc -l)
  if [ $PHILOSOPHY_COUNT -ge 5 ]; then
    echo "   ✅ Philosophy: Found in $PHILOSOPHY_COUNT locations"
    ((PASS++))
  else
    echo "   ⚠️  Philosophy: Found in $PHILOSOPHY_COUNT locations (<5)"
    ((FAIL++))
  fi
  
  # CLI executable
  CLI_FILE=$(find "$STD/cli" -name "*.sh" 2>/dev/null | head -1)
  if [ -n "$CLI_FILE" ] && [ -x "$CLI_FILE" ]; then
    echo "   ✅ CLI: Executable"
    ((PASS++))
  else
    echo "   ❌ CLI: Not executable"
    ((FAIL++))
  fi
  
  # install.sh executable
  if [ -f "$STD/install.sh" ] && [ -x "$STD/install.sh" ]; then
    echo "   ✅ install.sh: Executable"
    ((PASS++))
  else
    echo "   ❌ install.sh: Not executable"
    ((FAIL++))
  fi
  
  # TypeScript types
  if [ -f "$STD/api/typescript/src/types.ts" ]; then
    INTERFACE_COUNT=$(grep -c "^export interface" "$STD/api/typescript/src/types.ts" 2>/dev/null)
    if [ $INTERFACE_COUNT -ge 3 ]; then
      echo "   ✅ Types: $INTERFACE_COUNT interfaces defined"
      ((PASS++))
    else
      echo "   ⚠️  Types: $INTERFACE_COUNT interfaces (<3 expected)"
      ((FAIL++))
    fi
  else
    echo "   ❌ Types: File not found"
    ((FAIL++))
  fi
  
  # Summary
  TOTAL=$((PASS + FAIL))
  PERCENTAGE=$((PASS * 100 / TOTAL))
  
  echo ""
  echo "📊 Summary: $PASS/$TOTAL checks passed ($PERCENTAGE%)"
  
  if [ $PERCENTAGE -ge 80 ]; then
    echo "   ✅ OVERALL: PASS"
  elif [ $PERCENTAGE -ge 60 ]; then
    echo "   ⚠️  OVERALL: NEEDS IMPROVEMENT"
  else
    echo "   ❌ OVERALL: FAIL"
  fi
  
  TOTAL_PASS=$((TOTAL_PASS + PASS))
  TOTAL_FAIL=$((TOTAL_FAIL + FAIL))
  
  echo ""
done

echo "=========================================="
echo "📈 FINAL REPORT"
echo "=========================================="
echo ""
echo "Total Standards Audited: ${#STANDARDS[@]}"
echo "Total Checks: $((TOTAL_PASS + TOTAL_FAIL))"
echo "Passed: $TOTAL_PASS ✅"
echo "Failed: $TOTAL_FAIL ❌"
echo "Success Rate: $((TOTAL_PASS * 100 / (TOTAL_PASS + TOTAL_FAIL)))%"
echo ""
echo "=========================================="
