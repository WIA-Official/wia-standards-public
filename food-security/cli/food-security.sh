#!/bin/bash
# WIA food-security CLI Tool
# WIA-FOOD-006 — Food Security
# Version: 1.0.0
# 弘益人間 (Benefit All Humanity)

set -e

STANDARD_NAME="food-security"
STANDARD_ID="WIA-FOOD-006"
VERSION="1.0.0"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

show_help() {
    cat << EOF
WIA ${STANDARD_ID} ${STANDARD_NAME} CLI Tool v${VERSION}

Usage: $(basename "$0") [command] [options]

Commands:
    validate <file>     Validate a food-security report or supply-chain manifest JSON
    fies-template       Print a household FIES indicator JSON skeleton
    supply-template     Print a supply-chain manifest JSON skeleton
    nutrition-template  Print a nutrition adequacy JSON skeleton
    profiles            List supported food-security conformance profiles
    info                Show standard information
    help                Show this help message

Reference frameworks (CITATION-POLICY ALLOW only):
    FAO Food Insecurity Experience Scale (FIES) reporting protocol
    ISO 22005:2007 (traceability in feed and food chain)
    ISO 22000:2018, FSSC 22000 v6 (management systems)
    Codex Alimentarius CAC/GL 60-2006 (traceability principles)
    UN/CEFACT Buy-Ship-Pay reference data model
    OpenAPI Specification 3.1, JSON Schema 2020-12, Sigstore signatures

弘益人間 (Benefit All Humanity)
EOF
}

show_info() {
    echo -e "${BLUE}WIA ${STANDARD_ID} — ${STANDARD_NAME}${NC}"
    echo -e "Version: ${VERSION}"
    echo -e "Scope: cross-vendor exchange of food-security indicators (FIES,"
    echo -e "       availability, access, utilization, stability), supply-chain"
    echo -e "       traceability events, and nutrition adequacy reports."
    echo -e ""
    echo -e "Conformance tiers (see README.md): Surface / Verified / Anchored."
    echo -e ""
    echo -e "弘益人間 (Benefit All Humanity)"
}

show_profiles() {
    cat << EOF
Supported conformance profiles for ${STANDARD_ID}:

  - household       FIES survey instrument profile (8 question battery)
  - regional        regional aggregator with availability/access metrics
  - national        national reporting profile (ministry / statistical agency)
  - relief          humanitarian relief profile (cluster-coordination data)
  - supply          supply-chain trace profile (lot, origin, custody chain)
  - regulator       read-mostly profile for competent authorities

Profile manifests are signed using the same Sigstore key as the SBOM.
See spec/PHASE-4-INTEGRATION.md for the manifest format.
EOF
}

emit_fies_template() {
    cat << 'EOF'
{
  "documentId": "wia-fies-XXXX",
  "standardId": "WIA-FOOD-006",
  "version": "1.0.0",
  "respondentId": "anon-XXXX",
  "geographyIso3166": "KR-11",
  "surveyTimestamp": "2026-04-26T00:00:00Z",
  "responses": {
    "worried": false,
    "unhealthy": false,
    "fewKinds": false,
    "skipped": false,
    "ateLess": false,
    "ranOut": false,
    "hungry": false,
    "wholeDay": false
  },
  "rawScore": 0,
  "rasch": null,
  "interpreter": "FAO-FIES",
  "signedBy": "field-team-id"
}
EOF
}

emit_supply_template() {
    cat << 'EOF'
{
  "documentId": "wia-supply-XXXX",
  "standardId": "WIA-FOOD-006",
  "version": "1.0.0",
  "lotId": "LOT-2026-04-26-A",
  "sku": "SKU-1234",
  "originIso3166": "KR-11",
  "custodyChain": [
    { "step": "harvest", "actorId": "FARM-001", "ts": "2026-04-25T08:00:00Z" },
    { "step": "transport", "actorId": "LOG-002", "ts": "2026-04-25T20:00:00Z" }
  ],
  "traceability": "ISO 22005:2007",
  "manifestHashSha256": "<replace-with-sha256-hex>",
  "issuedAt": "2026-04-26T00:00:00Z"
}
EOF
}

emit_nutrition_template() {
    cat << 'EOF'
{
  "documentId": "wia-nutrition-XXXX",
  "standardId": "WIA-FOOD-006",
  "version": "1.0.0",
  "subjectGroup": "primary-school-K-1-grade-3",
  "ageRange": "6-9",
  "sampleSize": 240,
  "indicator": "MAD",
  "value": 0.78,
  "ciLower": 0.71,
  "ciUpper": 0.84,
  "method": "FAO MAD methodology",
  "issuedAt": "2026-04-26T00:00:00Z"
}
EOF
}

validate_file() {
    local f="$1"
    if [ ! -f "$f" ]; then
        echo -e "${RED}File not found: $f${NC}" >&2
        exit 1
    fi
    if command -v jq >/dev/null 2>&1; then
        local sid
        sid=$(jq -r '.standardId // empty' "$f")
        if [ "$sid" != "${STANDARD_ID}" ]; then
            echo -e "${RED}standardId mismatch: got '${sid}', expected '${STANDARD_ID}'${NC}" >&2
            exit 1
        fi
        echo -e "${GREEN}OK: ${f} declares standardId=${STANDARD_ID}${NC}"
    else
        python3 -c "import json,sys; d=json.load(open('$f')); assert d.get('standardId')=='${STANDARD_ID}', 'standardId mismatch'; print('OK')"
    fi
}

case "${1:-help}" in
    validate)            validate_file "$2" ;;
    fies-template)       emit_fies_template ;;
    supply-template)     emit_supply_template ;;
    nutrition-template)  emit_nutrition_template ;;
    profiles)            show_profiles ;;
    info)                show_info ;;
    help|*)              show_help ;;
esac
