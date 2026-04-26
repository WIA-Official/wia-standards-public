#!/bin/bash
# WIA food-safety CLI Tool
# WIA-FOOD-005 — Food Safety
# Version: 1.0.0
# 弘益人間 (Benefit All Humanity)

set -e

STANDARD_NAME="food-safety"
STANDARD_ID="WIA-FOOD-005"
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
    validate <file>     Validate a HACCP plan or test-result JSON document
    haccp-template      Print a HACCP plan JSON skeleton
    test-template       Print a laboratory test-result JSON skeleton
    incident-template   Print a food-safety incident JSON skeleton
    profiles            List supported food-safety conformance profiles
    info                Show standard information
    help                Show this help message

Reference frameworks (CITATION-POLICY ALLOW only):
    Codex Alimentarius CXC 1-1969 (HACCP general principles)
    ISO 22000:2018 (food-safety management systems)
    FSSC 22000 v6 (scheme certification)
    ISO/IEC 17025:2017 (testing & calibration laboratories)
    ISO/IEC 27001:2022 (information security for incident records)
    OpenAPI Specification 3.1, JSON Schema 2020-12

弘益人間 (Benefit All Humanity)
EOF
}

show_info() {
    echo -e "${BLUE}WIA ${STANDARD_ID} — ${STANDARD_NAME}${NC}"
    echo -e "Version: ${VERSION}"
    echo -e "Scope: cross-vendor exchange of HACCP plans, lab test results,"
    echo -e "       inspection scores, certifications, and incident reports."
    echo -e ""
    echo -e "Conformance tiers (see README.md): Surface / Verified / Anchored."
    echo -e ""
    echo -e "弘益人間 (Benefit All Humanity)"
}

show_profiles() {
    cat << EOF
Supported conformance profiles for ${STANDARD_ID}:

  - producer       single-site primary producer
  - processor      multi-line food processor with HACCP plan
  - distributor    cold-chain distributor with traceability obligations
  - retailer       in-store testing and incident-reporting profile
  - regulator      read-mostly profile for competent authorities
  - aggregator     directory profile that aggregates across producers

Profile manifests are signed using the same Sigstore key as the SBOM.
See spec/PHASE-4-INTEGRATION.md Annex I for the manifest format.
EOF
}

emit_haccp_template() {
    cat << 'EOF'
{
  "documentId": "wia-haccp-XXXX",
  "standardId": "WIA-FOOD-005",
  "version": "1.0.0",
  "siteId": "SITE-XXX",
  "scope": "describe scope of HACCP plan",
  "hazardAnalysis": [
    {
      "step": "receiving",
      "hazards": [
        { "type": "biological", "agent": "Salmonella", "controlled": true }
      ]
    }
  ],
  "criticalControlPoints": [
    {
      "ccpId": "CCP-1",
      "step": "cooking",
      "criticalLimit": { "minTempCelsius": 75, "minHoldSeconds": 15 },
      "monitoringFrequencySec": 300,
      "correctiveAction": "discard batch and root-cause-analysis"
    }
  ],
  "verificationCadenceDays": 90,
  "approvedAt": "2026-04-26T00:00:00Z",
  "signedBy": "team-lead@example.com"
}
EOF
}

emit_test_template() {
    cat << 'EOF'
{
  "documentId": "wia-test-XXXX",
  "standardId": "WIA-FOOD-005",
  "version": "1.0.0",
  "labId": "LAB-XXX",
  "labAccreditation": "ISO/IEC 17025:2017",
  "sampleId": "SAMPLE-XXX",
  "method": "ISO 6579-1:2017",
  "analyte": "Salmonella spp.",
  "result": "not detected",
  "unit": "CFU/25g",
  "uncertainty": null,
  "issuedAt": "2026-04-26T00:00:00Z",
  "signedBy": "lab-director@example.com"
}
EOF
}

emit_incident_template() {
    cat << 'EOF'
{
  "documentId": "wia-incident-XXXX",
  "standardId": "WIA-FOOD-005",
  "version": "1.0.0",
  "incidentId": "INC-XXX",
  "severity": "high",
  "category": "outbreak",
  "affectedSku": ["SKU-1234"],
  "affectedLot": ["LOT-2026-04-26-A"],
  "geographyIso3166": ["KR-11"],
  "rootCauseStatus": "investigating",
  "recallScope": "regional",
  "reportedAt": "2026-04-26T00:00:00Z",
  "reportedBy": "qa-lead@example.com"
}
EOF
}

validate_file() {
    local f="$1"
    if [ ! -f "$f" ]; then
        echo -e "${RED}File not found: $f${NC}" >&2
        exit 1
    fi
    if ! command -v jq >/dev/null 2>&1; then
        echo -e "${YELLOW}jq not installed; structural validation only via python${NC}" >&2
        python3 -c "import json,sys; json.load(open('$f')); print('JSON valid')"
        return
    fi
    local std_id
    std_id=$(jq -r '.standardId // empty' "$f")
    if [ "$std_id" != "${STANDARD_ID}" ]; then
        echo -e "${RED}standardId mismatch: got '${std_id}', expected '${STANDARD_ID}'${NC}" >&2
        exit 1
    fi
    echo -e "${GREEN}OK: ${f} declares standardId=${STANDARD_ID}${NC}"
}

case "${1:-help}" in
    validate)          validate_file "$2" ;;
    haccp-template)    emit_haccp_template ;;
    test-template)     emit_test_template ;;
    incident-template) emit_incident_template ;;
    profiles)          show_profiles ;;
    info)              show_info ;;
    help|*)            show_help ;;
esac
