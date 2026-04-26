#!/bin/bash
# WIA financial-data-exchange CLI Tool
# Version: 1.0.0
# 弘益人間 (Benefit All Humanity)

set -e

STANDARD_NAME="financial-data-exchange"
VERSION="1.0.0"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

show_help() {
    cat <<EOF
WIA ${STANDARD_NAME} CLI Tool v${VERSION}

Usage: $(basename "$0") [command] [options]

Commands:
    validate <file>      Validate a JSON document for WIA conformance
    manifest-template    Print a manifest JSON skeleton
    profiles             List supported conformance profiles
    info                 Show standard information
    help                 Show this help message

Reference frameworks (CITATION-POLICY ALLOW only):
    OpenAPI Specification 3.1, JSON Schema 2020-12
    ISO/IEC 27001:2022 (information security management)
    ISO/IEC 17065:2012 (product certification bodies)
    CycloneDX 1.5 / SPDX 2.3 (software bill of materials)
    Sigstore (DSSE envelope, Rekor transparency log)
    in-toto Attestation Framework 1.0

弘益人間 (Benefit All Humanity)
EOF
}

show_info() {
    echo -e "${BLUE}WIA ${STANDARD_NAME}${NC}"
    echo -e "Version: ${VERSION}"
    echo -e ""
    echo -e "Conformance tiers (see README.md): Surface / Verified / Anchored."
    echo -e "Conformance package follows CycloneDX 1.5 + Sigstore (DSSE + Rekor)."
    echo -e ""
    echo -e "弘益人間 (Benefit All Humanity)"
}

show_profiles() {
    cat <<EOF
Supported conformance profiles for ${STANDARD_NAME}:

  - single-tenant      one runtime per organization
  - multi-tenant       shared runtime, header-based isolation
  - federated          peer-to-peer mesh of runtimes
  - air-gapped         batch evidence package, no live connectivity

Profile manifests are signed using Sigstore (DSSE envelope) and the
signing key is rotated per the deployment policy.
EOF
}

emit_manifest_template() {
    cat <<EOF
{
  "documentId": "wia-${STANDARD_NAME}-XXXX",
  "standardSlug": "${STANDARD_NAME}",
  "version": "${VERSION}",
  "implementation": {
    "name": "REPLACE_ME",
    "build": "git-sha-or-image-digest",
    "sbom": "https://example.com/sbom.cdx.json"
  },
  "profile": { "named": "single-tenant", "version": 1 },
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
        sid=$(jq -r '.standardSlug // empty' "$f")
        if [ "$sid" != "${STANDARD_NAME}" ]; then
            echo -e "${RED}standardSlug mismatch: got '${sid}', expected '${STANDARD_NAME}'${NC}" >&2
            exit 1
        fi
        echo -e "${GREEN}OK: ${f} declares standardSlug=${STANDARD_NAME}${NC}"
    else
        python3 -c "import json,sys; d=json.load(open('$f')); assert d.get('standardSlug')=='${STANDARD_NAME}', 'standardSlug mismatch'; print('OK')"
    fi
}

case "${1:-help}" in
    validate)          validate_file "$2" ;;
    manifest-template) emit_manifest_template ;;
    profiles)          show_profiles ;;
    info)              show_info ;;
    help|*)            show_help ;;
esac
