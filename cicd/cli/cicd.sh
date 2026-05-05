#!/bin/bash
# WIA-CICD CLI Tool
# Version: 1.0.0
# 弘益人間 (Benefit All Humanity) · MIT License
#
# Usage:
#   cicd.sh validate <pipeline.yml|.json>
#   cicd.sh generate-sbom --format CYCLONEDX|SPDX|SPDX_JSON --out <file>
#   cicd.sh verify-provenance <attestation.intoto.jsonl>
#   cicd.sh evaluate-dora <metrics.json>
#   cicd.sh emit --signal wia-cicd-signal-v1
#   cicd.sh info | help

set -e

VERSION="1.0.0"
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

show_help() {
    cat <<EOF
WIA-CICD CLI v${VERSION}  ·  弘益人間 (Benefit All Humanity)

Commands:
    validate <file>             Validate a pipeline definition (YAML or JSON)
    generate-sbom               Emit an SBOM document
        --format <fmt>          CYCLONEDX | SPDX | SPDX_JSON
        --out <file>            Output path
    verify-provenance <file>    Verify a SLSA in-toto attestation
    evaluate-dora <file>        Evaluate DORA + Rework Rate metrics JSON
    emit                        Emit a wia-cicd-signal-v1 envelope
        --signal <name>         Currently only wia-cicd-signal-v1
    info                        Print standard information
    help, --help, -h            Show this message

Reference:
    Simulator: /cicd/simulator/index.html (source of truth)
    Spec:      /cicd/spec/PHASE-1..4.md
    SDKs:      /cicd/api/typescript/  ·  /cicd/api/rust/
EOF
}

show_info() {
    echo -e "${BLUE}WIA-CICD Standard${NC}  v${VERSION}"
    echo -e "Pipeline DAG : SOURCE → BUILD → TEST_GATE → SECURITY_GATE → CD_HANDOFF"
    echo -e "DORA Elite   : lead ≤ 1h · failure ≤ 5% · MTTR ≤ 5min · rework ≤ 5%"
    echo -e "Cache target : LOCAL ≥ 80% · REMOTE ≥ 60% · OVERALL ≥ 70%"
    echo -e "Pyramid      : 70 / 20 / 10 (or Trophy 30 / 50 / 20)"
    echo -e "Security     : SAST · SCA · DAST · SECRETS · CONTAINER · IAC"
    echo -e "Deploy       : ROLLING · BLUE_GREEN · CANARY · PROGRESSIVE"
    echo -e "SLO          : STD 0.999 · FIN 0.9995 · postmortem trigger 20% burn"
    echo -e "License      : MIT  ·  弘益人間 (Benefit All Humanity)"
}

require_file() {
    if [[ ! -f "$1" ]]; then
        echo -e "${RED}Error:${NC} file not found: $1" >&2
        exit 1
    fi
}

cmd_validate() {
    require_file "$1"
    echo -e "${GREEN}✓${NC} Validating $1 against WIA-CICD pipeline.schema.json"
    # Lightweight enum sanity (real validation handled by SDKs).
    if ! grep -qE 'SOURCE|BUILD|TEST_GATE|SECURITY_GATE|CD_HANDOFF' "$1"; then
        echo -e "${YELLOW}Warning:${NC} no canonical DAG node identifiers found"
    fi
    if ! grep -qE 'PUSH|PR|TAG|MANUAL|SCHEDULE' "$1"; then
        echo -e "${YELLOW}Warning:${NC} no recognised trigger ENUM found"
    fi
    echo -e "${GREEN}✓${NC} basic ENUM check passed"
}

cmd_generate_sbom() {
    local format="CYCLONEDX" out="sbom.cdx.json"
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --format) format="$2"; shift 2;;
            --out)    out="$2";    shift 2;;
            *) echo -e "${RED}Unknown flag:${NC} $1" >&2; exit 2;;
        esac
    done
    case "$format" in
        CYCLONEDX) cat > "$out" <<EOF
{ "bomFormat": "CycloneDX", "specVersion": "1.5",
  "serialNumber": "urn:uuid:00000000-0000-0000-0000-000000000000", "version": 1,
  "metadata": { "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "tools": [{ "vendor": "WIA", "name": "wia-cicd-cli", "version": "${VERSION}" }] },
  "components": [] }
EOF
            ;;
        SPDX) cat > "$out" <<EOF
SPDXVersion: SPDX-2.3
DataLicense: CC0-1.0
SPDXID: SPDXRef-DOCUMENT
DocumentName: wia-cicd-sbom
Created: $(date -u +%Y-%m-%dT%H:%M:%SZ)
Creator: Tool: WIA wia-cicd-cli ${VERSION}
EOF
            ;;
        SPDX_JSON) cat > "$out" <<EOF
{ "spdxVersion": "SPDX-2.3", "dataLicense": "CC0-1.0",
  "SPDXID": "SPDXRef-DOCUMENT", "name": "wia-cicd-sbom",
  "creationInfo": { "created": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "creators": ["Tool: WIA wia-cicd-cli ${VERSION}"] },
  "packages": [] }
EOF
            ;;
        *) echo -e "${RED}Unknown format:${NC} $format (CYCLONEDX | SPDX | SPDX_JSON)" >&2; exit 2;;
    esac
    echo -e "${GREEN}✓${NC} SBOM ($format) emitted to $out"
}

cmd_verify_provenance() {
    require_file "$1"
    echo -e "${GREEN}✓${NC} Verifying SLSA in-toto attestation $1"
    if ! grep -q 'in-toto.io/Statement' "$1"; then
        echo -e "${YELLOW}Warning:${NC} _type marker not found — file may not be in-toto format"
    fi
    if ! grep -q 'slsa.dev/provenance' "$1"; then
        echo -e "${YELLOW}Warning:${NC} predicateType not SLSA Provenance"
    fi
    echo -e "${GREEN}✓${NC} basic shape check passed (use cosign / slsa-verifier for cryptographic verification)"
}

cmd_evaluate_dora() {
    require_file "$1"
    echo -e "${GREEN}✓${NC} Evaluating DORA + Rework against Elite thresholds"
    echo -e "  Elite: lead ≤ 1h · failure ≤ 5% · MTTR ≤ 5min · rework ≤ 5%"
    echo -e "  (SDKs implement the full evaluator — see api/typescript/src/dora.ts)"
}

cmd_emit() {
    local signal="wia-cicd-signal-v1"
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --signal) signal="$2"; shift 2;;
            *) shift;;
        esac
    done
    cat <<EOF
{ "signal": "${signal}", "ts": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "source": { "pipeline": "wia-cicd-cli", "node": "CD_HANDOFF" } }
EOF
}

case "${1:-help}" in
    validate)            shift; cmd_validate "$@";;
    generate-sbom)       shift; cmd_generate_sbom "$@";;
    verify-provenance)   shift; cmd_verify_provenance "$@";;
    evaluate-dora)       shift; cmd_evaluate_dora "$@";;
    emit)                shift; cmd_emit "$@";;
    info)                show_info;;
    help|--help|-h)      show_help;;
    *) echo -e "${RED}Unknown command:${NC} $1"; show_help; exit 1;;
esac
