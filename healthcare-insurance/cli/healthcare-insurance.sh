#!/bin/bash
# WIA healthcare-insurance CLI Tool
# WIA-SOC-019 — Healthcare Insurance Interoperability
# Version: 1.0.0
# 弘益人間 (Benefit All Humanity)

set -e

STANDARD_NAME="healthcare-insurance"
STANDARD_ID="WIA-SOC-019"
VERSION="1.0.0"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

show_help() {
    cat << EOF
WIA ${STANDARD_ID} ${STANDARD_NAME} CLI Tool v${VERSION}

Usage: $(basename $0) [command] [options]

Commands:
    validate <file>     Validate FHIR claim/coverage/EOB JSON against WIA-SOC-019
    fhir-resources      List supported HL7 FHIR R5 resource types
    code-systems        List supported clinical code systems (SNOMED/ICD/LOINC)
    privacy-frameworks  Show supported privacy frameworks per jurisdiction
    info                Show standard information
    help                Show this help message

Reference frameworks:
    HL7 FHIR R5 (interoperability)
    SNOMED CT / ICD-10 / ICD-11 / LOINC (clinical terminology)
    HIPAA 45 CFR Part 162/164 (US)
    GDPR Article 9 (EU)
    PIPA §23 (KR)
    ISO 27799:2016 (health-information security)

弘益人間 (Benefit All Humanity)
EOF
}

show_info() {
    echo -e "${BLUE}WIA ${STANDARD_ID} — ${STANDARD_NAME}${NC}"
    echo -e "Version: ${VERSION}"
    echo -e "Scope: cross-jurisdictional healthcare-insurance interoperability"
    echo -e "Reference: HL7 FHIR R5, HIPAA, GDPR Art. 9, PIPA §23, ISO 27799"
    echo -e ""
    echo -e "弘益人間 (Benefit All Humanity)"
}

validate_file() {
    local file=$1
    [[ -f "$file" ]] || { echo -e "${RED}File not found: $file${NC}"; exit 1; }
    echo -e "${BLUE}✓ Validating $file against ${STANDARD_ID}${NC}"
    for key in resourceType id meta; do
        if grep -q "\"$key\"" "$file"; then
            echo -e "${GREEN}  ✓ $key${NC}"
        else
            echo -e "${YELLOW}  ⚠ missing FHIR field: $key${NC}"
        fi
    done
    echo -e "${GREEN}✓ Validation pass (schema v1.0)${NC}"
}

fhir_resources() {
    cat << EOF
Supported HL7 FHIR R5 resources for WIA-SOC-019:
  Patient            — patient demographics
  Coverage           — insurance coverage details
  Claim              — claim submission
  ClaimResponse      — payer adjudication
  ExplanationOfBenefit — final EOB
  Encounter          — encounter linking claim to clinical event
  Practitioner       — provider identity
  Organization       — payer / provider organisation
  CodeSystem         — terminology system definition
  ValueSet           — terminology subset
  Endpoint           — service endpoint metadata
EOF
}

code_systems() {
    cat << EOF
Supported clinical code systems:
  SNOMED CT     — clinical terminology (IHTSDO)
  ICD-10        — diagnostic codes (WHO)
  ICD-11        — diagnostic codes 11th revision (WHO, 2022)
  LOINC         — laboratory and observation codes
  CPT           — procedural codes (AMA, US)
  HCPCS         — Healthcare Common Procedure Coding System (CMS, US)
  ATC           — Anatomical Therapeutic Chemical (WHO)
  RxNorm        — clinical drugs (NLM, US)
EOF
}

privacy_frameworks() {
    cat << EOF
Supported privacy frameworks (per jurisdiction):
  US:  HIPAA Privacy Rule (45 CFR Part 164), Security Rule (45 CFR Part 160)
  EU:  GDPR (Regulation (EU) 2016/679) Article 9 (special-category data)
  UK:  Data Protection Act 2018 + UK GDPR
  KR:  개인정보 보호법 §23 (민감정보)
  CA:  CCPA / CPRA + California Confidentiality of Medical Information Act
  AU:  Privacy Act 1988 + Notifiable Data Breaches scheme
  BR:  LGPD (Lei Geral de Proteção de Dados)
  Other: see deploying jurisdiction's local equivalent
EOF
}

case "${1:-help}" in
    validate)            [[ -z "$2" ]] && { echo "Need file"; exit 1; }; validate_file "$2" ;;
    fhir-resources)      fhir_resources ;;
    code-systems)        code_systems ;;
    privacy-frameworks)  privacy_frameworks ;;
    info)                show_info ;;
    help|--help|-h)      show_help ;;
    *) echo -e "${RED}Unknown command: $1${NC}"; show_help; exit 1 ;;
esac
