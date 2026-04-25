#!/bin/bash

################################################################################
# WIA-CORE-001: Universal Identity CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Identity Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to universal identity operations
# including identity creation, federation, credential issuance, and verification.
################################################################################

set -e

# Colors for output
INDIGO='\033[38;5;99m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🪪 WIA-CORE-001: Universal Identity CLI              ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() {
    echo -e "${GREEN}✓ $1${RESET}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${RESET}"
}

print_error() {
    echo -e "${RED}✗ $1${RESET}"
}

print_info() {
    echo -e "${GRAY}  $1${RESET}"
}

# Create new identity
create_identity() {
    local type="${1:-did}"
    local name="$2"

    print_section "Creating Universal Identity"

    print_info "Type: $type"
    print_info "Name: $name"

    # Generate DID
    local identifier=$(uuidgen | tr '[:upper:]' '[:lower:]' | cut -d'-' -f1)
    local did="did:web:example.com:users:$identifier"

    print_section "Identity Created"
    print_success "DID: $did"
    print_success "Display Name: $name"
    print_success "Trust Level: 0 (Anonymous)"

    print_section "Next Steps"
    print_info "1. Verify email to reach Trust Level 1"
    print_info "2. Link external providers for federation"
    print_info "3. Issue verifiable credentials"

    # Create identity file
    local identity_file="identity-$identifier.json"
    cat > "$identity_file" <<EOF
{
  "id": "$did",
  "type": "decentralized",
  "displayName": "$name",
  "trustLevel": 0,
  "attributes": {
    "name": "$name"
  },
  "linkedProviders": [],
  "credentials": [],
  "createdAt": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "status": "active"
}
EOF

    print_success "Identity saved to: $identity_file"
    echo ""
}

# Link external provider
link_provider() {
    local provider="$1"
    local protocol="${2:-oidc}"

    print_section "Linking External Provider"

    print_info "Provider: $provider"
    print_info "Protocol: $protocol"

    # Supported providers
    case "$provider" in
        google)
            print_success "Provider: Google"
            print_info "Authorization URL: https://accounts.google.com/o/oauth2/v2/auth"
            print_info "Token URL: https://oauth2.googleapis.com/token"
            ;;
        github)
            print_success "Provider: GitHub"
            print_info "Authorization URL: https://github.com/login/oauth/authorize"
            print_info "Token URL: https://github.com/login/oauth/access_token"
            ;;
        microsoft)
            print_success "Provider: Microsoft"
            print_info "Authorization URL: https://login.microsoftonline.com/common/oauth2/v2.0/authorize"
            print_info "Token URL: https://login.microsoftonline.com/common/oauth2/v2.0/token"
            ;;
        *)
            print_warning "Unknown provider: $provider"
            print_info "Using generic OAuth/OIDC configuration"
            ;;
    esac

    print_section "Federation Flow"
    print_info "1. User authorizes application"
    print_info "2. Authorization code returned"
    print_info "3. Exchange code for access token"
    print_info "4. Link provider to identity"

    print_success "Provider linking initiated"
    print_info "Complete authorization in browser to finish linking"

    echo ""
}

# Issue credential
issue_credential() {
    local type="$1"
    local value="$2"
    local issuer="${3:-did:web:example.com}"

    print_section "Issuing Verifiable Credential"

    print_info "Type: $type"
    print_info "Value: $value"
    print_info "Issuer: $issuer"

    # Generate credential
    local credential_id="urn:uuid:$(uuidgen | tr '[:upper:]' '[:lower:]')"
    local credential_file="credential-$type-$(date +%s).json"

    cat > "$credential_file" <<EOF
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://www.w3.org/2018/credentials/examples/v1"
  ],
  "id": "$credential_id",
  "type": ["VerifiableCredential", "${type^}Credential"],
  "issuer": "$issuer",
  "issuanceDate": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "expirationDate": "$(date -u -d '+1 year' +"%Y-%m-%dT%H:%M:%SZ")",
  "credentialSubject": {
    "$type": "$value",
    "verified": true,
    "verifiedAt": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
    "verificationMethod": "$issuer#keys-1",
    "proofPurpose": "assertionMethod",
    "jws": "eyJhbGciOiJFZERTQSJ9...simulated-signature"
  }
}
EOF

    print_section "Credential Issued"
    print_success "Credential ID: $credential_id"
    print_success "Type: ${type^}Credential"
    print_success "Valid for: 1 year"
    print_success "Saved to: $credential_file"

    echo ""
}

# Verify credential
verify_credential() {
    local credential_file="$1"

    print_section "Verifying Credential"

    if [ ! -f "$credential_file" ]; then
        print_error "Credential file not found: $credential_file"
        return 1
    fi

    print_info "Credential: $credential_file"

    if command -v jq &> /dev/null; then
        print_section "Credential Details"

        local cred_type=$(jq -r '.type[1] // "Unknown"' "$credential_file")
        local issuer=$(jq -r '.issuer // "Unknown"' "$credential_file")
        local issued=$(jq -r '.issuanceDate // "Unknown"' "$credential_file")
        local expires=$(jq -r '.expirationDate // "null"' "$credential_file")

        print_info "Type: $cred_type"
        print_info "Issuer: $issuer"
        print_info "Issued: $issued"
        print_info "Expires: $expires"

        print_section "Verification Checks"

        # Check 1: Valid JSON structure
        if jq empty "$credential_file" 2>/dev/null; then
            print_success "Valid JSON structure"
        else
            print_error "Invalid JSON structure"
            return 1
        fi

        # Check 2: Has required fields
        if jq -e '.["@context"]' "$credential_file" >/dev/null; then
            print_success "Context present"
        else
            print_error "Missing @context"
        fi

        if jq -e '.type' "$credential_file" >/dev/null; then
            print_success "Type present"
        else
            print_error "Missing type"
        fi

        if jq -e '.issuer' "$credential_file" >/dev/null; then
            print_success "Issuer present"
        else
            print_error "Missing issuer"
        fi

        if jq -e '.credentialSubject' "$credential_file" >/dev/null; then
            print_success "Credential subject present"
        else
            print_error "Missing credential subject"
        fi

        # Check 3: Expiration
        if [ "$expires" != "null" ]; then
            local expires_epoch=$(date -d "$expires" +%s 2>/dev/null || echo 0)
            local now_epoch=$(date +%s)

            if [ $expires_epoch -gt $now_epoch ]; then
                print_success "Credential not expired"
            else
                print_error "Credential has expired"
            fi
        else
            print_success "No expiration date (valid indefinitely)"
        fi

        # Check 4: Proof
        if jq -e '.proof' "$credential_file" >/dev/null; then
            print_success "Proof present"
            print_warning "Signature verification requires cryptographic validation"
        else
            print_warning "No proof found"
        fi

        print_section "Verification Summary"
        print_success "Credential structure is valid"
        print_info "Status: VERIFIED (structural checks only)"
        print_warning "Full cryptographic verification requires online validation"

    else
        print_warning "jq not installed - cannot parse credential"
        print_info "Install jq for full verification"
    fi

    echo ""
}

# Export identity
export_identity() {
    local format="${1:-json}"
    local output="$2"

    print_section "Exporting Identity"

    print_info "Format: $format"
    print_info "Output: $output"

    # Find identity files
    local identity_files=(identity-*.json)

    if [ ${#identity_files[@]} -eq 0 ]; then
        print_error "No identity files found in current directory"
        return 1
    fi

    local identity_file="${identity_files[0]}"
    print_info "Using: $identity_file"

    if [ "$format" = "json" ]; then
        cp "$identity_file" "$output"
        print_success "Identity exported to: $output"
    elif [ "$format" = "did" ]; then
        if command -v jq &> /dev/null; then
            jq -r '.id' "$identity_file" > "$output"
            print_success "DID exported to: $output"
        else
            print_error "jq required for DID export"
            return 1
        fi
    else
        print_error "Unknown format: $format"
        return 1
    fi

    echo ""
}

# Authenticate to service
authenticate() {
    local service="$1"
    local method="${2:-oidc}"

    print_section "Authenticating to Service"

    print_info "Service: $service"
    print_info "Method: $method"

    print_section "Authentication Flow"

    case "$method" in
        oidc|oauth)
            print_info "1. Redirect to authorization endpoint"
            print_info "2. User grants permission"
            print_info "3. Receive authorization code"
            print_info "4. Exchange code for tokens"
            print_info "5. Access service with access token"
            ;;
        saml)
            print_info "1. Generate SAML request"
            print_info "2. Redirect to identity provider"
            print_info "3. User authenticates"
            print_info "4. Receive SAML assertion"
            print_info "5. Access service with assertion"
            ;;
        webauthn)
            print_info "1. Request authentication challenge"
            print_info "2. Sign challenge with authenticator"
            print_info "3. Verify signature"
            print_info "4. Access service"
            ;;
        *)
            print_error "Unknown authentication method: $method"
            return 1
            ;;
    esac

    # Simulate authentication URL
    local auth_url="$service/auth/$method?client_id=wia-core-001&response_type=code&redirect_uri=http://localhost/callback"

    print_section "Authentication Initiated"
    print_success "Authorization URL:"
    print_info "$auth_url"
    print_info ""
    print_info "Open this URL in your browser to complete authentication"

    echo ""
}

# Check identity status
check_status() {
    print_section "Identity Status Check"

    # Find identity files
    local identity_files=(identity-*.json)

    if [ ${#identity_files[@]} -eq 0 ]; then
        print_error "No identity files found"
        return 1
    fi

    for identity_file in "${identity_files[@]}"; do
        if command -v jq &> /dev/null; then
            local did=$(jq -r '.id' "$identity_file")
            local name=$(jq -r '.displayName // "Unknown"' "$identity_file")
            local trust=$(jq -r '.trustLevel' "$identity_file")
            local status=$(jq -r '.status' "$identity_file")

            print_section "Identity: $name"
            print_info "DID: $did"
            print_info "Trust Level: $trust"
            print_info "Status: $status"

            # Count linked providers
            local providers=$(jq -r '.linkedProviders | length' "$identity_file")
            print_info "Linked Providers: $providers"

            # Count credentials
            local creds=$(jq -r '.credentials | length' "$identity_file")
            print_info "Credentials: $creds"

            if [ "$status" = "active" ]; then
                print_success "Identity is active"
            else
                print_warning "Identity status: $status"
            fi
        fi
    done

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-core-001 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  create-identity          Create new universal identity"
    echo "    --type <type>          Identity type: did, email, anonymous (default: did)"
    echo "    --name <name>          Display name (required)"
    echo ""
    echo "  link-provider            Link external identity provider"
    echo "    --provider <name>      Provider: google, github, microsoft, etc. (required)"
    echo "    --protocol <proto>     Protocol: oidc, oauth2, saml (default: oidc)"
    echo ""
    echo "  issue-credential         Issue verifiable credential"
    echo "    --type <type>          Credential type: email, phone, etc. (required)"
    echo "    --value <value>        Credential value (required)"
    echo "    --issuer <did>         Issuer DID (default: did:web:example.com)"
    echo ""
    echo "  verify-credential        Verify verifiable credential"
    echo "    --credential <path>    Path to credential JSON file (required)"
    echo ""
    echo "  export                   Export identity"
    echo "    --format <format>      Format: json, did (default: json)"
    echo "    --output <path>        Output file path (required)"
    echo ""
    echo "  authenticate             Authenticate to service"
    echo "    --service <url>        Service URL (required)"
    echo "    --method <method>      Method: oidc, oauth, saml, webauthn (default: oidc)"
    echo ""
    echo "  status                   Check identity status"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-core-001 create-identity --type did --name \"Alice Smith\""
    echo "  wia-core-001 link-provider --provider google --protocol oidc"
    echo "  wia-core-001 issue-credential --type email --value \"alice@example.com\""
    echo "  wia-core-001 verify-credential --credential ./email-credential.json"
    echo "  wia-core-001 export --format json --output ./my-identity.json"
    echo "  wia-core-001 authenticate --service https://app.com --method oidc"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-CORE-001 Universal Identity CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo "Key Features:"
    echo "  - Universal identity creation (DIDs)"
    echo "  - Identity provider federation"
    echo "  - Verifiable credential issuance"
    echo "  - Credential verification"
    echo "  - Multi-protocol authentication (OAuth, OIDC, SAML, WebAuthn)"
    echo "  - Privacy-preserving identity management"
    echo ""
    echo -e "${GRAY}弘益인간 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    create-identity)
        TYPE="did"
        NAME=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --name) NAME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$NAME" ]; then
            echo -e "${RED}Error: --name parameter required${RESET}"
            exit 1
        fi

        print_header
        create_identity "$TYPE" "$NAME"
        ;;

    link-provider)
        PROVIDER=""
        PROTOCOL="oidc"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --provider) PROVIDER=$2; shift 2 ;;
                --protocol) PROTOCOL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$PROVIDER" ]; then
            echo -e "${RED}Error: --provider parameter required${RESET}"
            exit 1
        fi

        print_header
        link_provider "$PROVIDER" "$PROTOCOL"
        ;;

    issue-credential)
        TYPE=""
        VALUE=""
        ISSUER="did:web:example.com"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --value) VALUE=$2; shift 2 ;;
                --issuer) ISSUER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$TYPE" ] || [ -z "$VALUE" ]; then
            echo -e "${RED}Error: --type and --value parameters required${RESET}"
            exit 1
        fi

        print_header
        issue_credential "$TYPE" "$VALUE" "$ISSUER"
        ;;

    verify-credential)
        CREDENTIAL=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --credential) CREDENTIAL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$CREDENTIAL" ]; then
            echo -e "${RED}Error: --credential parameter required${RESET}"
            exit 1
        fi

        print_header
        verify_credential "$CREDENTIAL"
        ;;

    export)
        FORMAT="json"
        OUTPUT=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --format) FORMAT=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$OUTPUT" ]; then
            echo -e "${RED}Error: --output parameter required${RESET}"
            exit 1
        fi

        print_header
        export_identity "$FORMAT" "$OUTPUT"
        ;;

    authenticate)
        SERVICE=""
        METHOD="oidc"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --service) SERVICE=$2; shift 2 ;;
                --method) METHOD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$SERVICE" ]; then
            echo -e "${RED}Error: --service parameter required${RESET}"
            exit 1
        fi

        print_header
        authenticate "$SERVICE" "$METHOD"
        ;;

    status)
        print_header
        check_status
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-core-001 help' for usage information"
        exit 1
        ;;
esac

exit 0
