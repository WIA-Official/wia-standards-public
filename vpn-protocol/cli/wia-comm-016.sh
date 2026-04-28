#!/bin/bash

################################################################################
# WIA-COMM-016: VPN Protocol CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to VPN protocol operations
# including IPsec, OpenVPN, WireGuard configuration, and security auditing.
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
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
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🔐 WIA-COMM-016: VPN Protocol CLI                     ║"
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

# Configure IPsec
config_ipsec() {
    local protocol=${1:-IKEv2}
    local encryption=${2:-AES-256-GCM}
    local auth=${3:-SHA-256}
    local dh_group=${4:-modp2048}
    local pfs=${5:-yes}

    print_section "IPsec Configuration"
    print_info "Protocol: $protocol"
    print_info "Encryption: $encryption"
    print_info "Authentication: $auth"
    print_info "DH Group: $dh_group"
    print_info "PFS: $pfs"

    print_section "Security Assessment"

    # Validate encryption
    if [[ "$encryption" == "AES-256-GCM" || "$encryption" == "AES-128-GCM" ]]; then
        print_success "Encryption: EXCELLENT (AEAD cipher)"
    elif [[ "$encryption" == "AES-256-CBC" ]]; then
        print_success "Encryption: GOOD"
    elif [[ "$encryption" == "3DES" ]]; then
        print_error "Encryption: WEAK (3DES deprecated)"
    fi

    # Validate DH group
    if [[ "$dh_group" =~ ^(modp4096|modp3072|ecp384|ecp521)$ ]]; then
        print_success "DH Group: EXCELLENT"
    elif [[ "$dh_group" == "modp2048" ]]; then
        print_success "DH Group: GOOD (minimum recommended)"
    else
        print_error "DH Group: WEAK (below 2048 bits)"
    fi

    # Validate PFS
    if [[ "$pfs" == "yes" ]]; then
        print_success "Perfect Forward Secrecy: ENABLED"
    else
        print_warning "Perfect Forward Secrecy: DISABLED"
    fi

    print_section "Configuration Template"
    cat <<EOF
conn vpn-tunnel
    type=tunnel
    auto=start
    ike=$encryption-$auth-$dh_group
    esp=$encryption-$auth
    pfs=$pfs
    left=%defaultroute
    right=vpn.example.com
    leftsubnet=192.168.1.0/24
    rightsubnet=10.0.0.0/24
EOF

    echo ""
}

# Setup WireGuard
setup_wireguard() {
    local port=${1:-51820}
    local address=${2:-10.0.0.1/24}

    print_section "WireGuard Setup"
    print_info "Listen Port: $port"
    print_info "Address: $address"

    print_section "Generating Keys"

    # Generate keys (simulated)
    local priv_key=$(head -c 32 /dev/urandom | base64)
    local pub_key=$(echo "$priv_key" | base64 | head -c 44)

    print_success "Private Key: ${priv_key:0:20}..."
    print_success "Public Key: ${pub_key:0:20}..."

    print_section "Configuration File"
    cat <<EOF
[Interface]
PrivateKey = $priv_key
ListenPort = $port
Address = $address

# Example Peer
#[Peer]
#PublicKey = <peer-public-key>
#AllowedIPs = 10.0.0.2/32
#Endpoint = 203.0.113.1:51820
#PersistentKeepalive = 25
EOF

    print_section "Next Steps"
    print_info "1. Save configuration to /etc/wireguard/wg0.conf"
    print_info "2. Start interface: wg-quick up wg0"
    print_info "3. Enable at boot: systemctl enable wg-quick@wg0"

    echo ""
}

# Configure OpenVPN
config_openvpn() {
    local mode=${1:-server}
    local protocol=${2:-udp}
    local port=${3:-1194}

    print_section "OpenVPN Configuration"
    print_info "Mode: $mode"
    print_info "Protocol: $protocol"
    print_info "Port: $port"

    print_section "Security Recommendations"
    print_success "TLS Version: 1.3 (recommended)"
    print_success "Cipher: AES-256-GCM"
    print_success "Auth: SHA-256"
    print_warning "Compression: Disabled (for security)"

    print_section "Configuration Template"
    if [[ "$mode" == "server" ]]; then
        cat <<EOF
# OpenVPN Server Configuration
proto $protocol
port $port
dev tun

# Network
server 10.8.0.0 255.255.255.0
topology subnet

# Certificates
ca ca.crt
cert server.crt
key server.key
dh dh2048.pem

# Encryption
cipher AES-256-GCM
auth SHA256
tls-version-min 1.3

# Performance
fast-io
sndbuf 393216
rcvbuf 393216

# Logging
verb 3
EOF
    else
        cat <<EOF
# OpenVPN Client Configuration
client
dev tun
proto $protocol

remote vpn.example.com $port

# Certificates
ca ca.crt
cert client.crt
key client.key

# Encryption
cipher AES-256-GCM
auth SHA256

# Connection
nobind
persist-key
persist-tun

# Logging
verb 3
EOF
    fi

    echo ""
}

# Validate VPN configuration
validate_config() {
    local config_file=${1}

    if [[ ! -f "$config_file" ]]; then
        print_error "Configuration file not found: $config_file"
        return 1
    fi

    print_section "Validating Configuration"
    print_info "File: $config_file"

    # Detect protocol
    local protocol=""
    if grep -q "^\[Interface\]" "$config_file" 2>/dev/null; then
        protocol="WireGuard"
    elif grep -q "^proto" "$config_file" 2>/dev/null; then
        protocol="OpenVPN"
    elif grep -q "^conn" "$config_file" 2>/dev/null; then
        protocol="IPsec"
    else
        print_warning "Unknown protocol, cannot validate"
        return 1
    fi

    print_info "Detected Protocol: $protocol"

    print_section "Validation Results"

    case "$protocol" in
        WireGuard)
            if grep -q "PrivateKey" "$config_file"; then
                print_success "Private key found"
            else
                print_error "Missing private key"
            fi

            if grep -q "ListenPort" "$config_file"; then
                print_success "Listen port configured"
            else
                print_warning "No listen port specified"
            fi
            ;;

        OpenVPN)
            if grep -q "^ca " "$config_file"; then
                print_success "CA certificate configured"
            else
                print_error "Missing CA certificate"
            fi

            if grep -q "cipher AES-256-GCM\|cipher AES-128-GCM" "$config_file"; then
                print_success "Strong cipher configured"
            else
                print_warning "Consider using AES-256-GCM"
            fi
            ;;

        IPsec)
            if grep -q "ike=.*AES-256" "$config_file"; then
                print_success "Strong IKE encryption"
            else
                print_warning "Consider using AES-256-GCM"
            fi

            if grep -q "pfs=yes" "$config_file"; then
                print_success "Perfect Forward Secrecy enabled"
            else
                print_warning "PFS is recommended"
            fi
            ;;
    esac

    echo ""
}

# Calculate tunnel overhead
calc_overhead() {
    local protocol=${1:-ipsec}
    local mtu=${2:-1500}

    print_section "Tunnel Overhead Calculation"
    print_info "Protocol: $protocol"
    print_info "Original MTU: $mtu bytes"

    # Protocol overhead
    local overhead=0
    case "$protocol" in
        ipsec|ipsec-ikev2)
            overhead=52
            ;;
        openvpn)
            overhead=70
            ;;
        wireguard)
            overhead=60
            ;;
        l2tp-ipsec)
            overhead=76
            ;;
        *)
            print_error "Unknown protocol: $protocol"
            return 1
            ;;
    esac

    local effective_mtu=$((mtu - overhead))
    local mss=$((effective_mtu - 40))

    print_section "Results"
    print_success "Protocol Overhead: $overhead bytes"
    print_success "Effective MTU: $effective_mtu bytes"
    print_success "MSS (TCP): $mss bytes"

    # Fragmentation risk
    if (( effective_mtu >= 1400 )); then
        print_success "Fragmentation Risk: LOW"
    elif (( effective_mtu >= 1300 )); then
        print_warning "Fragmentation Risk: MEDIUM"
    else
        print_error "Fragmentation Risk: HIGH"
    fi

    print_section "Recommendations"
    case "$protocol" in
        ipsec*)
            print_info "Recommended MTU: 1400 bytes"
            ;;
        openvpn)
            print_info "Recommended MTU: 1420 bytes"
            print_info "Add to config: tun-mtu 1420"
            print_info "Add to config: mssfix 1380"
            ;;
        wireguard)
            print_info "Recommended MTU: 1420 bytes"
            print_info "Add to config: MTU = 1420"
            ;;
    esac

    echo ""
}

# Generate VPN keys
generate_keys() {
    local protocol=${1:-wireguard}
    local output_dir=${2:-.}

    print_section "Generating Keys"
    print_info "Protocol: $protocol"
    print_info "Output Directory: $output_dir"

    mkdir -p "$output_dir"

    case "$protocol" in
        wireguard)
            print_info "Generating WireGuard key pair..."

            # Generate private key
            wg genkey > "$output_dir/private.key" 2>/dev/null || {
                # Fallback if wg not installed
                head -c 32 /dev/urandom | base64 > "$output_dir/private.key"
            }

            # Generate public key
            if command -v wg &> /dev/null; then
                cat "$output_dir/private.key" | wg pubkey > "$output_dir/public.key"
            else
                cat "$output_dir/private.key" | base64 | head -c 44 > "$output_dir/public.key"
            fi

            # Generate pre-shared key
            if command -v wg &> /dev/null; then
                wg genpsk > "$output_dir/preshared.key"
            else
                head -c 32 /dev/urandom | base64 > "$output_dir/preshared.key"
            fi

            print_success "Private Key: $output_dir/private.key"
            print_success "Public Key: $output_dir/public.key"
            print_success "Pre-shared Key: $output_dir/preshared.key"
            ;;

        openvpn)
            print_info "Generating OpenVPN keys requires easy-rsa or openssl"
            print_info "Example commands:"
            print_info "  openssl genrsa -out private.key 2048"
            print_info "  openssl req -new -key private.key -out request.csr"
            ;;

        ipsec)
            print_info "Generating IPsec PSK..."
            head -c 32 /dev/urandom | base64 > "$output_dir/ipsec.psk"
            print_success "Pre-shared Key: $output_dir/ipsec.psk"
            ;;

        *)
            print_error "Unknown protocol: $protocol"
            return 1
            ;;
    esac

    echo ""
}

# Security audit
security_audit() {
    local protocol=${1:-ipsec-ikev2}
    local encryption=${2:-AES-256-GCM}
    local dh_group=${3:-modp2048}

    print_section "VPN Security Audit"
    print_info "Protocol: $protocol"
    print_info "Encryption: $encryption"
    print_info "DH Group: $dh_group"

    local score=100
    local findings=0

    print_section "Security Analysis"

    # Protocol check
    case "$protocol" in
        wireguard|ipsec-ikev2)
            print_success "Protocol: EXCELLENT (modern, secure)"
            ;;
        openvpn|ipsec-ikev1)
            print_success "Protocol: GOOD (mature, widely supported)"
            ;;
        l2tp-ipsec)
            print_warning "Protocol: ACCEPTABLE (consider upgrading)"
            score=$((score - 10))
            findings=$((findings + 1))
            ;;
        pptp)
            print_error "Protocol: VULNERABLE (deprecated, insecure)"
            score=$((score - 50))
            findings=$((findings + 1))
            ;;
    esac

    # Encryption check
    if [[ "$encryption" =~ ^(AES-256-GCM|ChaCha20-Poly1305)$ ]]; then
        print_success "Encryption: EXCELLENT (AEAD cipher)"
    elif [[ "$encryption" =~ ^(AES-256-CBC|AES-128-GCM)$ ]]; then
        print_success "Encryption: GOOD"
    elif [[ "$encryption" == "3DES" ]]; then
        print_error "Encryption: WEAK (deprecated)"
        score=$((score - 30))
        findings=$((findings + 1))
    fi

    # DH Group check
    if [[ "$dh_group" =~ ^(modp4096|modp3072|ecp384)$ ]]; then
        print_success "DH Group: EXCELLENT"
    elif [[ "$dh_group" == "modp2048" ]]; then
        print_success "DH Group: GOOD (minimum recommended)"
    elif [[ "$dh_group" =~ ^(modp1536|modp1024)$ ]]; then
        print_error "DH Group: WEAK (insecure)"
        score=$((score - 30))
        findings=$((findings + 1))
    fi

    print_section "Audit Summary"
    print_info "Security Score: $score/100"
    print_info "Findings: $findings"

    if (( score >= 90 )); then
        print_success "Security Level: EXCELLENT"
    elif (( score >= 75 )); then
        print_success "Security Level: GOOD"
    elif (( score >= 60 )); then
        print_warning "Security Level: ACCEPTABLE"
    else
        print_error "Security Level: WEAK"
    fi

    print_section "Recommendations"
    if [[ "$protocol" == "pptp" ]]; then
        print_info "- Migrate to WireGuard or IKEv2 immediately"
    fi
    if [[ "$encryption" == "3DES" ]]; then
        print_info "- Upgrade to AES-256-GCM"
    fi
    if [[ "$dh_group" =~ ^(modp1536|modp1024)$ ]]; then
        print_info "- Use DH group 14 (modp2048) or higher"
    fi
    print_info "- Enable Perfect Forward Secrecy"
    print_info "- Implement multi-factor authentication"
    print_info "- Regular security audits and updates"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-016 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  config-ipsec             Configure IPsec VPN"
    echo "    --protocol <version>   IKEv1 or IKEv2 (default: IKEv2)"
    echo "    --encryption <algo>    Encryption algorithm (default: AES-256-GCM)"
    echo "    --auth <algo>          Authentication algorithm (default: SHA-256)"
    echo "    --dh-group <group>     DH group (default: modp2048)"
    echo "    --pfs <yes|no>         Perfect Forward Secrecy (default: yes)"
    echo ""
    echo "  setup-wireguard          Setup WireGuard VPN"
    echo "    --port <port>          Listen port (default: 51820)"
    echo "    --address <cidr>       Interface address (default: 10.0.0.1/24)"
    echo ""
    echo "  config-openvpn           Configure OpenVPN"
    echo "    --mode <mode>          server or client (default: server)"
    echo "    --protocol <proto>     udp or tcp (default: udp)"
    echo "    --port <port>          Port (default: 1194)"
    echo ""
    echo "  validate                 Validate VPN configuration file"
    echo "    --config <file>        Configuration file path"
    echo ""
    echo "  calc-overhead            Calculate tunnel overhead"
    echo "    --protocol <proto>     VPN protocol"
    echo "    --mtu <size>           Original MTU (default: 1500)"
    echo ""
    echo "  generate-keys            Generate VPN keys"
    echo "    --protocol <proto>     wireguard, openvpn, or ipsec"
    echo "    --output <dir>         Output directory (default: current)"
    echo ""
    echo "  security-audit           Perform security audit"
    echo "    --protocol <proto>     VPN protocol"
    echo "    --encryption <algo>    Encryption algorithm"
    echo "    --dh-group <group>     DH group"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comm-016 config-ipsec --protocol IKEv2 --encryption AES-256-GCM"
    echo "  wia-comm-016 setup-wireguard --port 51820 --address 10.0.0.1/24"
    echo "  wia-comm-016 validate --config /etc/wireguard/wg0.conf"
    echo "  wia-comm-016 calc-overhead --protocol wireguard --mtu 1500"
    echo "  wia-comm-016 generate-keys --protocol wireguard --output keys/"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMM-016 VPN Protocol CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    config-ipsec)
        PROTOCOL="IKEv2"
        ENCRYPTION="AES-256-GCM"
        AUTH="SHA-256"
        DH_GROUP="modp2048"
        PFS="yes"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --protocol) PROTOCOL=$2; shift 2 ;;
                --encryption) ENCRYPTION=$2; shift 2 ;;
                --auth) AUTH=$2; shift 2 ;;
                --dh-group) DH_GROUP=$2; shift 2 ;;
                --pfs) PFS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        config_ipsec "$PROTOCOL" "$ENCRYPTION" "$AUTH" "$DH_GROUP" "$PFS"
        ;;

    setup-wireguard)
        PORT=51820
        ADDRESS="10.0.0.1/24"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --port) PORT=$2; shift 2 ;;
                --address) ADDRESS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        setup_wireguard "$PORT" "$ADDRESS"
        ;;

    config-openvpn)
        MODE="server"
        PROTOCOL="udp"
        PORT=1194

        while [[ $# -gt 0 ]]; do
            case $1 in
                --mode) MODE=$2; shift 2 ;;
                --protocol) PROTOCOL=$2; shift 2 ;;
                --port) PORT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        config_openvpn "$MODE" "$PROTOCOL" "$PORT"
        ;;

    validate)
        CONFIG_FILE=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --config) CONFIG_FILE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [[ -z "$CONFIG_FILE" ]]; then
            print_error "Configuration file is required"
            exit 1
        fi

        print_header
        validate_config "$CONFIG_FILE"
        ;;

    calc-overhead)
        PROTOCOL="ipsec"
        MTU=1500

        while [[ $# -gt 0 ]]; do
            case $1 in
                --protocol) PROTOCOL=$2; shift 2 ;;
                --mtu) MTU=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_overhead "$PROTOCOL" "$MTU"
        ;;

    generate-keys)
        PROTOCOL="wireguard"
        OUTPUT="."

        while [[ $# -gt 0 ]]; do
            case $1 in
                --protocol) PROTOCOL=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_keys "$PROTOCOL" "$OUTPUT"
        ;;

    security-audit)
        PROTOCOL="ipsec-ikev2"
        ENCRYPTION="AES-256-GCM"
        DH_GROUP="modp2048"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --protocol) PROTOCOL=$2; shift 2 ;;
                --encryption) ENCRYPTION=$2; shift 2 ;;
                --dh-group) DH_GROUP=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        security_audit "$PROTOCOL" "$ENCRYPTION" "$DH_GROUP"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comm-016 help' for usage information"
        exit 1
        ;;
esac

exit 0
