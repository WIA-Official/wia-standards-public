#!/bin/bash

################################################################################
# WIA-COMM-020: Network Protocol CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Standards Group
#
# 弘익人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to network protocol operations
# including IP address calculations, packet analysis, and protocol configuration.
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
    echo "║         📡 WIA-COMM-020: Network Protocol CLI Tool            ║"
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

# Calculate IPv4 subnet
calc_subnet() {
    local ip=${1:-192.168.1.0}
    local cidr=${2:-24}

    print_section "IPv4 Subnet Calculation"
    print_info "IP Address: $ip"
    print_info "CIDR: /$cidr"

    # Calculate subnet mask
    local mask_bits=$((0xFFFFFFFF << (32 - cidr)))
    local mask=$(printf "%d.%d.%d.%d" \
        $((mask_bits >> 24 & 0xFF)) \
        $((mask_bits >> 16 & 0xFF)) \
        $((mask_bits >> 8 & 0xFF)) \
        $((mask_bits & 0xFF)))

    # Convert IP to number
    IFS='.' read -r i1 i2 i3 i4 <<< "$ip"
    local ip_num=$(((i1 << 24) + (i2 << 16) + (i3 << 8) + i4))

    # Calculate network address
    local net_num=$((ip_num & mask_bits))
    local network=$(printf "%d.%d.%d.%d" \
        $((net_num >> 24 & 0xFF)) \
        $((net_num >> 16 & 0xFF)) \
        $((net_num >> 8 & 0xFF)) \
        $((net_num & 0xFF)))

    # Calculate broadcast address
    local wildcard_bits=$((0xFFFFFFFF >> cidr))
    local bcast_num=$((net_num | wildcard_bits))
    local broadcast=$(printf "%d.%d.%d.%d" \
        $((bcast_num >> 24 & 0xFF)) \
        $((bcast_num >> 16 & 0xFF)) \
        $((bcast_num >> 8 & 0xFF)) \
        $((bcast_num & 0xFF)))

    # Calculate first and last host
    local first_num=$((net_num + 1))
    local first_host=$(printf "%d.%d.%d.%d" \
        $((first_num >> 24 & 0xFF)) \
        $((first_num >> 16 & 0xFF)) \
        $((first_num >> 8 & 0xFF)) \
        $((first_num & 0xFF)))

    local last_num=$((bcast_num - 1))
    local last_host=$(printf "%d.%d.%d.%d" \
        $((last_num >> 24 & 0xFF)) \
        $((last_num >> 16 & 0xFF)) \
        $((last_num >> 8 & 0xFF)) \
        $((last_num & 0xFF)))

    # Calculate host count
    local host_count=$((wildcard_bits - 1))

    print_section "Results"
    print_success "Network Address: $network"
    print_info "Subnet Mask: $mask"
    print_info "Broadcast: $broadcast"
    print_info "First Host: $first_host"
    print_info "Last Host: $last_host"
    print_info "Total Hosts: $host_count"

    # Determine IP class
    if [ $i1 -lt 128 ]; then
        print_info "IP Class: A"
    elif [ $i1 -lt 192 ]; then
        print_info "IP Class: B"
    elif [ $i1 -lt 224 ]; then
        print_info "IP Class: C"
    fi

    # Check if private
    if [ $i1 -eq 10 ] || \
       ([ $i1 -eq 172 ] && [ $i2 -ge 16 ] && [ $i2 -le 31 ]) || \
       ([ $i1 -eq 192 ] && [ $i2 -eq 168 ]); then
        print_success "Type: Private IP"
    else
        print_warning "Type: Public IP"
    fi

    echo ""
}

# Analyze packet
analyze_packet() {
    local protocol=${1:-tcp}

    print_section "Packet Analysis"
    print_info "Protocol: ${protocol^^}"

    case "${protocol,,}" in
        tcp)
            print_section "TCP Header Structure"
            print_info "Source Port:      16 bits (0-65535)"
            print_info "Dest Port:        16 bits (0-65535)"
            print_info "Sequence Number:  32 bits"
            print_info "ACK Number:       32 bits"
            print_info "Data Offset:      4 bits"
            print_info "Flags:            6 bits (URG|ACK|PSH|RST|SYN|FIN)"
            print_info "Window Size:      16 bits"
            print_info "Checksum:         16 bits"
            print_info "Urgent Pointer:   16 bits"
            print_success "Total Header Size: 20 bytes minimum"
            ;;
        udp)
            print_section "UDP Header Structure"
            print_info "Source Port:      16 bits (0-65535)"
            print_info "Dest Port:        16 bits (0-65535)"
            print_info "Length:           16 bits"
            print_info "Checksum:         16 bits"
            print_success "Total Header Size: 8 bytes"
            ;;
        ipv4)
            print_section "IPv4 Header Structure"
            print_info "Version:          4 bits (4)"
            print_info "IHL:              4 bits (5-15)"
            print_info "TOS/DSCP:         8 bits"
            print_info "Total Length:     16 bits"
            print_info "Identification:   16 bits"
            print_info "Flags:            3 bits (DF|MF)"
            print_info "Fragment Offset:  13 bits"
            print_info "TTL:              8 bits"
            print_info "Protocol:         8 bits (TCP=6, UDP=17)"
            print_info "Header Checksum:  16 bits"
            print_info "Source IP:        32 bits"
            print_info "Dest IP:          32 bits"
            print_success "Total Header Size: 20 bytes minimum"
            ;;
        ipv6)
            print_section "IPv6 Header Structure"
            print_info "Version:          4 bits (6)"
            print_info "Traffic Class:    8 bits"
            print_info "Flow Label:       20 bits"
            print_info "Payload Length:   16 bits"
            print_info "Next Header:      8 bits"
            print_info "Hop Limit:        8 bits"
            print_info "Source IP:        128 bits"
            print_info "Dest IP:          128 bits"
            print_success "Total Header Size: 40 bytes"
            ;;
        *)
            print_error "Unknown protocol: $protocol"
            return 1
            ;;
    esac

    echo ""
}

# Test IPv6 connectivity
test_ipv6() {
    local host=${1:-2001:4860:4860::8888}

    print_section "IPv6 Connectivity Test"
    print_info "Target: $host"

    if ping6 -c 4 "$host" > /dev/null 2>&1; then
        print_success "IPv6 connectivity: WORKING"
    else
        print_error "IPv6 connectivity: FAILED"
        print_warning "Check IPv6 configuration and routing"
    fi

    echo ""
}

# Configure BGP
config_bgp() {
    local asn=${1:-65000}
    local peer=${2:-10.0.0.1}
    local remote_asn=${3:-65001}

    print_section "BGP Configuration"
    print_info "Local ASN: $asn"
    print_info "Peer IP: $peer"
    print_info "Remote ASN: $remote_asn"

    print_section "Configuration Template"
    echo -e "${GRAY}"
    echo "router bgp $asn"
    echo "  bgp router-id 1.1.1.1"
    echo "  neighbor $peer remote-as $remote_asn"
    echo "  neighbor $peer update-source Loopback0"
    echo "  !"
    echo "  address-family ipv4"
    echo "    neighbor $peer activate"
    echo "    network 0.0.0.0"
    echo "  exit-address-family"
    echo -e "${RESET}"

    if [ "$asn" -eq "$remote_asn" ]; then
        print_success "Peering Type: iBGP (Internal)"
        print_info "Same AS - use route reflectors or full mesh"
    else
        print_success "Peering Type: eBGP (External)"
        print_info "Different AS - standard BGP peering"
    fi

    echo ""
}

# Validate routing table
validate_routes() {
    print_section "Routing Table Validation"

    # Check if routing tools are available
    if command -v ip &> /dev/null; then
        print_info "Displaying IPv4 routes:"
        ip route show | head -10
        echo ""
        print_success "Route table validated"
    elif command -v netstat &> /dev/null; then
        print_info "Displaying routes:"
        netstat -rn | head -10
        echo ""
        print_success "Route table validated"
    else
        print_warning "No routing tools found (ip or netstat)"
    fi

    echo ""
}

# Generate protocol config
generate_config() {
    local protocol=${1:-ospf}
    local router_id=${2:-1.1.1.1}

    print_section "Protocol Configuration Generator"
    print_info "Protocol: ${protocol^^}"
    print_info "Router ID: $router_id"

    print_section "Generated Configuration"
    echo -e "${GRAY}"

    case "${protocol,,}" in
        ospf)
            echo "router ospf 1"
            echo "  router-id $router_id"
            echo "  network 192.168.1.0 0.0.0.255 area 0"
            echo "  network 10.0.0.0 0.255.255.255 area 1"
            echo "  passive-interface default"
            echo "  no passive-interface GigabitEthernet0/0"
            ;;
        bgp)
            echo "router bgp 65000"
            echo "  bgp router-id $router_id"
            echo "  neighbor 10.0.0.2 remote-as 65001"
            echo "  neighbor 10.0.0.2 update-source Loopback0"
            echo "  address-family ipv4"
            echo "    network 192.168.0.0"
            echo "  exit-address-family"
            ;;
        eigrp)
            echo "router eigrp 100"
            echo "  eigrp router-id $router_id"
            echo "  network 192.168.1.0 0.0.0.255"
            echo "  network 10.0.0.0 0.255.255.255"
            echo "  passive-interface default"
            ;;
        *)
            print_error "Unknown protocol: $protocol"
            ;;
    esac

    echo -e "${RESET}"
    print_success "Configuration generated"
    echo ""
}

# Traceroute
traceroute_path() {
    local target=${1:-8.8.8.8}
    local protocol=${2:-icmp}

    print_section "Traceroute to $target"
    print_info "Protocol: ${protocol^^}"

    if command -v traceroute &> /dev/null; then
        traceroute -m 15 "$target"
        print_success "Traceroute completed"
    elif command -v tracert &> /dev/null; then
        tracert -h 15 "$target"
        print_success "Traceroute completed"
    else
        print_error "Traceroute tool not found"
    fi

    echo ""
}

# Test HTTP/2
test_http2() {
    local url=${1:-https://www.google.com}

    print_section "HTTP/2 Connection Test"
    print_info "URL: $url"

    if command -v curl &> /dev/null; then
        if curl -sI --http2 "$url" | grep -q "HTTP/2"; then
            print_success "HTTP/2: SUPPORTED"
            curl -sI --http2 "$url" | head -5
        else
            print_warning "HTTP/2: NOT SUPPORTED or fallback to HTTP/1.1"
        fi
    else
        print_error "curl not found"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comm-020 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-subnet              Calculate IPv4 subnet information"
    echo "    --ip <address>         IP address (default: 192.168.1.0)"
    echo "    --cidr <prefix>        CIDR prefix length (default: 24)"
    echo ""
    echo "  analyze-packet           Analyze protocol packet structure"
    echo "    --protocol <type>      Protocol type: tcp, udp, ipv4, ipv6"
    echo ""
    echo "  test-ipv6                Test IPv6 connectivity"
    echo "    --host <address>       IPv6 host to test (default: 2001:4860:4860::8888)"
    echo ""
    echo "  config-bgp               Generate BGP configuration"
    echo "    --asn <number>         Local AS number (default: 65000)"
    echo "    --peer <ip>            Peer IP address (default: 10.0.0.1)"
    echo "    --remote-asn <number>  Remote AS number (default: 65001)"
    echo ""
    echo "  validate-routes          Validate routing table"
    echo ""
    echo "  generate-config          Generate protocol configuration"
    echo "    --protocol <type>      Protocol: ospf, bgp, eigrp"
    echo "    --router-id <id>       Router ID (default: 1.1.1.1)"
    echo ""
    echo "  traceroute               Trace route to destination"
    echo "    --target <host>        Target host (default: 8.8.8.8)"
    echo "    --protocol <type>      Protocol: icmp, udp, tcp"
    echo ""
    echo "  test-http2               Test HTTP/2 support"
    echo "    --url <url>            URL to test (default: https://www.google.com)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comm-020 calc-subnet --ip 192.168.1.0 --cidr 24"
    echo "  wia-comm-020 analyze-packet --protocol tcp"
    echo "  wia-comm-020 config-bgp --asn 65000 --peer 10.0.0.1 --remote-asn 65001"
    echo "  wia-comm-020 test-ipv6 --host 2001:4860:4860::8888"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMM-020 Network Protocol CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    calc-subnet)
        IP="192.168.1.0"
        CIDR=24

        while [[ $# -gt 0 ]]; do
            case $1 in
                --ip) IP=$2; shift 2 ;;
                --cidr) CIDR=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_subnet "$IP" "$CIDR"
        ;;

    analyze-packet)
        PROTOCOL="tcp"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --protocol) PROTOCOL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_packet "$PROTOCOL"
        ;;

    test-ipv6)
        HOST="2001:4860:4860::8888"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --host) HOST=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        test_ipv6 "$HOST"
        ;;

    config-bgp)
        ASN=65000
        PEER="10.0.0.1"
        REMOTE_ASN=65001

        while [[ $# -gt 0 ]]; do
            case $1 in
                --asn) ASN=$2; shift 2 ;;
                --peer) PEER=$2; shift 2 ;;
                --remote-asn) REMOTE_ASN=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        config_bgp "$ASN" "$PEER" "$REMOTE_ASN"
        ;;

    validate-routes)
        print_header
        validate_routes
        ;;

    generate-config)
        PROTOCOL="ospf"
        ROUTER_ID="1.1.1.1"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --protocol) PROTOCOL=$2; shift 2 ;;
                --router-id) ROUTER_ID=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_config "$PROTOCOL" "$ROUTER_ID"
        ;;

    traceroute)
        TARGET="8.8.8.8"
        PROTOCOL="icmp"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --target) TARGET=$2; shift 2 ;;
                --protocol) PROTOCOL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        traceroute_path "$TARGET" "$PROTOCOL"
        ;;

    test-http2)
        URL="https://www.google.com"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --url) URL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        test_http2 "$URL"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comm-020 help' for usage information"
        exit 1
        ;;
esac

exit 0
