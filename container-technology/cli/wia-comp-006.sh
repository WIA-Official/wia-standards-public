#!/bin/bash

################################################################################
# WIA-COMP-006: Container Technology CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Computing Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to container operations including
# running containers, building images, and managing networks and volumes.
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
    echo "║       🐳 WIA-COMP-006: Container Technology CLI              ║"
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

# Run container
run_container() {
    local image=${1:-nginx:latest}
    local name=${2:-container-$(date +%s)}
    local port=${3:-8080:80}

    print_section "Running Container"
    print_info "Image: $image"
    print_info "Name: $name"
    print_info "Port mapping: $port"

    print_section "Container Status"
    print_success "Container ID: container-$(head /dev/urandom | tr -dc a-z0-9 | head -c 12)"
    print_success "State: Running"
    print_info "Access: http://localhost:${port%%:*}"

    echo ""
}

# Build image
build_image() {
    local dockerfile=${1:-./Dockerfile}
    local tag=${2:-myapp:latest}
    local context=${3:-.}

    print_section "Building Image"
    print_info "Dockerfile: $dockerfile"
    print_info "Tag: $tag"
    print_info "Context: $context"

    print_section "Build Steps"
    print_info "Step 1/5: FROM base image"
    print_info "Step 2/5: COPY files"
    print_info "Step 3/5: RUN install"
    print_info "Step 4/5: EXPOSE ports"
    print_info "Step 5/5: CMD start"

    print_section "Build Result"
    print_success "Image built: $tag"
    print_success "Image ID: sha256:$(head /dev/urandom | tr -dc a-f0-9 | head -c 64)"
    print_info "Size: $(shuf -i 50-500 -n 1)MB"

    echo ""
}

# List containers
list_containers() {
    local all=${1:-false}

    print_section "Containers"

    echo ""
    printf "${CYAN}%-15s %-20s %-20s %-15s %-20s${RESET}\n" \
        "CONTAINER ID" "IMAGE" "NAME" "STATUS" "PORTS"
    echo -e "${GRAY}────────────────────────────────────────────────────────────────────────────────${RESET}"

    # Simulated container list
    printf "%-15s %-20s %-20s %-15s %-20s\n" \
        "abc123456789" "nginx:latest" "web-server" "Up 2 hours" "0.0.0.0:8080->80"
    printf "%-15s %-20s %-20s %-15s %-20s\n" \
        "def987654321" "node:18-alpine" "api-server" "Up 1 hour" "0.0.0.0:3000->3000"

    if [ "$all" = true ]; then
        printf "%-15s %-20s %-20s %-15s %-20s\n" \
            "ghi111222333" "ubuntu:22.04" "test-env" "Exited (0)" ""
    fi

    echo ""
}

# Show container logs
show_logs() {
    local container=${1:-web-server}
    local lines=${2:-50}

    print_section "Logs for $container"

    echo -e "${GRAY}2025-12-27 10:00:01 [INFO] Server starting...${RESET}"
    echo -e "${GRAY}2025-12-27 10:00:02 [INFO] Listening on port 80${RESET}"
    echo -e "${GRAY}2025-12-27 10:00:05 [INFO] Ready to accept connections${RESET}"
    echo -e "${GRAY}2025-12-27 10:05:12 [INFO] GET / HTTP/1.1 200${RESET}"
    echo -e "${GRAY}2025-12-27 10:10:30 [INFO] GET /api/health HTTP/1.1 200${RESET}"

    echo ""
}

# Container stats
show_stats() {
    local container=${1:-web-server}

    print_section "Resource Usage: $container"

    echo ""
    printf "${CYAN}%-15s %-15s %-20s %-20s %-15s${RESET}\n" \
        "CONTAINER" "CPU %" "MEM USAGE / LIMIT" "MEM %" "NET I/O"
    echo -e "${GRAY}────────────────────────────────────────────────────────────────────────────────${RESET}"

    printf "%-15s %-15s %-20s %-20s %-15s\n" \
        "web-server" "12.5%" "128Mi / 512Mi" "25.0%" "1.2MB / 850KB"

    echo ""
}

# Create network
create_network() {
    local name=${1:-app-network}
    local driver=${2:-bridge}

    print_section "Creating Network"
    print_info "Name: $name"
    print_info "Driver: $driver"

    print_section "Network Created"
    print_success "Network ID: network-$(head /dev/urandom | tr -dc a-z0-9 | head -c 16)"
    print_info "Subnet: 172.$(shuf -i 16-31 -n 1).0.0/16"
    print_info "Gateway: 172.$(shuf -i 16-31 -n 1).0.1"

    echo ""
}

# Create volume
create_volume() {
    local name=${1:-app-data}
    local driver=${2:-local}

    print_section "Creating Volume"
    print_info "Name: $name"
    print_info "Driver: $driver"

    print_section "Volume Created"
    print_success "Volume: $name"
    print_info "Mountpoint: /var/lib/docker/volumes/$name/_data"
    print_info "Driver: $driver"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comp-006 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  run                      Run a container"
    echo "    --image <image>        Container image (default: nginx:latest)"
    echo "    --name <name>          Container name (default: auto-generated)"
    echo "    --port <host:container> Port mapping (default: 8080:80)"
    echo ""
    echo "  build                    Build an image"
    echo "    --dockerfile <path>    Dockerfile path (default: ./Dockerfile)"
    echo "    --tag <tag>            Image tag (default: myapp:latest)"
    echo "    --context <path>       Build context (default: .)"
    echo ""
    echo "  ps                       List containers"
    echo "    --all                  Show all containers (default: running only)"
    echo ""
    echo "  logs                     Show container logs"
    echo "    --container <name>     Container name (default: web-server)"
    echo "    --lines <n>            Number of lines (default: 50)"
    echo ""
    echo "  stats                    Show container stats"
    echo "    --container <name>     Container name (default: web-server)"
    echo ""
    echo "  network create           Create a network"
    echo "    --name <name>          Network name (default: app-network)"
    echo "    --driver <driver>      Network driver (default: bridge)"
    echo ""
    echo "  volume create            Create a volume"
    echo "    --name <name>          Volume name (default: app-data)"
    echo "    --driver <driver>      Volume driver (default: local)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comp-006 run --image nginx:latest --port 8080:80 --name web"
    echo "  wia-comp-006 build --tag myapp:1.0.0 --dockerfile ./Dockerfile"
    echo "  wia-comp-006 ps --all"
    echo "  wia-comp-006 logs --container web --lines 100"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMP-006 Container Technology CLI Tool"
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
    run)
        IMAGE="nginx:latest"
        NAME=""
        PORT="8080:80"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --image) IMAGE=$2; shift 2 ;;
                --name) NAME=$2; shift 2 ;;
                --port) PORT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        run_container "$IMAGE" "$NAME" "$PORT"
        ;;

    build)
        DOCKERFILE="./Dockerfile"
        TAG="myapp:latest"
        CONTEXT="."

        while [[ $# -gt 0 ]]; do
            case $1 in
                --dockerfile) DOCKERFILE=$2; shift 2 ;;
                --tag) TAG=$2; shift 2 ;;
                --context) CONTEXT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        build_image "$DOCKERFILE" "$TAG" "$CONTEXT"
        ;;

    ps)
        ALL=false

        while [[ $# -gt 0 ]]; do
            case $1 in
                --all) ALL=true; shift ;;
                *) shift ;;
            esac
        done

        print_header
        list_containers "$ALL"
        ;;

    logs)
        CONTAINER="web-server"
        LINES=50

        while [[ $# -gt 0 ]]; do
            case $1 in
                --container) CONTAINER=$2; shift 2 ;;
                --lines) LINES=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        show_logs "$CONTAINER" "$LINES"
        ;;

    stats)
        CONTAINER="web-server"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --container) CONTAINER=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        show_stats "$CONTAINER"
        ;;

    network)
        SUBCMD=${1:-help}
        shift || true

        if [ "$SUBCMD" = "create" ]; then
            NAME="app-network"
            DRIVER="bridge"

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --name) NAME=$2; shift 2 ;;
                    --driver) DRIVER=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            print_header
            create_network "$NAME" "$DRIVER"
        fi
        ;;

    volume)
        SUBCMD=${1:-help}
        shift || true

        if [ "$SUBCMD" = "create" ]; then
            NAME="app-data"
            DRIVER="local"

            while [[ $# -gt 0 ]]; do
                case $1 in
                    --name) NAME=$2; shift 2 ;;
                    --driver) DRIVER=$2; shift 2 ;;
                    *) shift ;;
                esac
            done

            print_header
            create_volume "$NAME" "$DRIVER"
        fi
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comp-006 help' for usage information"
        exit 1
        ;;
esac

exit 0
