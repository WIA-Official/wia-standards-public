#!/bin/bash

################################################################################
# WIA-COMP-001: Supercomputing CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Computing Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to supercomputing calculations
# including FLOPS estimation, scalability analysis, and performance benchmarking.
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
EXAFLOPS=1000000000000000000
PETAFLOPS=1000000000000000
TERAFLOPS=1000000000000
GIGAFLOPS=1000000000

# Helper functions
print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🚀 WIA-COMP-001: Supercomputing CLI                   ║"
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

format_flops() {
    local flops=$1

    if (( $(echo "$flops >= $EXAFLOPS" | bc -l) )); then
        printf "%.2f exaFLOPS" "$(echo "$flops / $EXAFLOPS" | bc -l)"
    elif (( $(echo "$flops >= $PETAFLOPS" | bc -l) )); then
        printf "%.2f petaFLOPS" "$(echo "$flops / $PETAFLOPS" | bc -l)"
    elif (( $(echo "$flops >= $TERAFLOPS" | bc -l) )); then
        printf "%.2f teraFLOPS" "$(echo "$flops / $TERAFLOPS" | bc -l)"
    elif (( $(echo "$flops >= $GIGAFLOPS" | bc -l) )); then
        printf "%.2f gigaFLOPS" "$(echo "$flops / $GIGAFLOPS" | bc -l)"
    else
        printf "%.2f FLOPS" "$flops"
    fi
}

# Calculate FLOPS
calc_flops() {
    local nodes=${1:-1000}
    local cores=${2:-128}
    local clock=${3:-2.5e9}
    local flop_per_cycle=${4:-32}
    local gpu_count=${5:-0}
    local gpu_tflops=${6:-0}

    print_section "FLOPS Calculation"
    print_info "Nodes: $nodes"
    print_info "Cores per node: $cores"
    print_info "Clock speed: $(echo "$clock / 1e9" | bc -l) GHz"
    print_info "FLOP/cycle: $flop_per_cycle"

    # CPU FLOPS
    local cpu_flops=$(echo "$nodes * $cores * $clock * $flop_per_cycle" | bc -l)

    # GPU FLOPS
    local gpu_flops=0
    if (( $(echo "$gpu_count > 0" | bc -l) )); then
        print_info "GPUs per node: $gpu_count"
        print_info "GPU TFLOPS: $gpu_tflops"
        gpu_flops=$(echo "$nodes * $gpu_count * $gpu_tflops * $TERAFLOPS" | bc -l)
    fi

    # Total FLOPS
    local total_flops=$(echo "$cpu_flops + $gpu_flops" | bc -l)
    local sustained_flops=$(echo "$total_flops * 0.7" | bc -l)

    print_section "Performance"
    print_success "CPU FLOPS: $(format_flops $cpu_flops)"
    if (( $(echo "$gpu_flops > 0" | bc -l) )); then
        print_success "GPU FLOPS: $(format_flops $gpu_flops)"
    fi
    print_success "Peak FLOPS: $(format_flops $total_flops)"
    print_info "Sustained FLOPS: $(format_flops $sustained_flops) (70% efficiency)"

    # Determine tier
    print_section "Classification"
    if (( $(echo "$total_flops >= $EXAFLOPS" | bc -l) )); then
        print_success "Performance Tier: EXASCALE 🏆"
    elif (( $(echo "$total_flops >= $PETAFLOPS" | bc -l) )); then
        print_success "Performance Tier: PETASCALE"
    elif (( $(echo "$total_flops >= $TERAFLOPS" | bc -l) )); then
        print_success "Performance Tier: TERASCALE"
    else
        print_info "Performance Tier: HIGH-PERFORMANCE"
    fi

    echo ""
}

# Validate cluster configuration
validate_cluster() {
    local topology=${1:-fat-tree}
    local nodes=${2:-1000}
    local bandwidth=${3:-200e9}
    local latency=${4:-1e-6}

    print_section "Cluster Configuration Validation"
    print_info "Topology: $topology"
    print_info "Nodes: $nodes"
    print_info "Interconnect Bandwidth: $(echo "$bandwidth / 1e9" | bc -l) Gb/s"
    print_info "Network Latency: $(echo "$latency * 1e6" | bc -l) μs"

    print_section "Validation Checks"

    # Check node count
    if (( nodes < 100 )); then
        print_warning "Small cluster (<100 nodes) - simple topology sufficient"
    elif (( nodes > 100000 )); then
        print_warning "Very large cluster (>100k nodes) - requires careful design"
    else
        print_success "Node count: OPTIMAL"
    fi

    # Check bandwidth
    local gbps=$(echo "$bandwidth / 1e9" | bc -l)
    if (( $(echo "$gbps < 100" | bc -l) )); then
        print_error "Low bandwidth (<100 Gb/s)"
        print_info "Recommendation: Upgrade to InfiniBand HDR (200 Gb/s)"
    else
        print_success "Interconnect bandwidth: GOOD"
    fi

    # Check latency
    local latency_us=$(echo "$latency * 1e6" | bc -l)
    if (( $(echo "$latency_us > 5" | bc -l) )); then
        print_warning "High latency (>${latency_us} μs)"
        print_info "Recommendation: Use RDMA-capable interconnect"
    else
        print_success "Network latency: EXCELLENT"
    fi

    print_section "Topology Metrics"
    case $topology in
        fat-tree)
            local diameter=$(echo "l($nodes)/l(2) * 2" | bc -l)
            print_info "Estimated diameter: $(printf "%.0f" $diameter) hops"
            print_info "Bisection bandwidth: FULL (non-blocking)"
            ;;
        dragonfly)
            print_info "Estimated diameter: 3 hops"
            print_info "Cost-effective for large systems"
            ;;
        torus)
            local dim=3
            local nodes_per_dim=$(echo "e(l($nodes)/$dim)" | bc -l)
            local diameter=$(echo "$dim * $nodes_per_dim / 2" | bc -l)
            print_info "Estimated diameter: $(printf "%.0f" $diameter) hops"
            print_info "Good for nearest-neighbor communication"
            ;;
    esac

    echo ""
}

# Run benchmark
benchmark() {
    local type=${1:-LINPACK}
    local nodes=${2:-1000}
    local cores=${3:-128}
    local problem_size=${4:-1000000}

    print_section "Performance Benchmark: $type"
    print_info "Nodes: $nodes"
    print_info "Cores per node: $cores"
    print_info "Problem size: $problem_size"

    # Calculate theoretical performance
    local theoretical=$(echo "$nodes * $cores * 2.5e9 * 32" | bc -l)

    case $type in
        LINPACK)
            local efficiency=0.75
            local performance=$(echo "$theoretical * $efficiency" | bc -l)
            local ops=$(echo "2.0/3.0 * $problem_size * $problem_size * $problem_size" | bc -l)
            local time=$(echo "$ops / $performance" | bc -l)

            print_section "Results"
            print_success "R_max: $(format_flops $performance)"
            print_info "R_peak: $(format_flops $theoretical)"
            print_success "Efficiency: $(echo "$efficiency * 100" | bc -l)%"
            print_info "Execution time: $(printf "%.1f" $time) seconds"
            print_info "Problem: ${problem_size}x${problem_size} dense linear system"
            ;;

        HPCG)
            local efficiency=0.03
            local performance=$(echo "$theoretical * $efficiency" | bc -l)

            print_section "Results"
            print_success "Performance: $(format_flops $performance)"
            print_info "Theoretical: $(format_flops $theoretical)"
            print_info "Efficiency: $(echo "$efficiency * 100" | bc -l)%"
            print_warning "HPCG is much harder to optimize than LINPACK"
            print_info "More realistic for sparse iterative solvers"
            ;;

        STREAM)
            local mem_bw=$(echo "$nodes * 200e9" | bc -l)
            print_section "Results"
            print_success "Memory Bandwidth: $(echo "$mem_bw / 1e9" | bc -l) GB/s"
            print_info "Copy: $(echo "$mem_bw * 0.8 / 1e9" | bc -l) GB/s"
            print_info "Scale: $(echo "$mem_bw * 0.85 / 1e9" | bc -l) GB/s"
            print_info "Add: $(echo "$mem_bw * 0.9 / 1e9" | bc -l) GB/s"
            print_info "Triad: $(echo "$mem_bw * 0.95 / 1e9" | bc -l) GB/s"
            ;;

        *)
            print_error "Unknown benchmark type: $type"
            return 1
            ;;
    esac

    echo ""
}

# Analyze scalability
analyze_scalability() {
    local baseline_nodes=${1:-1}
    local baseline_time=${2:-3600}
    local target_nodes=${3:-1000}
    local serial_fraction=${4:-0.05}

    print_section "Scalability Analysis"
    print_info "Baseline: $baseline_nodes nodes, $baseline_time seconds"
    print_info "Target: $target_nodes nodes"
    print_info "Serial fraction: $(echo "$serial_fraction * 100" | bc -l)%"

    # Amdahl's Law: Speedup = 1 / (s + (1-s)/p)
    local node_ratio=$(echo "$target_nodes / $baseline_nodes" | bc -l)
    local parallel_fraction=$(echo "1 - $serial_fraction" | bc -l)
    local speedup=$(echo "1 / ($serial_fraction + $parallel_fraction / $node_ratio)" | bc -l)
    local efficiency=$(echo "$speedup / $node_ratio" | bc -l)
    local estimated_time=$(echo "$baseline_time / $speedup" | bc -l)

    print_section "Scalability Results"
    print_success "Speedup: $(printf "%.2f" $speedup)x"
    print_success "Parallel efficiency: $(echo "$efficiency * 100" | bc -l)%"
    print_success "Estimated time: $(printf "%.1f" $estimated_time) seconds"

    print_section "Assessment"
    if (( $(echo "$efficiency >= 0.9" | bc -l) )); then
        print_success "Scaling: EXCELLENT (≥90%)"
    elif (( $(echo "$efficiency >= 0.7" | bc -l) )); then
        print_success "Scaling: GOOD (≥70%)"
    elif (( $(echo "$efficiency >= 0.5" | bc -l) )); then
        print_warning "Scaling: MODERATE (≥50%)"
    else
        print_error "Scaling: POOR (<50%)"
        print_info "Bottleneck: High serial fraction or communication overhead"
    fi

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comp-001 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  calc-flops               Calculate system FLOPS"
    echo "    --nodes <n>            Number of nodes (default: 1000)"
    echo "    --cores <n>            Cores per node (default: 128)"
    echo "    --clock <Hz>           Clock speed (default: 2.5e9)"
    echo "    --flop-cycle <n>       FLOP per cycle (default: 32)"
    echo "    --gpus <n>             GPUs per node (default: 0)"
    echo "    --gpu-tflops <n>       TFLOPS per GPU (default: 0)"
    echo ""
    echo "  validate-cluster         Validate cluster configuration"
    echo "    --topology <name>      Topology: fat-tree, dragonfly, torus (default: fat-tree)"
    echo "    --nodes <n>            Number of nodes (default: 1000)"
    echo "    --bandwidth <bps>      Interconnect bandwidth (default: 200e9)"
    echo "    --latency <s>          Network latency (default: 1e-6)"
    echo ""
    echo "  benchmark                Run performance benchmark"
    echo "    --type <name>          LINPACK, HPCG, STREAM (default: LINPACK)"
    echo "    --nodes <n>            Number of nodes (default: 1000)"
    echo "    --cores <n>            Cores per node (default: 128)"
    echo "    --size <n>             Problem size (default: 1000000)"
    echo ""
    echo "  analyze-scalability      Analyze application scalability"
    echo "    --baseline-nodes <n>   Baseline node count (default: 1)"
    echo "    --baseline-time <s>    Baseline time (default: 3600)"
    echo "    --target-nodes <n>     Target node count (default: 1000)"
    echo "    --serial <fraction>    Serial fraction (default: 0.05)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comp-001 calc-flops --nodes 1000 --cores 128 --clock 2.5e9"
    echo "  wia-comp-001 validate-cluster --topology fat-tree --nodes 1000"
    echo "  wia-comp-001 benchmark --type LINPACK --nodes 1000"
    echo "  wia-comp-001 analyze-scalability --target-nodes 1000"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMP-001 Supercomputing CLI Tool"
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
    calc-flops)
        NODES=1000
        CORES=128
        CLOCK=2.5e9
        FLOP_CYCLE=32
        GPUS=0
        GPU_TFLOPS=0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --nodes) NODES=$2; shift 2 ;;
                --cores) CORES=$2; shift 2 ;;
                --clock) CLOCK=$2; shift 2 ;;
                --flop-cycle) FLOP_CYCLE=$2; shift 2 ;;
                --gpus) GPUS=$2; shift 2 ;;
                --gpu-tflops) GPU_TFLOPS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        calc_flops "$NODES" "$CORES" "$CLOCK" "$FLOP_CYCLE" "$GPUS" "$GPU_TFLOPS"
        ;;

    validate-cluster)
        TOPOLOGY="fat-tree"
        NODES=1000
        BANDWIDTH=200e9
        LATENCY=1e-6

        while [[ $# -gt 0 ]]; do
            case $1 in
                --topology) TOPOLOGY=$2; shift 2 ;;
                --nodes) NODES=$2; shift 2 ;;
                --bandwidth) BANDWIDTH=$2; shift 2 ;;
                --latency) LATENCY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_cluster "$TOPOLOGY" "$NODES" "$BANDWIDTH" "$LATENCY"
        ;;

    benchmark)
        TYPE="LINPACK"
        NODES=1000
        CORES=128
        SIZE=1000000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --type) TYPE=$2; shift 2 ;;
                --nodes) NODES=$2; shift 2 ;;
                --cores) CORES=$2; shift 2 ;;
                --size) SIZE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        benchmark "$TYPE" "$NODES" "$CORES" "$SIZE"
        ;;

    analyze-scalability)
        BASELINE_NODES=1
        BASELINE_TIME=3600
        TARGET_NODES=1000
        SERIAL=0.05

        while [[ $# -gt 0 ]]; do
            case $1 in
                --baseline-nodes) BASELINE_NODES=$2; shift 2 ;;
                --baseline-time) BASELINE_TIME=$2; shift 2 ;;
                --target-nodes) TARGET_NODES=$2; shift 2 ;;
                --serial) SERIAL=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        analyze_scalability "$BASELINE_NODES" "$BASELINE_TIME" "$TARGET_NODES" "$SERIAL"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comp-001 help' for usage information"
        exit 1
        ;;
esac

exit 0
