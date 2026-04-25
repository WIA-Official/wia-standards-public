#!/bin/bash

################################################################################
# WIA-QUA-013: Dark Matter Detection CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Dark Matter Physics Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to dark matter detection methods
# including WIMP searches, axion detection, indirect detection, collider
# searches, and astrophysical observations.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
MAGENTA='\033[0;35m'
GRAY='\033[0;90m'
DARK='\033[0;30m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
LOCAL_DM_DENSITY=0.3  # GeV/cm³
V0_VELOCITY=220       # km/s
ESCAPE_VELOCITY=544   # km/s
THERMAL_CROSS_SECTION=3e-26  # cm³/s

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🌑 WIA-QUA-013: Dark Matter Detection CLI              ║"
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

print_dark() {
    echo -e "${DARK}◉ $1${RESET}"
}

# Generate random number
random_float() {
    echo "scale=4; $(shuf -i 0-9999 -n 1) / 10000" | bc
}

# WIMP Direct Detection Search
wimp_search() {
    local detector=${1:-xenon}
    local mass=${2:-1000}  # kg
    local exposure=${3:-365}  # days
    local wimp_mass=${4:-50}  # GeV

    print_section "WIMP Direct Detection Search"
    print_info "Detector: $detector"
    print_info "Target mass: $mass kg"
    print_info "Exposure: $exposure days"
    print_info "WIMP mass: $wimp_mass GeV"

    print_dark "Initializing detector systems..."
    sleep 0.5

    # Simulate data collection
    print_section "Data Collection"
    local total_exposure=$(echo "$mass * $exposure" | bc)
    print_info "Total exposure: $total_exposure kg·days"

    print_dark "Collecting scintillation events..."
    sleep 0.3
    print_dark "Recording ionization signals..."
    sleep 0.3

    # Simulate event analysis
    print_section "Event Analysis"
    local total_events=$(shuf -i 1000-5000 -n 1)
    local nr_events=$(shuf -i 5-20 -n 1)
    local er_events=$((total_events - nr_events))

    print_info "Total events detected: $total_events"
    print_info "Nuclear recoils: $nr_events"
    print_info "Electron recoils: $er_events (rejected)"

    # Background estimation
    print_section "Background Estimation"
    local bg_rate=$(echo "scale=4; $(shuf -i 100-300 -n 1) / 100" | bc)
    local expected_bg=$(echo "scale=2; $bg_rate * $total_exposure / 1000" | bc)

    print_info "Background rate: ${bg_rate} events/keV/kg/day"
    print_info "Expected background: ${expected_bg} events"

    # Statistical analysis
    print_section "Statistical Analysis"
    local excess=$(echo "$nr_events - $expected_bg" | bc)
    local significance=$(echo "scale=2; $excess / sqrt($expected_bg + 1)" | bc)

    print_info "Observed excess: ${excess} events"
    print_info "Significance: ${significance}σ"

    if (( $(echo "$significance > 3.0" | bc -l) )); then
        print_warning "Evidence for signal (>3σ)!"
    elif (( $(echo "$significance > 5.0" | bc -l) )); then
        print_success "Discovery! (>5σ)"
    else
        print_info "No significant excess"
    fi

    # Cross-section limit
    print_section "Cross-Section Limit"
    local limit_exp=$(shuf -i -47--43 -n 1)
    local limit_mantissa=$(echo "scale=2; $(shuf -i 100-900 -n 1) / 100" | bc)

    print_info "90% CL upper limit: ${limit_mantissa} × 10^${limit_exp} cm²"
    print_info "WIMP mass: $wimp_mass GeV"

    echo ""
}

# Axion Cavity Search
axion_search() {
    local frequency=${1:-5.0}  # GHz
    local field=${2:-8.0}  # Tesla
    local quality=${3:-100000}

    print_section "Axion Cavity Haloscope Search"
    print_info "Center frequency: $frequency GHz"
    print_info "Magnetic field: $field Tesla"
    print_info "Cavity Q-factor: $quality"

    # Convert frequency to mass
    local mass=$(echo "scale=3; $frequency * 4.136" | bc)
    print_info "Axion mass: $mass μeV"

    print_section "Cavity Tuning"
    print_dark "Adjusting cavity resonance..."
    sleep 0.5
    print_success "Resonance locked at $frequency GHz"

    print_section "Data Acquisition"
    print_dark "Integrating signal..."
    sleep 0.8

    # Simulate power measurement
    local noise_power=$(echo "scale=4; $(shuf -i 100-200 -n 1) / 1000" | bc)
    print_info "Noise power: $noise_power fW"

    # Calculate sensitivity
    print_section "Sensitivity Calculation"
    local coupling_limit=$(echo "scale=4; 1.0 / sqrt($field * $field * $quality)" | bc)
    local limit_exp="-15"

    print_info "Coupling limit: ${coupling_limit} × 10^${limit_exp} GeV⁻¹"
    print_info "Mass range: ${mass} μeV"

    # KSVZ/DFSZ bands
    print_section "Theoretical Models"
    print_info "KSVZ model: g_aγγ ∼ -0.97 × (m_a/1 μeV)"
    print_info "DFSZ model: g_aγγ ∼ 0.36 × (m_a/1 μeV)"

    if (( $(echo "$mass > 4.0 && $mass < 6.0" | bc -l) )); then
        print_success "Sensitive to QCD axion range!"
    fi

    echo ""
}

# Direct Detection Event Analysis
direct_analyze() {
    local detector=${1:-xenon}
    local energy=${2:-3.5}  # keV
    local event_type=${3:-nuclear-recoil}

    print_section "Direct Detection Event Analysis"
    print_info "Detector: $detector"
    print_info "Energy: $energy keV"
    print_info "Event type: $event_type"

    print_section "Signal Processing"

    # S1 and S2 signals (for two-phase detectors)
    if [[ "$detector" == "xenon" || "$detector" == "argon" ]]; then
        local s1=$(shuf -i 50-200 -n 1)
        local s2_ratio=$(echo "scale=2; $(shuf -i 10-30 -n 1) / 100" | bc)
        local s2=$(echo "$s1 * $s2_ratio" | bc)

        print_info "S1 (scintillation): $s1 photons"
        print_info "S2 (ionization): $s2 electrons"
        print_info "S2/S1 ratio: $s2_ratio"

        print_section "Discrimination"
        local log_s2s1=$(echo "scale=3; l($s2_ratio) / l(10)" | bc -l)
        print_info "log₁₀(S2/S1): $log_s2s1"

        if (( $(echo "$s2_ratio < 0.5" | bc -l) )); then
            print_success "Consistent with nuclear recoil"
        else
            print_warning "Consistent with electron recoil (background)"
        fi
    fi

    # Pulse shape analysis
    print_section "Pulse Shape Analysis"
    local rise_time=$(echo "scale=1; $(shuf -i 50-150 -n 1) / 10" | bc)
    local fall_time=$(echo "scale=1; $(shuf -i 200-600 -n 1) / 10" | bc)

    print_info "Rise time: $rise_time ns"
    print_info "Fall time: $fall_time ns"

    if [[ "$detector" == "argon" ]]; then
        local prompt_frac=$(echo "scale=3; $(shuf -i 650-750 -n 1) / 1000" | bc)
        print_info "Prompt fraction: $prompt_frac"

        if (( $(echo "$prompt_frac > 0.65" | bc -l) )); then
            print_success "Nuclear recoil signature"
        else
            print_info "Electron recoil signature"
        fi
    fi

    echo ""
}

# Indirect Detection - Gamma Ray
indirect_gamma() {
    local target=${1:-galactic-center}
    local instrument=${2:-fermi-lat}
    local wimp_mass=${3:-100}  # GeV

    print_section "Gamma Ray Indirect Detection"
    print_info "Target: $target"
    print_info "Instrument: $instrument"
    print_info "WIMP mass: $wimp_mass GeV"

    # J-factor
    print_section "Astrophysical Factor"
    local j_factor_exp=$(shuf -i 22-26 -n 1)
    local j_factor_mantissa=$(echo "scale=2; $(shuf -i 100-900 -n 1) / 100" | bc)

    print_info "J-factor: ${j_factor_mantissa} × 10^${j_factor_exp} GeV² cm⁻⁵"

    # Observation
    print_section "Observation"
    print_dark "Analyzing $instrument data..."
    sleep 0.5

    local obs_time=$(shuf -i 100-1000 -n 1)
    print_info "Observation time: $obs_time hours"

    # Flux limit
    print_section "Results"
    local flux_limit_exp=$(shuf -i -12--10 -n 1)
    local flux_limit_mant=$(echo "scale=2; $(shuf -i 100-900 -n 1) / 100" | bc)

    print_info "Photon flux limit: ${flux_limit_mant} × 10^${flux_limit_exp} cm⁻² s⁻¹"

    # Cross-section limit
    local sigma_v_exp=$(shuf -i -26--24 -n 1)
    local sigma_v_mant=$(echo "scale=2; $(shuf -i 100-900 -n 1) / 100" | bc)

    print_info "⟨σv⟩ limit: ${sigma_v_mant} × 10^${sigma_v_exp} cm³ s⁻¹"
    print_info "Channel: bb̄"

    if (( $(echo "$sigma_v_exp >= -26" | bc -l) )); then
        print_success "Approaching thermal relic cross-section!"
    fi

    echo ""
}

# Collider Search
collider_search() {
    local experiment=${1:-atlas}
    local channel=${2:-monojet}
    local luminosity=${3:-139}  # fb⁻¹

    print_section "Collider Dark Matter Search"
    print_info "Experiment: ${experiment^^}"
    print_info "Channel: $channel"
    print_info "Luminosity: $luminosity fb⁻¹"

    print_section "Event Selection"
    print_dark "Applying triggers..."
    sleep 0.3

    case "$channel" in
        monojet)
            print_info "Requirement: High-pT jet + large MET"
            print_info "Jet pT > 250 GeV"
            print_info "MET > 250 GeV"
            ;;
        monophoton)
            print_info "Requirement: High-pT photon + large MET"
            print_info "Photon pT > 150 GeV"
            print_info "MET > 150 GeV"
            ;;
    esac

    # Event counts
    print_section "Event Counts"
    local observed=$(shuf -i 8000-12000 -n 1)
    local expected=$(shuf -i 7800-11800 -n 1)

    print_info "Observed: $observed events"
    print_info "Expected (SM): $expected events"
    print_info "Data/MC ratio: $(echo "scale=3; $observed / $expected" | bc)"

    # Limits
    print_section "Interpretation"
    local eft_limit=$(shuf -i 1500-3000 -n 1)
    print_info "EFT scale limit: $eft_limit GeV"

    local dm_mass_limit=$(shuf -i 400-800 -n 1)
    print_info "DM mass limit: $dm_mass_limit GeV (simplified model)"

    print_success "Results published in JHEP"

    echo ""
}

# Gravitational Lensing
lensing_analyze() {
    local cluster=${1:-abell-1689}
    local method=${2:-weak-lensing}

    print_section "Gravitational Lensing Analysis"
    print_info "Cluster: ${cluster^^}"
    print_info "Method: $method"

    print_section "Mass Reconstruction"
    print_dark "Analyzing lensing shear..."
    sleep 0.5
    print_dark "Reconstructing mass distribution..."
    sleep 0.5

    # Results
    print_section "Results"
    local total_mass=$(shuf -i 15-25 -n 1)
    print_info "Total mass: ${total_mass} × 10¹⁴ M☉"

    local dm_fraction=$(echo "scale=3; $(shuf -i 820-880 -n 1) / 1000" | bc)
    print_info "Dark matter fraction: $dm_fraction"

    local ml_ratio=$(shuf -i 250-350 -n 1)
    print_info "Mass-to-light ratio: $ml_ratio M☉/L☉"

    print_section "Dark Matter Profile"
    local concentration=$(echo "scale=1; $(shuf -i 35-55 -n 1) / 10" | bc)
    print_info "NFW concentration: $concentration"
    print_info "Virial radius: 2000 kpc"

    print_success "Strong evidence for dark matter halo"

    echo ""
}

# Background Estimation
background_estimate() {
    local detector=${1:-xenon}
    local location=${2:-underground}

    print_section "Background Estimation"
    print_info "Detector: $detector"
    print_info "Location: $location"

    print_section "Radioactive Backgrounds"
    print_info "⁴⁰K: $(echo "scale=2; $(shuf -i 10-50 -n 1) / 100" | bc) mBq/kg"
    print_info "²³⁸U chain: $(echo "scale=2; $(shuf -i 5-20 -n 1) / 100" | bc) mBq/kg"
    print_info "²³²Th chain: $(echo "scale=2; $(shuf -i 5-20 -n 1) / 100" | bc) mBq/kg"

    if [[ "$detector" == "xenon" ]]; then
        print_info "⁸⁵Kr: $(echo "scale=3; $(shuf -i 10-100 -n 1) / 1000" | bc) ppt"
        print_info "²²²Rn: $(echo "scale=2; $(shuf -i 1-10 -n 1) / 10" | bc) μBq/m³"
    fi

    print_section "Cosmogenic Activation"
    if [[ "$location" == "underground" ]]; then
        local depth=$(shuf -i 3000-6000 -n 1)
        print_info "Depth: $depth m.w.e."
        print_info "Muon flux reduction: 10⁻⁷"
        print_success "Cosmogenic backgrounds negligible"
    else
        print_warning "Surface location - high cosmogenic background"
    fi

    print_section "Neutrino Floor"
    print_info "Solar neutrinos: Dominant at low cross-section"
    print_info "Atmospheric neutrinos: Sub-dominant"
    print_warning "Cannot reduce below neutrino coherent scattering"

    echo ""
}

# Calculate Detection Limits
limits_calculate() {
    local detector=${1:-xenon}
    local exposure=${2:-1000}  # kg·days
    local wimp_mass=${3:-50}  # GeV

    print_section "Detection Limit Calculation"
    print_info "Detector: $detector"
    print_info "Exposure: $exposure kg·days"
    print_info "WIMP mass: $wimp_mass GeV"

    print_section "Calculation"
    print_dark "Computing expected sensitivity..."
    sleep 0.5

    # Background rate
    local bg_rate=$(echo "scale=4; $(shuf -i 50-200 -n 1) / 1000" | bc)
    print_info "Background rate: $bg_rate events/keV/kg/day"

    # Energy window
    local e_min=1
    local e_max=50
    print_info "Energy window: $e_min - $e_max keV"

    # Expected background
    local expected_bg=$(echo "scale=2; $bg_rate * $exposure * ($e_max - $e_min)" | bc)
    print_info "Expected background: $expected_bg events"

    # 90% CL limit
    print_section "90% Confidence Level Limit"
    local limit_events=2.44  # Feldman-Cousins for 0 observed

    local limit_exp=$(shuf -i -46--44 -n 1)
    local limit_mant=$(echo "scale=2; $(shuf -i 100-900 -n 1) / 100" | bc)

    print_info "Cross-section limit: ${limit_mant} × 10^${limit_exp} cm²"
    print_info "WIMP mass: $wimp_mass GeV"

    print_section "Projected Sensitivity"
    local sens_10x=$(echo "scale=2; $limit_mant / sqrt(10)" | bc)
    print_info "With 10× exposure: ${sens_10x} × 10^${limit_exp} cm²"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-qua-013 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  wimp search              Perform WIMP direct detection search"
    echo "    --detector <type>      Detector type (xenon/argon/germanium)"
    echo "    --mass <kg>            Target mass in kg"
    echo "    --exposure <days>      Exposure time in days"
    echo "    --wimp-mass <GeV>      WIMP mass in GeV"
    echo ""
    echo "  axion search             Perform axion cavity search"
    echo "    --type <method>        Search type (cavity/helioscope)"
    echo "    --frequency <GHz>      Resonant frequency in GHz"
    echo "    --field <Tesla>        Magnetic field in Tesla"
    echo ""
    echo "  direct analyze           Analyze direct detection event"
    echo "    --detector <type>      Detector type"
    echo "    --energy <keV>         Event energy in keV"
    echo "    --type <recoil>        Event type (nuclear/electron)"
    echo ""
    echo "  indirect gamma           Gamma ray indirect detection"
    echo "    --target <name>        Target (galactic-center/dwarf)"
    echo "    --instrument <name>    Instrument (fermi-lat/hess)"
    echo ""
    echo "  collider search          Search for DM at colliders"
    echo "    --experiment <name>    ATLAS/CMS/LHCb"
    echo "    --channel <type>       monojet/monophoton/mono-Z"
    echo "    --lumi <fb-1>          Integrated luminosity"
    echo ""
    echo "  lensing analyze          Gravitational lensing analysis"
    echo "    --cluster <name>       Cluster name"
    echo "    --method <type>        weak-lensing/strong-lensing"
    echo ""
    echo "  background estimate      Estimate background rates"
    echo "    --detector <type>      Detector type"
    echo "    --location <place>     underground/surface"
    echo ""
    echo "  limits calculate         Calculate detection limits"
    echo "    --detector <type>      Detector type"
    echo "    --exposure <kg·days>   Total exposure"
    echo "    --wimp-mass <GeV>      WIMP mass"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-qua-013 wimp search --detector xenon --mass 1000 --exposure 365"
    echo "  wia-qua-013 axion search --frequency 5.0 --field 8.0"
    echo "  wia-qua-013 direct analyze --detector xenon --energy 3.5 --type nuclear-recoil"
    echo "  wia-qua-013 indirect gamma --target galactic-center --instrument fermi-lat"
    echo "  wia-qua-013 collider search --experiment atlas --channel monojet --lumi 139"
    echo "  wia-qua-013 lensing analyze --cluster abell-1689 --method weak-lensing"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-QUA-013 Dark Matter Detection CLI Tool"
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
    wimp)
        SUBCMD=${1:-search}
        shift || true

        DETECTOR="xenon"
        MASS=1000
        EXPOSURE=365
        WIMP_MASS=50

        while [[ $# -gt 0 ]]; do
            case $1 in
                --detector) DETECTOR=$2; shift 2 ;;
                --mass) MASS=$2; shift 2 ;;
                --exposure) EXPOSURE=$2; shift 2 ;;
                --wimp-mass) WIMP_MASS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        wimp_search "$DETECTOR" "$MASS" "$EXPOSURE" "$WIMP_MASS"
        ;;

    axion)
        SUBCMD=${1:-search}
        shift || true

        FREQUENCY=5.0
        FIELD=8.0
        QUALITY=100000

        while [[ $# -gt 0 ]]; do
            case $1 in
                --frequency) FREQUENCY=$2; shift 2 ;;
                --field) FIELD=$2; shift 2 ;;
                --quality) QUALITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        axion_search "$FREQUENCY" "$FIELD" "$QUALITY"
        ;;

    direct)
        SUBCMD=${1:-analyze}
        shift || true

        DETECTOR="xenon"
        ENERGY=3.5
        TYPE="nuclear-recoil"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --detector) DETECTOR=$2; shift 2 ;;
                --energy) ENERGY=$2; shift 2 ;;
                --type) TYPE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        direct_analyze "$DETECTOR" "$ENERGY" "$TYPE"
        ;;

    indirect)
        SUBCMD=${1:-gamma}
        shift || true

        TARGET="galactic-center"
        INSTRUMENT="fermi-lat"
        WIMP_MASS=100

        while [[ $# -gt 0 ]]; do
            case $1 in
                --target) TARGET=$2; shift 2 ;;
                --instrument) INSTRUMENT=$2; shift 2 ;;
                --wimp-mass) WIMP_MASS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        indirect_gamma "$TARGET" "$INSTRUMENT" "$WIMP_MASS"
        ;;

    collider)
        SUBCMD=${1:-search}
        shift || true

        EXPERIMENT="atlas"
        CHANNEL="monojet"
        LUMI=139

        while [[ $# -gt 0 ]]; do
            case $1 in
                --experiment) EXPERIMENT=$2; shift 2 ;;
                --channel) CHANNEL=$2; shift 2 ;;
                --lumi) LUMI=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        collider_search "$EXPERIMENT" "$CHANNEL" "$LUMI"
        ;;

    lensing)
        SUBCMD=${1:-analyze}
        shift || true

        CLUSTER="abell-1689"
        METHOD="weak-lensing"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --cluster) CLUSTER=$2; shift 2 ;;
                --method) METHOD=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        lensing_analyze "$CLUSTER" "$METHOD"
        ;;

    background)
        SUBCMD=${1:-estimate}
        shift || true

        DETECTOR="xenon"
        LOCATION="underground"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --detector) DETECTOR=$2; shift 2 ;;
                --location) LOCATION=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        background_estimate "$DETECTOR" "$LOCATION"
        ;;

    limits)
        SUBCMD=${1:-calculate}
        shift || true

        DETECTOR="xenon"
        EXPOSURE=1000
        WIMP_MASS=50

        while [[ $# -gt 0 ]]; do
            case $1 in
                --detector) DETECTOR=$2; shift 2 ;;
                --exposure) EXPOSURE=$2; shift 2 ;;
                --wimp-mass) WIMP_MASS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        limits_calculate "$DETECTOR" "$EXPOSURE" "$WIMP_MASS"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-qua-013 help' for usage information"
        exit 1
        ;;
esac

exit 0
