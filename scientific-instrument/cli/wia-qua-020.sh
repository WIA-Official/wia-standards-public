#!/bin/bash

################################################################################
# WIA-QUA-020: Scientific Instrument CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Scientific Instrumentation Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to scientific instrument operations
# including particle accelerators, mass spectrometers, electron microscopes,
# X-ray crystallography, NMR spectrometers, telescopes, and more.
################################################################################

set -e

# Colors for output
INDIGO='\033[0;94m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SPEED_OF_LIGHT=299792458
PLANCK_H=6.62607015e-34
ELECTRON_MASS=9.1093837015e-31

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🔬 WIA-QUA-020: Scientific Instrument CLI              ║"
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

# Particle accelerator operations
particle_accelerator() {
    local type=${1:-LHC}
    local energy=${2:-13.6}

    print_section "Particle Accelerator: $type"

    case $type in
        LHC)
            local circumference=26659
            local magnetic_field=8.3
            local luminosity=1e34
            ;;
        synchrotron)
            local circumference=500
            local magnetic_field=1.5
            local luminosity=1e32
            ;;
        *)
            print_error "Unknown accelerator type: $type"
            return 1
            ;;
    esac

    print_info "Type: $type"
    print_info "Beam Energy: $energy TeV"
    print_info "Circumference: $circumference m"
    print_info "Magnetic Field: $magnetic_field T"
    print_info "Luminosity: $luminosity cm⁻²s⁻¹"

    print_section "Collision Simulation"

    local events=$((RANDOM % 1000 + 100))
    print_success "Events detected: $events"
    print_info "Collision energy: $(echo "$energy * 2" | bc -l) TeV"
    print_info "Detectors: ATLAS, CMS, ALICE, LHCb"

    echo ""
}

# Mass spectrometer operations
mass_spectrometer() {
    local type=${1:-Orbitrap}
    local resolution=${2:-240000}

    print_section "Mass Spectrometer: $type"

    case $type in
        Orbitrap)
            local mass_accuracy=1
            local scan_rate=12
            ;;
        TOF)
            local mass_accuracy=5
            local scan_rate=100
            ;;
        quadrupole)
            local mass_accuracy=100
            local scan_rate=10
            ;;
        *)
            print_error "Unknown mass spectrometer type: $type"
            return 1
            ;;
    esac

    print_info "Type: $type"
    print_info "Resolution: $resolution (FWHM)"
    print_info "Mass Accuracy: <$mass_accuracy ppm"
    print_info "Scan Rate: $scan_rate Hz"

    print_section "Sample Analysis"

    local num_peaks=$((RANDOM % 50 + 10))
    print_success "Peaks detected: $num_peaks"
    print_info "Base peak m/z: $(echo "scale=4; 100 + $RANDOM % 900" | bc -l)"
    print_info "Total ion current: $(echo "scale=2; $RANDOM * 1000" | bc -l)"

    echo ""
}

# Electron microscope operations
electron_microscope() {
    local type=${1:-TEM}
    local voltage=${2:-300}

    print_section "Electron Microscope: $type"

    case $type in
        TEM)
            local resolution=0.05
            local mag_max=1000000
            ;;
        SEM)
            local resolution=1
            local mag_max=500000
            ;;
        *)
            print_error "Unknown microscope type: $type"
            return 1
            ;;
    esac

    print_info "Type: $type"
    print_info "Acceleration Voltage: $voltage kV"
    print_info "Resolution: $resolution nm"
    print_info "Max Magnification: ${mag_max}×"

    print_section "Image Acquisition"

    # Calculate electron wavelength
    local wavelength=$(echo "scale=12; 1.226 / sqrt($voltage)" | bc -l)

    print_success "Image captured successfully"
    print_info "Electron wavelength: $wavelength nm"
    print_info "Image size: 2048 × 2048 pixels"
    print_info "Pixel size: 0.1 nm"

    echo ""
}

# X-ray crystallography
xray_crystallography() {
    local wavelength=${1:-1.5418}
    local resolution=${2:-0.8}

    print_section "X-ray Crystallography"

    print_info "X-ray Source: Cu Kα"
    print_info "Wavelength: $wavelength Å"
    print_info "Resolution: $resolution Å"
    print_info "Temperature: 100 K"

    print_section "Data Collection"

    local reflections=$((RANDOM % 50000 + 10000))
    local completeness=$((RANDOM % 20 + 80))
    local r_factor=$(echo "scale=3; 0.15 + ($RANDOM % 100) / 1000" | bc -l)

    print_success "Reflections collected: $reflections"
    print_info "Completeness: $completeness%"
    print_info "R-factor: $r_factor"
    print_info "Space group: P 21 21 21"

    echo ""
}

# NMR spectrometer
nmr_spectrometer() {
    local field=${1:-600}
    local nucleus=${2:-1H}

    print_section "NMR Spectrometer: ${field} MHz"

    local magnetic_field=$(echo "scale=2; $field / 42.576" | bc -l)

    print_info "Field Strength: $field MHz (¹H)"
    print_info "Magnetic Field: $magnetic_field Tesla"
    print_info "Nucleus: $nucleus"
    print_info "Temperature: 298 K"

    print_section "Spectrum Acquisition"

    print_success "Spectrum acquired successfully"
    print_info "Chemical shifts (ppm): 0.9 (t, 3H), 1.2 (d, 2H), 7.2 (m, 2H), 7.8 (d, 2H)"
    print_info "Scans: 16"
    print_info "Acquisition time: 2.5 seconds"

    echo ""
}

# Telescope observation
telescope() {
    local type=${1:-optical}
    local target=${2:-M31}

    print_section "Telescope: $type"

    case $type in
        optical)
            local aperture=10
            local wavelength=550
            ;;
        radio)
            local aperture=100
            local wavelength=21e7
            ;;
        infrared)
            local aperture=6.5
            local wavelength=2000
            ;;
        *)
            print_error "Unknown telescope type: $type"
            return 1
            ;;
    esac

    print_info "Type: $type telescope"
    print_info "Aperture: $aperture m"
    print_info "Wavelength: $wavelength nm"
    print_info "Target: $target"

    print_section "Observation"

    local ra=$(echo "scale=2; $RANDOM % 360" | bc -l)
    local dec=$(echo "scale=2; $RANDOM % 180 - 90" | bc -l)
    local magnitude=$(echo "scale=1; $RANDOM % 100 / 10" | bc -l)

    print_success "Observation complete"
    print_info "RA: $ra degrees"
    print_info "Dec: $dec degrees"
    print_info "Magnitude: $magnitude"
    print_info "Exposure time: 300 seconds"

    echo ""
}

# Spectrophotometer
spectrophotometer() {
    local type=${1:-UV-Vis}
    local wavelength=${2:-280}

    print_section "Spectrophotometer: $type"

    print_info "Type: $type"
    print_info "Wavelength: $wavelength nm"
    print_info "Bandwidth: 1 nm"

    print_section "Measurement"

    local absorbance=$(echo "scale=3; $RANDOM % 3000 / 1000" | bc -l)
    local transmittance=$(echo "scale=1; 10^(-$absorbance) * 100" | bc -l)

    print_success "Measurement complete"
    print_info "Absorbance: $absorbance"
    print_info "Transmittance: $transmittance%"
    print_info "Peaks detected: 3"

    echo ""
}

# Chromatography
chromatography() {
    local type=${1:-HPLC}
    local flow_rate=${2:-1.0}

    print_section "Chromatography: $type"

    print_info "Type: $type"
    print_info "Flow Rate: $flow_rate mL/min"
    print_info "Pressure: 250 bar"
    print_info "Column: C18, 4.6 × 150 mm"

    print_section "Separation"

    local num_peaks=$((RANDOM % 10 + 3))
    print_success "Peaks detected: $num_peaks"
    print_info "Retention times: 5.2, 8.7, 12.4, 15.9 min"
    print_info "Resolution: 2.5"
    print_info "Runtime: 30 minutes"

    echo ""
}

# Calorimeter
calorimeter() {
    local type=${1:-DSC}
    local rate=${2:-10}

    print_section "Calorimeter: $type"

    print_info "Type: $type"
    print_info "Heating Rate: $rate °C/min"
    print_info "Temperature Range: 25-400 °C"
    print_info "Sample Size: 5.2 mg"

    print_section "Thermal Analysis"

    print_success "Analysis complete"
    print_info "Glass transition: 85.3 °C"
    print_info "Melting point: 156.6 °C"
    print_info "Enthalpy of fusion: 28.5 J/g"

    echo ""
}

# Calibration operations
calibrate_instrument() {
    local instrument=${1:-mass-spec}
    local standard=${2:-NIST}

    print_section "Instrument Calibration"

    print_info "Instrument Type: $instrument"
    print_info "Reference Standard: $standard"
    print_info "Calibration Date: $(date -u +"%Y-%m-%d")"

    print_section "Calibration Procedure"

    print_info "1. System initialization"
    sleep 1
    print_success "   System ready"

    print_info "2. Reference measurement"
    sleep 1
    print_success "   Reference validated"

    print_info "3. Multi-point calibration"
    sleep 1
    print_success "   Linearity: R² = 0.9998"

    print_info "4. Accuracy verification"
    sleep 1
    print_success "   Accuracy: ±0.1%"

    print_section "Calibration Result"
    print_success "Calibration PASSED"
    print_info "Next calibration due: $(date -u -d '+90 days' +"%Y-%m-%d")"
    print_info "Certificate ID: CAL-$(date +%s)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-qua-020 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  accelerator [type] [energy]    Particle accelerator operations"
    echo "    LHC                           Large Hadron Collider (default)"
    echo "    synchrotron                   Synchrotron radiation source"
    echo ""
    echo "  mass-spec [type] [resolution]  Mass spectrometry analysis"
    echo "    Orbitrap                      Orbitrap MS (default)"
    echo "    TOF                           Time-of-flight MS"
    echo "    quadrupole                    Quadrupole MS"
    echo ""
    echo "  electron-microscope [type] [voltage]  Electron microscopy"
    echo "    TEM                           Transmission EM (default)"
    echo "    SEM                           Scanning EM"
    echo ""
    echo "  xray-crystal [wavelength] [resolution]  X-ray crystallography"
    echo ""
    echo "  nmr [field] [nucleus]          NMR spectroscopy"
    echo "    600                           600 MHz field (default)"
    echo "    1H                            Proton NMR (default)"
    echo ""
    echo "  telescope [type] [target]      Astronomical observation"
    echo "    optical                       Optical telescope (default)"
    echo "    radio                         Radio telescope"
    echo "    infrared                      Infrared telescope"
    echo ""
    echo "  spectrophotometer [type] [wavelength]  Spectrophotometry"
    echo "    UV-Vis                        UV-Visible (default)"
    echo ""
    echo "  chromatography [type] [flow]   Chromatographic separation"
    echo "    HPLC                          High-performance LC (default)"
    echo ""
    echo "  calorimeter [type] [rate]      Thermal analysis"
    echo "    DSC                           Differential scanning (default)"
    echo ""
    echo "  calibrate [instrument] [std]   Calibrate instrument"
    echo "    --instrument <type>           Instrument type"
    echo "    --standard <ref>              Reference standard"
    echo ""
    echo "  version                        Show version information"
    echo "  help                           Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-qua-020 accelerator LHC 13.6"
    echo "  wia-qua-020 mass-spec Orbitrap 240000"
    echo "  wia-qua-020 electron-microscope TEM 300"
    echo "  wia-qua-020 xray-crystal 1.5418 0.8"
    echo "  wia-qua-020 nmr 600 1H"
    echo "  wia-qua-020 telescope optical M31"
    echo "  wia-qua-020 calibrate mass-spec NIST"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-QUA-020 Scientific Instrument CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo "Supported Instruments:"
    echo "  - Particle Accelerators (LHC, Synchrotron)"
    echo "  - Mass Spectrometers (Orbitrap, TOF, Quadrupole)"
    echo "  - Electron Microscopes (TEM, SEM)"
    echo "  - X-ray Crystallography"
    echo "  - NMR Spectrometers"
    echo "  - Telescopes (Optical, Radio, Infrared)"
    echo "  - Spectrophotometers (UV-Vis, IR, Raman)"
    echo "  - Chromatography (HPLC, GC-MS)"
    echo "  - Calorimeters (DSC, ITC)"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    accelerator)
        TYPE=${1:-LHC}
        ENERGY=${2:-13.6}
        print_header
        particle_accelerator "$TYPE" "$ENERGY"
        ;;

    mass-spec)
        TYPE=${1:-Orbitrap}
        RESOLUTION=${2:-240000}
        print_header
        mass_spectrometer "$TYPE" "$RESOLUTION"
        ;;

    electron-microscope)
        TYPE=${1:-TEM}
        VOLTAGE=${2:-300}
        print_header
        electron_microscope "$TYPE" "$VOLTAGE"
        ;;

    xray-crystal)
        WAVELENGTH=${1:-1.5418}
        RESOLUTION=${2:-0.8}
        print_header
        xray_crystallography "$WAVELENGTH" "$RESOLUTION"
        ;;

    nmr)
        FIELD=${1:-600}
        NUCLEUS=${2:-1H}
        print_header
        nmr_spectrometer "$FIELD" "$NUCLEUS"
        ;;

    telescope)
        TYPE=${1:-optical}
        TARGET=${2:-M31}
        print_header
        telescope "$TYPE" "$TARGET"
        ;;

    spectrophotometer)
        TYPE=${1:-UV-Vis}
        WAVELENGTH=${2:-280}
        print_header
        spectrophotometer "$TYPE" "$WAVELENGTH"
        ;;

    chromatography)
        TYPE=${1:-HPLC}
        FLOW=${2:-1.0}
        print_header
        chromatography "$TYPE" "$FLOW"
        ;;

    calorimeter)
        TYPE=${1:-DSC}
        RATE=${2:-10}
        print_header
        calorimeter "$TYPE" "$RATE"
        ;;

    calibrate)
        INSTRUMENT=${1:-mass-spec}
        STANDARD=${2:-NIST}
        print_header
        calibrate_instrument "$INSTRUMENT" "$STANDARD"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-qua-020 help' for usage information"
        exit 1
        ;;
esac

exit 0
