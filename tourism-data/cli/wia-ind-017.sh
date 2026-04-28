#!/bin/bash

################################################################################
# WIA-IND-017: Tourism Data CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Standards Group
#
# 弘익人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to tourism data including
# attractions, destinations, POIs, crowd density, and visitor statistics.
################################################################################

set -e

# Colors for output
AMBER='\033[0;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
BLUE='\033[0;34m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
EARTH_RADIUS=6371000  # meters

# Helper functions
print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🗺️  WIA-IND-017: Tourism Data CLI                    ║"
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

print_data() {
    echo -e "${BLUE}  $1${RESET}"
}

# Calculate distance between two coordinates (Haversine formula)
calculate_distance() {
    local lat1=$1
    local lng1=$2
    local lat2=$3
    local lng2=$4

    # Convert to radians
    local phi1=$(echo "scale=10; $lat1 * 3.14159265359 / 180" | bc -l)
    local phi2=$(echo "scale=10; $lat2 * 3.14159265359 / 180" | bc -l)
    local delta_phi=$(echo "scale=10; ($lat2 - $lat1) * 3.14159265359 / 180" | bc -l)
    local delta_lambda=$(echo "scale=10; ($lng2 - $lng1) * 3.14159265359 / 180" | bc -l)

    # Haversine formula
    local a=$(echo "scale=10; s($delta_phi/2) * s($delta_phi/2) + c($phi1) * c($phi2) * s($delta_lambda/2) * s($delta_lambda/2)" | bc -l)
    local c=$(echo "scale=10; 2 * a(sqrt($a) / sqrt(1-$a))" | bc -l)
    local distance=$(echo "scale=2; $EARTH_RADIUS * $c" | bc -l)

    echo "$distance"
}

format_distance() {
    local meters=$1

    if (( $(echo "$meters < 1000" | bc -l) )); then
        printf "%.0f m" "$meters"
    else
        printf "%.2f km" "$(echo "$meters / 1000" | bc -l)"
    fi
}

# Search tourist attractions
search_attractions() {
    local lat=${1:-48.8566}
    local lng=${2:-2.3522}
    local radius=${3:-5000}
    local category=${4:-all}

    print_section "Searching Tourist Attractions"
    print_info "Location: ${lat}, ${lng}"
    print_info "Radius: $(format_distance $radius)"
    if [ "$category" != "all" ]; then
        print_info "Category: $category"
    fi

    # Mock data - In production, this would query an API
    print_section "Results"

    echo -e "${GREEN}1. Eiffel Tower${RESET}"
    print_data "   Category: Historical Monument"
    print_data "   Rating: ⭐⭐⭐⭐⭐ 4.7 (125,000 reviews)"
    print_data "   Location: 48.8584°N, 2.2945°E"
    print_data "   Distance: 1.8 km"
    print_data "   Status: Open • Wait time: 45 min"
    print_data "   Tickets: Adult €26.10, Child €13.10"

    echo ""
    echo -e "${GREEN}2. Louvre Museum${RESET}"
    print_data "   Category: Cultural Museum"
    print_data "   Rating: ⭐⭐⭐⭐⭐ 4.8 (250,000 reviews)"
    print_data "   Location: 48.8606°N, 2.3376°E"
    print_data "   Distance: 1.2 km"
    print_data "   Status: Open • Wait time: 30 min"
    print_data "   Tickets: Adult €17.00, Free under 18"

    echo ""
    echo -e "${GREEN}3. Arc de Triomphe${RESET}"
    print_data "   Category: Historical Monument"
    print_data "   Rating: ⭐⭐⭐⭐ 4.6 (95,000 reviews)"
    print_data "   Location: 48.8738°N, 2.2950°E"
    print_data "   Distance: 2.3 km"
    print_data "   Status: Open • Wait time: 20 min"
    print_data "   Tickets: Adult €13.00, Free under 18"

    print_section "Summary"
    print_success "Found 3 attractions within $(format_distance $radius)"
    print_info "Use --id <attraction-id> to get more details"

    echo ""
}

# Get destination information
get_destination() {
    local id=${1:-paris-france}
    local lang=${2:-en}

    print_section "Destination Information"
    print_info "Destination ID: $id"
    print_info "Language: $lang"

    print_section "Overview"
    echo -e "${AMBER}📍 Paris, France${RESET}"
    print_data "Type: City"
    print_data "Population: 2,161,000"
    print_data "Area: 105.4 km²"
    print_data "Time Zone: CET (UTC+1)"
    print_data "Currency: Euro (€)"
    print_data "Languages: French, English"

    print_section "Tourism"
    print_data "Annual Visitors: 30 million"
    print_data "Peak Season: June - August"
    print_data "Best Time to Visit: Spring (April-May), Fall (September-October)"
    print_data "Average Daily Budget: €100-150"
    print_data "Safety Rating: ⭐⭐⭐⭐ 4.0/5"

    print_section "Climate"
    print_data "Climate Type: Oceanic"
    print_data "Average Temperature:"
    print_info "  Winter: 3-7°C  | Spring: 8-16°C"
    print_info "  Summer: 15-25°C | Fall: 8-16°C"
    print_data "Rainfall: ~640mm annually"

    print_section "Top Attractions"
    print_data "1. Eiffel Tower"
    print_data "2. Louvre Museum"
    print_data "3. Notre-Dame Cathedral"
    print_data "4. Arc de Triomphe"
    print_data "5. Sacré-Cœur Basilica"

    echo ""
}

# Get real-time crowd density
crowd_density() {
    local attraction_id=${1:-eiffel-tower}
    local realtime=${2:-true}

    print_section "Crowd Density Information"
    print_info "Attraction: $attraction_id"
    print_info "Real-time: $realtime"
    print_info "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"

    print_section "Current Status"

    # Mock crowd data
    local current=2500
    local capacity=3000
    local percentage=$(( (current * 100) / capacity ))

    print_data "Current Visitors: $current"
    print_data "Capacity: $capacity"

    if [ $percentage -lt 50 ]; then
        echo -e "${GREEN}  Occupancy: ${percentage}% (Low)${RESET}"
    elif [ $percentage -lt 75 ]; then
        echo -e "${YELLOW}  Occupancy: ${percentage}% (Moderate)${RESET}"
    else
        echo -e "${RED}  Occupancy: ${percentage}% (High)${RESET}"
    fi

    print_data "Estimated Wait Time: 45 minutes"

    print_section "Crowd Level Forecast"
    print_info "Next 3 hours:"
    echo -e "  ${date +%H}:00 - Current  ${RED}███████████░░${RESET} High (83%)"
    echo -e "  $(date -d '+1 hour' +%H):00 - +1hr     ${RED}█████████████${RESET} Very High (95%)"
    echo -e "  $(date -d '+2 hours' +%H):00 - +2hr     ${RED}████████████░${RESET} Very High (90%)"
    echo -e "  $(date -d '+3 hours' +%H):00 - +3hr     ${YELLOW}█████████░░░░${RESET} High (70%)"

    print_section "Recommendations"
    print_warning "Peak crowd level expected in 1 hour"
    print_success "Consider visiting after 6 PM for lower crowds"

    echo ""
}

# Get visitor statistics
visitor_stats() {
    local attraction_id=${1:-eiffel-tower}
    local period=${2:-2024}
    local granularity=${3:-monthly}

    print_section "Visitor Statistics"
    print_info "Attraction: $attraction_id"
    print_info "Period: $period"
    print_info "Granularity: $granularity"

    print_section "Overall Statistics"
    print_data "Total Visitors (2024): 7,000,000"
    print_data "Average Daily Visitors: 19,178"
    print_data "Peak Day: July 14 (Bastille Day) - 35,000 visitors"
    print_data "Lowest Day: January 3 - 8,500 visitors"

    print_section "Monthly Breakdown"
    print_info "Month      | Visitors  | Revenue     | Satisfaction"
    print_info "-----------|-----------|-------------|-------------"
    print_data "January    | 450,000   | €8.2M       | 4.5/5"
    print_data "February   | 480,000   | €8.7M       | 4.6/5"
    print_data "March      | 550,000   | €10.0M      | 4.7/5"
    print_data "April      | 620,000   | €11.3M      | 4.8/5"
    print_data "May        | 680,000   | €12.4M      | 4.7/5"
    print_data "June       | 750,000   | €13.7M      | 4.6/5"
    print_data "July       | 820,000   | €15.0M      | 4.5/5"
    print_data "August     | 800,000   | €14.6M      | 4.4/5"
    print_data "September  | 720,000   | €13.1M      | 4.7/5"
    print_data "October    | 640,000   | €11.7M      | 4.8/5"
    print_data "November   | 520,000   | €9.5M       | 4.7/5"
    print_data "December   | 470,000   | €8.6M       | 4.6/5"

    print_section "Demographics"
    print_data "International Visitors: 65%"
    print_data "Domestic Visitors: 35%"
    print_data "Top 5 Origin Countries:"
    print_info "  1. 🇺🇸 United States (18%)"
    print_info "  2. 🇬🇧 United Kingdom (12%)"
    print_info "  3. 🇩🇪 Germany (10%)"
    print_info "  4. 🇮🇹 Italy (8%)"
    print_info "  5. 🇯🇵 Japan (7%)"

    echo ""
}

# Search points of interest
search_poi() {
    local category=${1:-restaurants}
    local city=${2:-paris}
    local rating=${3:-4.0}

    print_section "Searching Points of Interest"
    print_info "Category: $category"
    print_info "City: $city"
    print_info "Minimum Rating: $rating"

    print_section "Results"

    echo -e "${GREEN}1. Le Jules Verne${RESET}"
    print_data "   Type: Fine Dining Restaurant"
    print_data "   Rating: ⭐⭐⭐⭐⭐ 4.8 (2,500 reviews)"
    print_data "   Price: $$$$"
    print_data "   Cuisine: French, Contemporary"
    print_data "   Location: Eiffel Tower, 2nd Floor"
    print_data "   Status: Open • Reservations Required"

    echo ""
    echo -e "${GREEN}2. L'Astrance${RESET}"
    print_data "   Type: Fine Dining Restaurant"
    print_data "   Rating: ⭐⭐⭐⭐⭐ 4.9 (1,800 reviews)"
    print_data "   Price: $$$$"
    print_data "   Cuisine: French, Innovative"
    print_data "   Location: 16th Arrondissement"
    print_data "   Status: Closed • Opens at 19:00"

    echo ""
    echo -e "${GREEN}3. Septime${RESET}"
    print_data "   Type: Contemporary Bistro"
    print_data "   Rating: ⭐⭐⭐⭐ 4.7 (3,200 reviews)"
    print_data "   Price: $$$"
    print_data "   Cuisine: French, Seasonal"
    print_data "   Location: 11th Arrondissement"
    print_data "   Status: Open • Limited availability"

    print_section "Summary"
    print_success "Found 3 POIs matching criteria"

    echo ""
}

# Get accessibility information
accessibility() {
    local attraction_id=${1:-louvre-museum}

    print_section "Accessibility Information"
    print_info "Attraction: $attraction_id"

    print_section "Wheelchair Accessibility"
    print_success "Wheelchair accessible entrances"
    print_success "Elevators available to all floors"
    print_success "Accessible restrooms"
    print_success "Wheelchair rentals available (free)"

    print_section "Visual Assistance"
    print_success "Audio guides available"
    print_data "   Languages: English, French, Spanish, German, Italian, Japanese, Chinese"
    print_success "Braille signage throughout"
    print_success "Tactile tours available (advance booking)"
    print_success "Service animals welcome"

    print_section "Hearing Assistance"
    print_success "Sign language interpretation (French Sign Language)"
    print_data "   Available: Wednesday, Saturday (advance booking)"
    print_success "Assistive listening devices"
    print_success "Written guides available"

    print_section "Other Accommodations"
    print_success "Priority access for visitors with disabilities"
    print_success "Dedicated parking spaces"
    print_success "Seating areas throughout venue"
    print_info "Sensory-friendly hours: First Sunday of month, 9-10 AM"

    print_section "Accessibility Rating"
    echo -e "${GREEN}  Overall: ⭐⭐⭐⭐⭐ 4.8/5${RESET}"
    print_info "Based on 1,200 accessibility reviews"

    print_section "Contact"
    print_data "Accessibility Coordinator: +33 1 40 20 53 17"
    print_data "Email: accessibility@louvre.fr"

    echo ""
}

# Export tourism data
export_data() {
    local destination=${1:-paris-france}
    local format=${2:-json}
    local output=${3:-tourism-data.$format}

    print_section "Exporting Tourism Data"
    print_info "Destination: $destination"
    print_info "Format: $format"
    print_info "Output: $output"

    # Mock export
    print_info "Collecting data..."
    sleep 1

    case $format in
        json)
            echo '{"destination":"paris-france","attractions":[],"pois":[]}' > "$output"
            ;;
        csv)
            echo "id,name,category,rating" > "$output"
            echo "1,Eiffel Tower,monument,4.7" >> "$output"
            ;;
        xml)
            echo '<?xml version="1.0"?><tourism></tourism>' > "$output"
            ;;
    esac

    print_success "Data exported to: $output"
    print_info "File size: $(du -h "$output" | cut -f1)"

    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-ind-017 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  search-attractions       Search tourist attractions"
    echo "    --lat <latitude>       Latitude (default: 48.8566)"
    echo "    --lng <longitude>      Longitude (default: 2.3522)"
    echo "    --radius <meters>      Search radius (default: 5000)"
    echo "    --category <name>      Category filter (default: all)"
    echo ""
    echo "  get-destination          Get destination information"
    echo "    --id <dest-id>         Destination ID (default: paris-france)"
    echo "    --lang <code>          Language code (default: en)"
    echo ""
    echo "  crowd-density            Get real-time crowd data"
    echo "    --attraction-id <id>   Attraction ID (default: eiffel-tower)"
    echo "    --realtime             Use real-time data (default: true)"
    echo ""
    echo "  visitor-stats            Get visitor statistics"
    echo "    --attraction-id <id>   Attraction ID (default: eiffel-tower)"
    echo "    --period <year>        Year (default: 2024)"
    echo "    --granularity <type>   monthly|weekly|daily (default: monthly)"
    echo ""
    echo "  search-poi               Search points of interest"
    echo "    --category <name>      Category (default: restaurants)"
    echo "    --city <name>          City (default: paris)"
    echo "    --rating <min>         Minimum rating (default: 4.0)"
    echo ""
    echo "  accessibility            Get accessibility information"
    echo "    --attraction-id <id>   Attraction ID (default: louvre-museum)"
    echo ""
    echo "  export                   Export tourism data"
    echo "    --destination <id>     Destination ID (default: paris-france)"
    echo "    --format <type>        json|csv|xml (default: json)"
    echo "    --output <file>        Output filename"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-ind-017 search-attractions --lat 48.8566 --lng 2.3522 --radius 5000"
    echo "  wia-ind-017 get-destination --id tokyo-japan --lang en"
    echo "  wia-ind-017 crowd-density --attraction-id louvre-museum --realtime"
    echo "  wia-ind-017 visitor-stats --attraction-id eiffel-tower --period 2024"
    echo "  wia-ind-017 search-poi --category restaurants --city paris --rating 4.5"
    echo "  wia-ind-017 accessibility --attraction-id louvre-museum"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-IND-017 Tourism Data CLI Tool"
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
    search-attractions)
        LAT=48.8566
        LNG=2.3522
        RADIUS=5000
        CATEGORY="all"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --lat) LAT=$2; shift 2 ;;
                --lng) LNG=$2; shift 2 ;;
                --radius) RADIUS=$2; shift 2 ;;
                --category) CATEGORY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        search_attractions "$LAT" "$LNG" "$RADIUS" "$CATEGORY"
        ;;

    get-destination)
        ID="paris-france"
        LANG="en"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --id) ID=$2; shift 2 ;;
                --lang) LANG=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        get_destination "$ID" "$LANG"
        ;;

    crowd-density)
        ATTRACTION_ID="eiffel-tower"
        REALTIME=true

        while [[ $# -gt 0 ]]; do
            case $1 in
                --attraction-id) ATTRACTION_ID=$2; shift 2 ;;
                --realtime) REALTIME=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        crowd_density "$ATTRACTION_ID" "$REALTIME"
        ;;

    visitor-stats)
        ATTRACTION_ID="eiffel-tower"
        PERIOD="2024"
        GRANULARITY="monthly"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --attraction-id) ATTRACTION_ID=$2; shift 2 ;;
                --period) PERIOD=$2; shift 2 ;;
                --granularity) GRANULARITY=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        visitor_stats "$ATTRACTION_ID" "$PERIOD" "$GRANULARITY"
        ;;

    search-poi)
        CATEGORY="restaurants"
        CITY="paris"
        RATING=4.0

        while [[ $# -gt 0 ]]; do
            case $1 in
                --category) CATEGORY=$2; shift 2 ;;
                --city) CITY=$2; shift 2 ;;
                --rating) RATING=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        search_poi "$CATEGORY" "$CITY" "$RATING"
        ;;

    accessibility)
        ATTRACTION_ID="louvre-museum"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --attraction-id) ATTRACTION_ID=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        accessibility "$ATTRACTION_ID"
        ;;

    export)
        DESTINATION="paris-france"
        FORMAT="json"
        OUTPUT=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --destination) DESTINATION=$2; shift 2 ;;
                --format) FORMAT=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        if [ -z "$OUTPUT" ]; then
            OUTPUT="tourism-data.$FORMAT"
        fi

        print_header
        export_data "$DESTINATION" "$FORMAT" "$OUTPUT"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-ind-017 help' for usage information"
        exit 1
        ;;
esac

exit 0

# ============================================================================
# 弘益人間 (홍익인간) · Benefit All Humanity
# ============================================================================
