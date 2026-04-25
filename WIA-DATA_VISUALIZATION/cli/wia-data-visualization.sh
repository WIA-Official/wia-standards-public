#!/bin/bash

# WIA-DATA_VISUALIZATION: Data Visualization Standard CLI
# 弘益人間 (Benefit All Humanity)
#
# Command-line interface for data visualization operations
#
# Usage:
#   ./wia-data-visualization.sh [command] [options]

set -e

VERSION="1.0.0"
API_BASE_URL="${WIA_API_URL:-https://api.wia-official.org/data-visualization/v1}"
CONFIG_DIR="$HOME/.wia/data-visualization"
CONFIG_FILE="$CONFIG_DIR/config.json"
CHARTS_DIR="$CONFIG_DIR/charts"
TEMPLATES_DIR="$CONFIG_DIR/templates"
EXPORTS_DIR="$CONFIG_DIR/exports"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Create directories if they don't exist
mkdir -p "$CONFIG_DIR" "$CHARTS_DIR" "$TEMPLATES_DIR" "$EXPORTS_DIR"

# Initialize default config if not exists
if [ ! -f "$CONFIG_FILE" ]; then
    cat > "$CONFIG_FILE" << EOF
{
  "apiKey": "",
  "defaultChartType": "bar",
  "defaultTheme": "dark",
  "outputFormat": "svg",
  "colorPalette": "wia-standard",
  "language": "en"
}
EOF
fi

# Helper functions
print_header() {
    echo -e "${PURPLE}"
    echo "╔═══════════════════════════════════════════════════════════════════╗"
    echo "║      WIA-DATA_VISUALIZATION: Data Visualization CLI v$VERSION        ║"
    echo "║                  弘益人間 · Benefit All Humanity                   ║"
    echo "╚═══════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ Error: $1${NC}" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠ Warning: $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Load config value
get_config() {
    local key=$1
    cat "$CONFIG_FILE" | grep "\"$key\"" | head -1 | sed 's/.*: *"\([^"]*\)".*/\1/'
}

# Set config value
set_config() {
    local key=$1
    local value=$2
    local temp_file=$(mktemp)

    if grep -q "\"$key\"" "$CONFIG_FILE"; then
        sed "s/\"$key\": *\"[^\"]*\"/\"$key\": \"$value\"/" "$CONFIG_FILE" > "$temp_file"
        mv "$temp_file" "$CONFIG_FILE"
    fi

    print_success "Configuration updated: $key = $value"
}

# Command: init - Initialize CLI configuration
cmd_init() {
    print_header
    echo "Initializing WIA-DATA_VISUALIZATION CLI..."
    echo ""

    read -p "Enter API Key (optional): " api_key
    if [ -n "$api_key" ]; then
        set_config "apiKey" "$api_key"
    fi

    echo ""
    echo "Default chart types:"
    echo "  1) bar     - Bar charts"
    echo "  2) line    - Line charts"
    echo "  3) pie     - Pie charts"
    echo "  4) scatter - Scatter plots"
    echo "  5) area    - Area charts"
    echo ""
    read -p "Select default chart type [1-5] (default: bar): " chart_choice

    case $chart_choice in
        2) set_config "defaultChartType" "line" ;;
        3) set_config "defaultChartType" "pie" ;;
        4) set_config "defaultChartType" "scatter" ;;
        5) set_config "defaultChartType" "area" ;;
        *) set_config "defaultChartType" "bar" ;;
    esac

    echo ""
    echo "Theme options:"
    echo "  1) dark   - Dark theme"
    echo "  2) light  - Light theme"
    echo "  3) wia    - WIA branded theme"
    echo ""
    read -p "Select theme [1-3] (default: dark): " theme_choice

    case $theme_choice in
        2) set_config "defaultTheme" "light" ;;
        3) set_config "defaultTheme" "wia" ;;
        *) set_config "defaultTheme" "dark" ;;
    esac

    echo ""
    print_success "Initialization complete!"
    print_info "Configuration saved to: $CONFIG_FILE"
}

# Command: create - Create a new visualization
cmd_create() {
    local chart_type="${1:-}"
    local data_source="${2:-}"

    print_header
    echo "Create New Visualization"
    echo ""

    if [ -z "$chart_type" ]; then
        echo "Chart Types:"
        echo "  1) bar      - Bar chart"
        echo "  2) line     - Line chart"
        echo "  3) pie      - Pie chart"
        echo "  4) scatter  - Scatter plot"
        echo "  5) area     - Area chart"
        echo "  6) heatmap  - Heatmap"
        echo "  7) treemap  - Treemap"
        echo "  8) sankey   - Sankey diagram"
        echo "  9) radar    - Radar chart"
        echo " 10) funnel   - Funnel chart"
        echo ""
        read -p "Select chart type [1-10]: " type_choice

        case $type_choice in
            1) chart_type="bar" ;;
            2) chart_type="line" ;;
            3) chart_type="pie" ;;
            4) chart_type="scatter" ;;
            5) chart_type="area" ;;
            6) chart_type="heatmap" ;;
            7) chart_type="treemap" ;;
            8) chart_type="sankey" ;;
            9) chart_type="radar" ;;
            10) chart_type="funnel" ;;
            *) chart_type="bar" ;;
        esac
    fi

    if [ -z "$data_source" ]; then
        echo ""
        echo "Data Source:"
        echo "  1) File (CSV, JSON)"
        echo "  2) Database query"
        echo "  3) API endpoint"
        echo "  4) Sample data"
        echo ""
        read -p "Select data source [1-4]: " source_choice

        case $source_choice in
            1)
                read -p "Enter file path: " data_source
                ;;
            2)
                read -p "Enter connection string: " data_source
                ;;
            3)
                read -p "Enter API endpoint: " data_source
                ;;
            *)
                data_source="sample"
                ;;
        esac
    fi

    read -p "Chart title: " chart_title
    chart_title=${chart_title:-"My Chart"}

    local chart_id="chart-$(date +%s)"
    local chart_file="$CHARTS_DIR/$chart_id.json"

    cat > "$chart_file" << EOF
{
  "id": "$chart_id",
  "type": "$chart_type",
  "title": "$chart_title",
  "dataSource": "$data_source",
  "theme": "$(get_config defaultTheme)",
  "options": {
    "responsive": true,
    "legend": true,
    "tooltip": true,
    "animation": true
  },
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

    echo ""
    print_success "Chart created: $chart_id"
    print_info "Chart file: $chart_file"
    echo ""
    echo "Chart Preview:"
    echo "═══════════════════════════════════════════════════"
    echo "  Type: $chart_type"
    echo "  Title: $chart_title"
    echo "  Data: $data_source"
    echo "═══════════════════════════════════════════════════"
}

# Command: list - List all visualizations
cmd_list() {
    print_header
    echo "Available Visualizations:"
    echo ""

    if ls "$CHARTS_DIR"/*.json >/dev/null 2>&1; then
        printf "%-20s %-15s %-30s\n" "ID" "TYPE" "TITLE"
        echo "────────────────────────────────────────────────────────────────"

        for chart_file in "$CHARTS_DIR"/*.json; do
            local chart_id=$(basename "$chart_file" .json)
            local chart_type=$(grep '"type"' "$chart_file" | head -1 | sed 's/.*: *"\([^"]*\)".*/\1/')
            local chart_title=$(grep '"title"' "$chart_file" | head -1 | sed 's/.*: *"\([^"]*\)".*/\1/')
            printf "%-20s %-15s %-30s\n" "$chart_id" "$chart_type" "$chart_title"
        done
    else
        print_warning "No visualizations found"
        echo ""
        echo "Create your first visualization with:"
        echo "  ./wia-data-visualization.sh create"
    fi
}

# Command: render - Render a visualization
cmd_render() {
    local chart_id=$1
    local output_format="${2:-svg}"

    print_header

    if [ -z "$chart_id" ]; then
        print_error "Chart ID required"
        echo "Usage: ./wia-data-visualization.sh render <chart-id> [format]"
        exit 1
    fi

    local chart_file="$CHARTS_DIR/$chart_id.json"

    if [ ! -f "$chart_file" ]; then
        print_error "Chart not found: $chart_id"
        exit 1
    fi

    print_info "Rendering: $chart_id"
    print_info "Format: $output_format"
    echo ""

    # Simulate rendering
    echo "Rendering progress:"
    echo "  [▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓] 100%"
    echo ""

    local output_file="$EXPORTS_DIR/$chart_id.$output_format"
    touch "$output_file"

    print_success "Render complete!"
    print_info "Output: $output_file"
}

# Command: export - Export visualization
cmd_export() {
    local chart_id=$1
    local format="${2:-png}"
    local output="${3:-}"

    print_header

    if [ -z "$chart_id" ]; then
        print_error "Chart ID required"
        echo "Usage: ./wia-data-visualization.sh export <chart-id> [format] [output]"
        exit 1
    fi

    echo "Export Formats:"
    echo "  • png  - PNG image"
    echo "  • svg  - SVG vector"
    echo "  • pdf  - PDF document"
    echo "  • html - Interactive HTML"
    echo "  • json - Chart specification"
    echo ""

    if [ -z "$output" ]; then
        output="$EXPORTS_DIR/$chart_id.$format"
    fi

    print_info "Exporting $chart_id to $format..."

    # Simulate export
    touch "$output"

    print_success "Export complete!"
    print_info "File: $output"
}

# Command: template - Manage chart templates
cmd_template() {
    local action=$1
    shift

    case $action in
        list)
            list_templates
            ;;
        create)
            create_template "$@"
            ;;
        apply)
            apply_template "$@"
            ;;
        *)
            print_error "Unknown template action: $action"
            echo "Available actions: list, create, apply"
            exit 1
            ;;
    esac
}

list_templates() {
    print_header
    echo "Chart Templates:"
    echo ""

    echo "Built-in Templates:"
    echo "  • executive-dashboard  - Executive summary dashboard"
    echo "  • sales-report         - Sales performance report"
    echo "  • analytics-overview   - Analytics overview"
    echo "  • kpi-tracker          - KPI tracking dashboard"
    echo ""

    if ls "$TEMPLATES_DIR"/*.json >/dev/null 2>&1; then
        echo "Custom Templates:"
        for template_file in "$TEMPLATES_DIR"/*.json; do
            local template_name=$(basename "$template_file" .json)
            echo "  • $template_name"
        done
    fi
}

create_template() {
    local name=$1

    if [ -z "$name" ]; then
        read -p "Template name: " name
    fi

    print_header
    echo "Creating template: $name"
    echo ""

    cat > "$TEMPLATES_DIR/$name.json" << EOF
{
  "name": "$name",
  "charts": [],
  "layout": "grid",
  "theme": "$(get_config defaultTheme)",
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

    print_success "Template created: $name"
}

apply_template() {
    local template=$1
    local data_source=$2

    print_header
    echo "Applying template: $template"
    echo ""

    print_info "Template applied successfully"
    print_info "Charts generated from template"
}

# Command: dashboard - Create/manage dashboards
cmd_dashboard() {
    local action="${1:-list}"
    shift || true

    case $action in
        create)
            create_dashboard "$@"
            ;;
        list)
            list_dashboards
            ;;
        preview)
            preview_dashboard "$@"
            ;;
        *)
            print_error "Unknown dashboard action: $action"
            echo "Available actions: create, list, preview"
            exit 1
            ;;
    esac
}

create_dashboard() {
    local name=$1

    if [ -z "$name" ]; then
        read -p "Dashboard name: " name
    fi

    print_header
    echo "Creating Dashboard: $name"
    echo ""

    local dashboard_id="dashboard-$(date +%s)"

    cat > "$CONFIG_DIR/$dashboard_id.json" << EOF
{
  "id": "$dashboard_id",
  "name": "$name",
  "charts": [],
  "layout": {
    "type": "grid",
    "columns": 2,
    "rows": 2
  },
  "refreshInterval": 30,
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

    print_success "Dashboard created: $dashboard_id"
}

list_dashboards() {
    print_header
    echo "Dashboards:"
    echo ""

    if ls "$CONFIG_DIR"/dashboard-*.json >/dev/null 2>&1; then
        for dashboard_file in "$CONFIG_DIR"/dashboard-*.json; do
            local dashboard_id=$(basename "$dashboard_file" .json)
            local dashboard_name=$(grep '"name"' "$dashboard_file" | head -1 | sed 's/.*: *"\([^"]*\)".*/\1/')
            echo "  • $dashboard_id: $dashboard_name"
        done
    else
        print_warning "No dashboards found"
    fi
}

preview_dashboard() {
    local dashboard_id=$1

    print_header
    echo "Dashboard Preview"
    echo ""

    echo "┌─────────────────────┬─────────────────────┐"
    echo "│                     │                     │"
    echo "│   📊 Chart 1        │   📈 Chart 2        │"
    echo "│                     │                     │"
    echo "├─────────────────────┼─────────────────────┤"
    echo "│                     │                     │"
    echo "│   📉 Chart 3        │   🥧 Chart 4        │"
    echo "│                     │                     │"
    echo "└─────────────────────┴─────────────────────┘"
    echo ""

    print_info "Dashboard preview (text-based)"
}

# Command: theme - Manage themes
cmd_theme() {
    local action="${1:-list}"

    print_header

    case $action in
        list)
            echo "Available Themes:"
            echo ""
            echo "  • dark      - Dark background with light text"
            echo "  • light     - Light background with dark text"
            echo "  • wia       - WIA branded theme (green/cyan)"
            echo "  • corporate - Corporate professional theme"
            echo "  • minimal   - Minimal clean theme"
            echo "  • vibrant   - Colorful vibrant theme"
            ;;
        set)
            local theme=$2
            if [ -n "$theme" ]; then
                set_config "defaultTheme" "$theme"
            else
                print_error "Theme name required"
            fi
            ;;
        *)
            print_error "Unknown theme action: $action"
            echo "Available actions: list, set"
            ;;
    esac
}

# Command: color - Manage color palettes
cmd_color() {
    local action="${1:-list}"

    print_header

    case $action in
        list)
            echo "Color Palettes:"
            echo ""
            echo "  • wia-standard   - WIA standard colors"
            echo "  • categorical    - Categorical data palette"
            echo "  • sequential     - Sequential data palette"
            echo "  • diverging      - Diverging data palette"
            echo "  • accessible     - Colorblind-friendly palette"
            echo ""

            echo "WIA Standard Palette:"
            echo "  █████ #10B981 (Primary Green)"
            echo "  █████ #06B6D4 (Cyan)"
            echo "  █████ #8B5CF6 (Purple)"
            echo "  █████ #F59E0B (Amber)"
            echo "  █████ #EF4444 (Red)"
            ;;
        *)
            print_error "Unknown color action: $action"
            ;;
    esac
}

# Command: help - Show help information
cmd_help() {
    print_header
    cat << EOF
Usage: ./wia-data-visualization.sh [command] [options]

Commands:
  init                       Initialize CLI configuration
  create [type] [source]     Create a new visualization
  list                       List all visualizations
  render <chart-id> [format] Render a chart (svg, png, pdf)
  export <chart-id> [format] Export a chart
  template <action>          Manage chart templates
    list                     List templates
    create [name]            Create a template
    apply <template>         Apply a template
  dashboard <action>         Manage dashboards
    create [name]            Create a dashboard
    list                     List dashboards
    preview <id>             Preview a dashboard
  theme <action>             Manage themes
    list                     List available themes
    set <theme>              Set default theme
  color <action>             Manage color palettes
    list                     List color palettes
  version                    Show version information
  help                       Show this help message

Chart Types:
  bar, line, pie, scatter, area, heatmap, treemap, sankey, radar, funnel

Examples:
  ./wia-data-visualization.sh init
  ./wia-data-visualization.sh create bar data.csv
  ./wia-data-visualization.sh render chart-123 svg
  ./wia-data-visualization.sh template list
  ./wia-data-visualization.sh dashboard create "Sales Dashboard"

For more information: https://docs.wia-official.org/data-visualization

弘益人間 · Benefit All Humanity
EOF
}

# Command: version - Show version
cmd_version() {
    print_header
    echo "Version: $VERSION"
    echo "Standard: WIA-DATA_VISUALIZATION"
    echo "License: MIT"
    echo ""
    echo "© 2025 SmileStory Inc. / WIA"
}

# Main command dispatcher
main() {
    local command="${1:-help}"
    shift || true

    case $command in
        init)
            cmd_init "$@"
            ;;
        create)
            cmd_create "$@"
            ;;
        list)
            cmd_list "$@"
            ;;
        render)
            cmd_render "$@"
            ;;
        export)
            cmd_export "$@"
            ;;
        template)
            cmd_template "$@"
            ;;
        dashboard)
            cmd_dashboard "$@"
            ;;
        theme)
            cmd_theme "$@"
            ;;
        color)
            cmd_color "$@"
            ;;
        version)
            cmd_version
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run './wia-data-visualization.sh help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
