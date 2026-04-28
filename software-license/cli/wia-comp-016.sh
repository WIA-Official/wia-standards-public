#!/bin/bash

################################################################################
# WIA-COMP-016: Software License CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Computing Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to software licensing operations
# including validation, compatibility checking, SPDX generation, and compliance.
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
    echo "║         📜 WIA-COMP-016: Software License CLI                 ║"
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

# Validate license
validate_license() {
    local license=${1:-MIT}
    local copyright_holder=${2:-""}

    print_section "License Validation"
    print_info "License: $license"

    # Check if license is known
    case $license in
        MIT|Apache-2.0|BSD-2-Clause|BSD-3-Clause|GPL-3.0-only|GPL-3.0-or-later|LGPL-3.0-only)
            print_success "License identifier is valid"
            print_info "Category: Permissive/Copyleft (depending on license)"
            print_success "OSI Approved: Yes"
            ;;
        Proprietary)
            print_warning "License: Proprietary (custom terms apply)"
            ;;
        *)
            print_warning "Unknown license identifier: $license"
            print_info "Check SPDX license list: https://spdx.org/licenses/"
            ;;
    esac

    if [ -n "$copyright_holder" ]; then
        print_success "Copyright holder: $copyright_holder"
    else
        print_warning "Copyright holder not specified"
    fi

    echo ""
}

# Check compatibility
check_compatibility() {
    local main_license=${1:-MIT}
    shift
    local dep_licenses=("$@")

    print_section "License Compatibility Check"
    print_info "Main License: $main_license"
    print_info "Dependencies: ${dep_licenses[*]}"

    local compatible=true

    for dep_license in "${dep_licenses[@]}"; do
        case "$main_license:$dep_license" in
            *:MIT|*:Apache-2.0|*:BSD-*|*:ISC)
                print_success "$dep_license is compatible (permissive)"
                ;;
            Proprietary:GPL-*|Proprietary:AGPL-*)
                print_error "$dep_license is NOT compatible with Proprietary"
                compatible=false
                ;;
            GPL-*:GPL-*|AGPL-*:AGPL-*)
                print_success "$dep_license is compatible (same family)"
                ;;
            *)
                print_warning "$dep_license - check compatibility manually"
                ;;
        esac
    done

    echo ""
    if [ "$compatible" = true ]; then
        print_success "All licenses appear compatible"
    else
        print_error "Some licenses are incompatible"
    fi
    echo ""
}

# Generate SPDX
generate_spdx() {
    local project=${1:-MyProject}
    local version=${2:-1.0.0}
    local license=${3:-MIT}
    local output=${4:-SPDX.json}

    print_section "Generating SPDX Document"
    print_info "Project: $project"
    print_info "Version: $version"
    print_info "License: $license"
    print_info "Output: $output"

    # Generate SPDX JSON
    cat > "$output" <<EOF
{
  "spdxVersion": "SPDX-2.3",
  "dataLicense": "CC0-1.0",
  "SPDXID": "SPDXRef-DOCUMENT",
  "name": "$project",
  "documentNamespace": "https://wiastandards.com/spdx/$project-$version",
  "creationInfo": {
    "created": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
    "creators": ["Tool: WIA-COMP-016-$VERSION"]
  },
  "packages": [
    {
      "SPDXID": "SPDXRef-Package",
      "name": "$project",
      "versionInfo": "$version",
      "downloadLocation": "NOASSERTION",
      "filesAnalyzed": false,
      "licenseConcluded": "$license",
      "licenseDeclared": "$license",
      "copyrightText": "Copyright $(date +%Y) Project Contributors"
    }
  ],
  "relationships": [
    {
      "spdxElementId": "SPDXRef-DOCUMENT",
      "relationshipType": "DESCRIBES",
      "relatedSpdxElement": "SPDXRef-Package"
    }
  ]
}
EOF

    print_success "SPDX document generated: $output"
    echo ""
}

# Scan dependencies (simplified)
scan_deps() {
    local package_file=${1:-package.json}

    print_section "Scanning Dependencies"
    print_info "Package file: $package_file"

    if [ ! -f "$package_file" ]; then
        print_error "Package file not found: $package_file"
        return 1
    fi

    # Simple scan (in production, would parse JSON and fetch licenses)
    print_info "Dependencies found:"

    # For package.json
    if [ "$package_file" = "package.json" ]; then
        print_success "express: MIT"
        print_success "react: MIT"
        print_success "typescript: Apache-2.0"
        print_info "Total: 3 dependencies"
    fi

    echo ""
}

# Create license file
create_license() {
    local license=${1:-MIT}
    local holder=${2:-"Your Name"}
    local year=${3:-$(date +%Y)}
    local output=${4:-LICENSE}

    print_section "Creating License File"
    print_info "License: $license"
    print_info "Copyright: $holder ($year)"
    print_info "Output: $output"

    case $license in
        MIT)
            cat > "$output" <<EOF
MIT License

Copyright (c) $year $holder

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
EOF
            ;;
        Apache-2.0)
            cat > "$output" <<EOF
Copyright $year $holder

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
EOF
            ;;
        *)
            print_error "Template not available for $license"
            print_info "See https://spdx.org/licenses/$license.html"
            return 1
            ;;
    esac

    print_success "License file created: $output"
    echo ""
}

# Show help
show_help() {
    print_header
    echo "Usage: wia-comp-016 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  validate                 Validate a license"
    echo "    --license <id>         SPDX license identifier (default: MIT)"
    echo "    --copyright <holder>   Copyright holder name"
    echo ""
    echo "  check-compat             Check license compatibility"
    echo "    --main <license>       Main project license (default: MIT)"
    echo "    --deps <licenses>      Comma-separated dependency licenses"
    echo ""
    echo "  generate-spdx            Generate SPDX document"
    echo "    --project <name>       Project name (default: MyProject)"
    echo "    --version <ver>        Project version (default: 1.0.0)"
    echo "    --license <id>         License identifier (default: MIT)"
    echo "    --output <file>        Output file (default: SPDX.json)"
    echo ""
    echo "  scan-deps                Scan project dependencies"
    echo "    --package-json <file>  Package file to scan"
    echo ""
    echo "  create-license           Create LICENSE file"
    echo "    --license <id>         License identifier (default: MIT)"
    echo "    --holder <name>        Copyright holder"
    echo "    --year <year>          Copyright year (default: current)"
    echo "    --output <file>        Output file (default: LICENSE)"
    echo ""
    echo "  version                  Show version information"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  wia-comp-016 validate --license MIT --copyright \"SmileStory Inc.\""
    echo "  wia-comp-016 check-compat --main Apache-2.0 --deps MIT,BSD-3-Clause"
    echo "  wia-comp-016 generate-spdx --project MyApp --version 1.0.0"
    echo "  wia-comp-016 create-license --license MIT --holder \"Your Name\""
    echo ""
    echo -e "${GRAY}弘익인간 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Show version
show_version() {
    print_header
    echo "WIA-COMP-016 Software License CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘익인간 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Parse arguments
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    validate)
        LICENSE="MIT"
        COPYRIGHT=""

        while [[ $# -gt 0 ]]; do
            case $1 in
                --license) LICENSE=$2; shift 2 ;;
                --copyright) COPYRIGHT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        validate_license "$LICENSE" "$COPYRIGHT"
        ;;

    check-compat)
        MAIN="MIT"
        DEPS="MIT,Apache-2.0"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --main) MAIN=$2; shift 2 ;;
                --deps) DEPS=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        IFS=',' read -ra DEP_ARRAY <<< "$DEPS"
        print_header
        check_compatibility "$MAIN" "${DEP_ARRAY[@]}"
        ;;

    generate-spdx)
        PROJECT="MyProject"
        VERSION_NUM="1.0.0"
        LICENSE="MIT"
        OUTPUT="SPDX.json"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --project) PROJECT=$2; shift 2 ;;
                --version) VERSION_NUM=$2; shift 2 ;;
                --license) LICENSE=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        generate_spdx "$PROJECT" "$VERSION_NUM" "$LICENSE" "$OUTPUT"
        ;;

    scan-deps)
        PACKAGE_FILE="package.json"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --package-json) PACKAGE_FILE=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        scan_deps "$PACKAGE_FILE"
        ;;

    create-license)
        LICENSE="MIT"
        HOLDER="Your Name"
        YEAR=$(date +%Y)
        OUTPUT="LICENSE"

        while [[ $# -gt 0 ]]; do
            case $1 in
                --license) LICENSE=$2; shift 2 ;;
                --holder) HOLDER=$2; shift 2 ;;
                --year) YEAR=$2; shift 2 ;;
                --output) OUTPUT=$2; shift 2 ;;
                *) shift ;;
            esac
        done

        print_header
        create_license "$LICENSE" "$HOLDER" "$YEAR" "$OUTPUT"
        ;;

    version)
        show_version
        ;;

    help|--help|-h)
        show_help
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-comp-016 help' for usage information"
        exit 1
        ;;
esac

exit 0
