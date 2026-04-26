#!/bin/bash

################################################################################
# WIA-IND-025: Quality Control Standard - Installation Script
#
# This script installs the WIA-IND-025 Quality Control standard, including:
# - TypeScript SDK
# - CLI tools
# - Documentation
# - Quality control templates
# - SPC calculators
# - Certification frameworks
#
# Usage: ./install.sh
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Emoji
CHECK_MARK="✅"
CROSS_MARK="❌"
ROCKET="🚀"
PACKAGE="📦"
WRENCH="🔧"
DOCS="📚"
GEAR="⚙️"
CHART="📊"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                                                            ║${NC}"
echo -e "${BLUE}║  ${CHECK_MARK}  WIA-IND-025: Quality Control Standard Installation  ║${NC}"
echo -e "${BLUE}║                                                            ║${NC}"
echo -e "${BLUE}║  Version: 1.0.0                                            ║${NC}"
echo -e "${BLUE}║  Category: IND (Industry)                                  ║${NC}"
echo -e "${BLUE}║  Color: Amber (#F59E0B)                                    ║${NC}"
echo -e "${BLUE}║                                                            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${PURPLE}弘益人間 (Benefit All Humanity)${NC}"
echo ""

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   echo -e "${YELLOW}⚠️  Warning: Running as root. Installation will be system-wide.${NC}"
   INSTALL_DIR="/usr/local"
   BIN_DIR="/usr/local/bin"
else
   echo -e "${CYAN}ℹ️  Installing for current user.${NC}"
   INSTALL_DIR="$HOME/.wia"
   BIN_DIR="$HOME/.local/bin"
fi

echo ""
echo -e "${CYAN}Installation directory: ${INSTALL_DIR}${NC}"
echo -e "${CYAN}Binary directory: ${BIN_DIR}${NC}"
echo ""

# Create directories
echo -e "${YELLOW}${WRENCH} Creating installation directories...${NC}"
mkdir -p "$INSTALL_DIR/wia-standards/quality-control"
mkdir -p "$BIN_DIR"
mkdir -p "$HOME/.wia/quality-control/templates"
mkdir -p "$HOME/.wia/quality-control/certificates"
mkdir -p "$HOME/.wia/quality-control/calibration"
mkdir -p "$HOME/.wia/quality-control/spc-charts"
echo -e "${GREEN}${CHECK_MARK} Directories created${NC}"
echo ""

# Copy standard files
echo -e "${YELLOW}${PACKAGE} Installing WIA-IND-025 Quality Control Standard...${NC}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Copy specification
if [ -d "$SCRIPT_DIR/spec" ]; then
    cp -r "$SCRIPT_DIR/spec" "$INSTALL_DIR/wia-standards/quality-control/"
    echo -e "${GREEN}${CHECK_MARK} Specification installed${NC}"
fi

# Copy API
if [ -d "$SCRIPT_DIR/api" ]; then
    cp -r "$SCRIPT_DIR/api" "$INSTALL_DIR/wia-standards/quality-control/"
    echo -e "${GREEN}${CHECK_MARK} API installed${NC}"
fi

# Copy CLI
if [ -d "$SCRIPT_DIR/cli" ]; then
    cp -r "$SCRIPT_DIR/cli" "$INSTALL_DIR/wia-standards/quality-control/"
    chmod +x "$INSTALL_DIR/wia-standards/quality-control/cli"/*.sh
    echo -e "${GREEN}${CHECK_MARK} CLI tools installed${NC}"
fi

# Copy README
if [ -f "$SCRIPT_DIR/README.md" ]; then
    cp "$SCRIPT_DIR/README.md" "$INSTALL_DIR/wia-standards/quality-control/"
    echo -e "${GREEN}${CHECK_MARK} Documentation installed${NC}"
fi

echo ""

# Install TypeScript SDK
echo -e "${YELLOW}${PACKAGE} Installing TypeScript SDK...${NC}"
if command -v npm &> /dev/null; then
    if [ -d "$SCRIPT_DIR/api/typescript" ]; then
        cd "$SCRIPT_DIR/api/typescript"
        echo -e "${CYAN}  Running npm install...${NC}"
        npm install --silent
        echo -e "${CYAN}  Building TypeScript SDK...${NC}"
        npm run build --silent 2>/dev/null || true
        echo -e "${GREEN}${CHECK_MARK} TypeScript SDK installed${NC}"
        cd - > /dev/null
    fi
else
    echo -e "${YELLOW}⚠️  npm not found. Skipping TypeScript SDK installation.${NC}"
    echo -e "${CYAN}  To install later: cd $INSTALL_DIR/wia-standards/quality-control/api/typescript && npm install${NC}"
fi
echo ""

# Install CLI symlink
echo -e "${YELLOW}${WRENCH} Installing CLI tool...${NC}"
if [ -f "$INSTALL_DIR/wia-standards/quality-control/cli/wia-ind-025.sh" ]; then
    ln -sf "$INSTALL_DIR/wia-standards/quality-control/cli/wia-ind-025.sh" "$BIN_DIR/wia-ind-025"
    chmod +x "$BIN_DIR/wia-ind-025"
    echo -e "${GREEN}${CHECK_MARK} CLI tool linked to: $BIN_DIR/wia-ind-025${NC}"
else
    echo -e "${RED}${CROSS_MARK} CLI tool not found${NC}"
fi
echo ""

# Create quality control templates
echo -e "${YELLOW}${DOCS} Creating quality control templates...${NC}"

# Inspection checklist template
cat > "$HOME/.wia/quality-control/templates/inspection-checklist.json" << 'EOF'
{
  "templateId": "inspection-checklist-v1",
  "templateName": "Generic Inspection Checklist",
  "version": "1.0",
  "inspectionType": "final",
  "checkpoints": [
    {
      "id": "CP-001",
      "description": "Visual inspection for surface defects",
      "type": "visual",
      "acceptanceCriteria": "No scratches, dents, or discoloration"
    },
    {
      "id": "CP-002",
      "description": "Dimensional verification",
      "type": "measurement",
      "characteristics": ["length", "width", "height", "diameter"],
      "acceptanceCriteria": "Within specified tolerances"
    },
    {
      "id": "CP-003",
      "description": "Functional testing",
      "type": "functional",
      "acceptanceCriteria": "Operates per specification"
    },
    {
      "id": "CP-004",
      "description": "Packaging verification",
      "type": "visual",
      "acceptanceCriteria": "Proper packaging, labels, and documentation"
    }
  ]
}
EOF

# NCR template
cat > "$HOME/.wia/quality-control/templates/ncr-template.json" << 'EOF'
{
  "templateId": "ncr-v1",
  "templateName": "Non-Conformance Report",
  "version": "1.0",
  "fields": {
    "ncrNumber": "NCR-YYYY-NNNNNN",
    "dateReported": "ISO-8601",
    "reportedBy": "Employee ID",
    "productSku": "Product identifier",
    "batchNumber": "Batch/Lot number",
    "quantityAffected": "Number of units",
    "issueDescription": "Detailed description of non-conformance",
    "detectionPoint": "receiving|in-process|final|customer",
    "severity": "critical|major|minor|cosmetic",
    "disposition": "scrap|rework|use-as-is|return-to-supplier|repair",
    "rootCause": "Root cause analysis",
    "containmentAction": "Immediate containment",
    "correctiveAction": "Long-term correction",
    "capaRequired": true,
    "customerImpact": "Impact assessment",
    "closedBy": "Employee ID",
    "closedDate": "ISO-8601"
  }
}
EOF

# CAPA template
cat > "$HOME/.wia/quality-control/templates/capa-template.json" << 'EOF'
{
  "templateId": "capa-v1",
  "templateName": "Corrective and Preventive Action",
  "version": "1.0",
  "fields": {
    "capaNumber": "CAPA-YYYY-NNNNNN",
    "type": "corrective|preventive",
    "dateInitiated": "ISO-8601",
    "initiator": "Employee ID",
    "linkedNCR": "NCR number if applicable",
    "problemStatement": "Clear problem description",
    "rootCauseAnalysis": {
      "method": "5-why|fishbone|fault-tree",
      "findings": "Analysis results",
      "rootCause": "Identified root cause"
    },
    "correctiveAction": {
      "description": "Action to be taken",
      "assignedTo": "Employee ID",
      "dueDate": "ISO-8601",
      "status": "planned|in-progress|completed"
    },
    "preventiveAction": {
      "description": "Prevention measures",
      "assignedTo": "Employee ID",
      "dueDate": "ISO-8601",
      "status": "planned|in-progress|completed"
    },
    "verification": {
      "method": "How effectiveness will be verified",
      "responsible": "Employee ID",
      "verificationDate": "ISO-8601",
      "effectivenessConfirmed": true
    }
  }
}
EOF

# Control plan template
cat > "$HOME/.wia/quality-control/templates/control-plan.json" << 'EOF'
{
  "templateId": "control-plan-v1",
  "templateName": "Process Control Plan",
  "version": "1.0",
  "productFamily": "Product line identifier",
  "processSteps": [
    {
      "stepNumber": 1,
      "processName": "Process name",
      "machine": "Equipment identifier",
      "characteristics": [
        {
          "name": "Characteristic name",
          "specification": "Target ± tolerance",
          "measurementMethod": "How to measure",
          "sampleSize": 5,
          "frequency": "Every hour",
          "controlMethod": "X-bar & R chart",
          "reactionPlan": "What to do if out of spec"
        }
      ]
    }
  ]
}
EOF

echo -e "${GREEN}${CHECK_MARK} Quality control templates created${NC}"
echo ""

# Create SPC calculator helper
echo -e "${YELLOW}${CHART} Creating SPC calculators...${NC}"

cat > "$HOME/.wia/quality-control/spc-charts/calculate-cpk.sh" << 'EOF'
#!/bin/bash
# SPC Calculator: Process Capability (Cp, Cpk)

if [ $# -lt 4 ]; then
    echo "Usage: $0 <mean> <stddev> <lsl> <usl>"
    echo "Example: $0 50.0 0.01 49.95 50.05"
    exit 1
fi

MEAN=$1
STDDEV=$2
LSL=$3
USL=$4

# Calculate Cp
CP=$(echo "scale=3; ($USL - $LSL) / (6 * $STDDEV)" | bc)

# Calculate Cpk
CPU=$(echo "scale=3; ($USL - $MEAN) / (3 * $STDDEV)" | bc)
CPL=$(echo "scale=3; ($MEAN - $LSL) / (3 * $STDDEV)" | bc)

if (( $(echo "$CPU < $CPL" | bc -l) )); then
    CPK=$CPU
else
    CPK=$CPL
fi

# Calculate Sigma Level
SIGMA=$(echo "scale=2; $CPK * 3" | bc)

# Calculate DPMO (approximation)
DPMO=$(echo "scale=0; 1000000 * e(-($SIGMA * $SIGMA / 2)) / 2.5066" | bc -l)

echo "Process Capability Analysis"
echo "============================"
echo "Mean (μ):     $MEAN"
echo "Std Dev (σ):  $STDDEV"
echo "LSL:          $LSL"
echo "USL:          $USL"
echo ""
echo "Cp:           $CP"
echo "Cpk:          $CPK"
echo "Sigma Level:  $SIGMA"
echo "DPMO:         ~$DPMO"
echo ""

if (( $(echo "$CPK >= 1.67" | bc -l) )); then
    echo "Status: ✅ Excellent (Cpk ≥ 1.67)"
elif (( $(echo "$CPK >= 1.33" | bc -l) )); then
    echo "Status: ✅ Capable (Cpk ≥ 1.33)"
elif (( $(echo "$CPK >= 1.00" | bc -l) )); then
    echo "Status: ⚠️  Marginally Capable (Cpk ≥ 1.00)"
else
    echo "Status: ❌ Not Capable (Cpk < 1.00)"
fi
EOF

chmod +x "$HOME/.wia/quality-control/spc-charts/calculate-cpk.sh"
echo -e "${GREEN}${CHECK_MARK} SPC calculators created${NC}"
echo ""

# Create certification framework
echo -e "${YELLOW}${GEAR} Creating certification frameworks...${NC}"

cat > "$HOME/.wia/quality-control/certificates/iso9001-checklist.md" << 'EOF'
# ISO 9001:2015 Compliance Checklist

## 4. Context of the Organization
- [ ] 4.1 Understanding the organization and its context
- [ ] 4.2 Understanding the needs and expectations of interested parties
- [ ] 4.3 Determining the scope of the QMS
- [ ] 4.4 Quality management system and its processes

## 5. Leadership
- [ ] 5.1 Leadership and commitment
- [ ] 5.2 Quality policy
- [ ] 5.3 Organizational roles, responsibilities and authorities

## 6. Planning
- [ ] 6.1 Actions to address risks and opportunities
- [ ] 6.2 Quality objectives and planning to achieve them
- [ ] 6.3 Planning of changes

## 7. Support
- [ ] 7.1 Resources
- [ ] 7.2 Competence
- [ ] 7.3 Awareness
- [ ] 7.4 Communication
- [ ] 7.5 Documented information

## 8. Operation
- [ ] 8.1 Operational planning and control
- [ ] 8.2 Requirements for products and services
- [ ] 8.3 Design and development
- [ ] 8.4 Control of externally provided processes
- [ ] 8.5 Production and service provision
- [ ] 8.6 Release of products and services
- [ ] 8.7 Control of nonconforming outputs

## 9. Performance Evaluation
- [ ] 9.1 Monitoring, measurement, analysis and evaluation
- [ ] 9.2 Internal audit
- [ ] 9.3 Management review

## 10. Improvement
- [ ] 10.1 General
- [ ] 10.2 Nonconformity and corrective action
- [ ] 10.3 Continual improvement
EOF

echo -e "${GREEN}${CHECK_MARK} Certification frameworks created${NC}"
echo ""

# Add to PATH if not already there
echo -e "${YELLOW}${WRENCH} Configuring PATH...${NC}"
SHELL_RC=""
if [ -n "$BASH_VERSION" ]; then
    SHELL_RC="$HOME/.bashrc"
elif [ -n "$ZSH_VERSION" ]; then
    SHELL_RC="$HOME/.zshrc"
fi

if [ -n "$SHELL_RC" ] && [ -f "$SHELL_RC" ]; then
    if ! grep -q "$BIN_DIR" "$SHELL_RC"; then
        echo "" >> "$SHELL_RC"
        echo "# WIA Quality Control Standard" >> "$SHELL_RC"
        echo "export PATH=\"\$PATH:$BIN_DIR\"" >> "$SHELL_RC"
        echo -e "${GREEN}${CHECK_MARK} Added $BIN_DIR to PATH in $SHELL_RC${NC}"
        echo -e "${YELLOW}⚠️  Please run: source $SHELL_RC${NC}"
    else
        echo -e "${GREEN}${CHECK_MARK} PATH already configured${NC}"
    fi
fi
echo ""

# Create environment example
echo -e "${YELLOW}${GEAR} Creating environment configuration...${NC}"
cat > "$HOME/.wia/quality-control/.env.example" << 'EOF'
# WIA-IND-025 Quality Control Configuration

# API Configuration
WIA_API_KEY=your-api-key-here
WIA_API_ENDPOINT=https://api.wiastandards.com/v1

# Quality Management System
QMS_CERTIFICATION_LEVEL=ISO-9001
QMS_INDUSTRY=automotive
QMS_FACILITY_ID=FACILITY-001

# SPC Configuration
SPC_CONTROL_LIMIT_SIGMA=3
SPC_WARNING_LIMIT_SIGMA=2
SPC_MIN_CPK=1.33
SPC_MIN_SIGMA_LEVEL=4.0

# Calibration Settings
CALIBRATION_CRITICAL_INTERVAL_DAYS=180
CALIBRATION_NONCRITICAL_INTERVAL_DAYS=365
CALIBRATION_WARNING_DAYS=30

# NCR/CAPA Settings
NCR_AUTO_CAPA_THRESHOLD=major
CAPA_DEFAULT_DUE_DAYS=30

# Defect Detection AI
AI_DEFECT_MODEL=yolov8-defect-detection
AI_CONFIDENCE_THRESHOLD=0.85

# Audit Settings
AUDIT_INTERNAL_FREQUENCY_DAYS=180
AUDIT_EXTERNAL_FREQUENCY_DAYS=365

# Document Control
DOC_REVIEW_CYCLE_DAYS=365
DOC_APPROVAL_REQUIRED=true

# Database (optional)
DATABASE_TYPE=postgresql
DATABASE_HOST=localhost
DATABASE_PORT=5432
DATABASE_NAME=quality_control
DATABASE_USER=qc_user
DATABASE_PASSWORD=secure-password

# Notifications
NOTIFICATION_EMAIL=quality@company.com
NOTIFICATION_SLACK_WEBHOOK=https://hooks.slack.com/services/YOUR/WEBHOOK/URL
EOF

echo -e "${GREEN}${CHECK_MARK} Environment configuration created${NC}"
echo ""

# Installation summary
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                                                            ║${NC}"
echo -e "${GREEN}║  ${ROCKET} Installation Complete!                                 ║${NC}"
echo -e "${GREEN}║                                                            ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}${CHECK_MARK} Installed Components:${NC}"
echo -e "  • Quality Control specification"
echo -e "  • TypeScript SDK (@wia/ind-025)"
echo -e "  • CLI tool (wia-ind-025)"
echo -e "  • Quality templates (inspection, NCR, CAPA, control plan)"
echo -e "  • SPC calculators"
echo -e "  • ISO 9001 certification framework"
echo ""
echo -e "${CYAN}${ROCKET} Quick Start:${NC}"
echo ""
echo -e "  ${YELLOW}1.${NC} Verify installation:"
echo -e "     ${BLUE}wia-ind-025 --version${NC}"
echo ""
echo -e "  ${YELLOW}2.${NC} Create quality inspection:"
echo -e "     ${BLUE}wia-ind-025 inspect --product WIDGET-A100 --batch BATCH-001${NC}"
echo ""
echo -e "  ${YELLOW}3.${NC} Calculate SPC metrics:"
echo -e "     ${BLUE}wia-ind-025 spc --characteristic diameter --nominal 50.0${NC}"
echo ""
echo -e "  ${YELLOW}4.${NC} Check calibration status:"
echo -e "     ${BLUE}wia-ind-025 calibration-status --equipment all${NC}"
echo ""
echo -e "  ${YELLOW}5.${NC} View templates:"
echo -e "     ${BLUE}ls $HOME/.wia/quality-control/templates/${NC}"
echo ""
echo -e "${CYAN}${DOCS} Documentation:${NC}"
echo -e "  • README: $INSTALL_DIR/wia-standards/quality-control/README.md"
echo -e "  • Spec: $INSTALL_DIR/wia-standards/quality-control/spec/WIA-IND-025-v1.0.md"
echo -e "  • Templates: $HOME/.wia/quality-control/templates/"
echo -e "  • Online: https://docs.wiastandards.com/ind-025"
echo ""
echo -e "${CYAN}${PACKAGE} TypeScript SDK:${NC}"
echo -e "  ${BLUE}npm install @wia/ind-025${NC}"
echo ""
echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${PURPLE}弘益人間 (홍익인간) · Benefit All Humanity${NC}"
echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${CYAN}WIA - World Certification Industry Association${NC}"
echo -e "${CYAN}© 2025 SmileStory Inc. / WIA${NC}"
echo -e "${CYAN}MIT License${NC}"
echo ""

# Final PATH reminder
if [ -n "$SHELL_RC" ]; then
    echo -e "${YELLOW}⚠️  Don't forget to reload your shell:${NC}"
    echo -e "   ${BLUE}source $SHELL_RC${NC}"
    echo -e "   ${GRAY}or restart your terminal${NC}"
    echo ""
fi

exit 0

################################################################################
# 弘益人間 (홍익인간) · Benefit All Humanity
################################################################################
