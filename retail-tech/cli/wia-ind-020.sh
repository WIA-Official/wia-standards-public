#!/bin/bash

################################################################################
# WIA-IND-020: Retail Tech CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This CLI tool provides command-line access to retail operations including:
# - Transaction management
# - Payment processing
# - Inventory management
# - Loyalty program operations
# - Discount application
# - Return processing
# - Sales reporting
################################################################################

set -e

# Colors for output
AMBER='\033[0;93m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
CONFIG_DIR="$HOME/.wia/ind-020"
CONFIG_FILE="$CONFIG_DIR/config.json"

# Helper functions
print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║              🛒 WIA-IND-020: Retail Tech CLI                  ║"
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

print_value() {
    echo -e "${BLUE}  $1:${RESET} ${AMBER}$2${RESET}"
}

# Generate unique ID
generate_id() {
    local prefix=$1
    local timestamp=$(date +%s)
    local random=$(openssl rand -hex 4 2>/dev/null || echo $RANDOM)
    echo "${prefix}-${timestamp}-${random}" | tr '[:lower:]' '[:upper:]'
}

# Generate receipt number
generate_receipt_number() {
    local date=$(date +%y%m%d)
    local seq=$(printf "%04d" $((RANDOM % 10000)))
    echo "${date}-${seq}"
}

# Format currency
format_currency() {
    local amount=$1
    printf "$%.2f" "$amount"
}

# Calculate percentage
calculate_percentage() {
    local value=$1
    local total=$2
    if command -v bc &> /dev/null; then
        echo "scale=2; ($value / $total) * 100" | bc
    else
        echo "0"
    fi
}

# Load configuration
load_config() {
    if [ ! -f "$CONFIG_FILE" ]; then
        mkdir -p "$CONFIG_DIR"
        cat > "$CONFIG_FILE" <<EOF
{
  "storeId": "STORE-001",
  "currency": "USD",
  "taxRate": 0.08,
  "loyaltyPointsPerDollar": 1,
  "environment": "production"
}
EOF
        print_info "Created default configuration at $CONFIG_FILE"
    fi
}

# Transaction commands
cmd_transaction_create() {
    print_section "Create Transaction"

    local store_id="${STORE_ID:-STORE-001}"
    local register_id="${REGISTER_ID:-POS-01}"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --store) store_id="$2"; shift 2 ;;
            --register) register_id="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    local txn_id=$(generate_id "TXN")
    local receipt_num=$(generate_receipt_number)

    print_success "Transaction created"
    print_value "Transaction ID" "$txn_id"
    print_value "Store ID" "$store_id"
    print_value "Register ID" "$register_id"
    print_value "Receipt Number" "$receipt_num"
    print_value "Status" "PENDING"
    print_value "Created" "$(date '+%Y-%m-%d %H:%M:%S')"

    echo ""
    print_info "Use 'wia-ind-020 transaction add-item --txn $txn_id' to add items"
}

cmd_transaction_add_item() {
    print_section "Add Item to Transaction"

    local txn_id=""
    local sku=""
    local quantity=1
    local price=0

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --txn) txn_id="$2"; shift 2 ;;
            --sku) sku="$2"; shift 2 ;;
            --quantity) quantity="$2"; shift 2 ;;
            --price) price="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$txn_id" ] || [ -z "$sku" ]; then
        print_error "Transaction ID and SKU are required"
        echo "Usage: wia-ind-020 transaction add-item --txn TXN-ID --sku SKU-123 --quantity 1 --price 99.99"
        exit 1
    fi

    local item_total=$(echo "$price * $quantity" | bc -l)
    local tax_amount=$(echo "$item_total * 0.08" | bc -l)
    local line_total=$(echo "$item_total + $tax_amount" | bc -l)

    print_success "Item added to transaction"
    print_value "Transaction ID" "$txn_id"
    print_value "SKU" "$sku"
    print_value "Quantity" "$quantity"
    print_value "Price" "$(format_currency $price)"
    print_value "Subtotal" "$(format_currency $item_total)"
    print_value "Tax" "$(format_currency $tax_amount)"
    print_value "Total" "$(format_currency $line_total)"
}

# Payment commands
cmd_payment_process() {
    print_section "Process Payment"

    local txn_id=""
    local method="credit_card"
    local amount=0

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --txn) txn_id="$2"; shift 2 ;;
            --method) method="$2"; shift 2 ;;
            --amount) amount="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$txn_id" ]; then
        print_error "Transaction ID is required"
        exit 1
    fi

    local payment_id=$(generate_id "PAY")
    local auth_code=$(openssl rand -hex 6 2>/dev/null | tr '[:lower:]' '[:upper:]' || echo "AUTH$(date +%s)")

    print_success "Payment processed successfully"
    print_value "Payment ID" "$payment_id"
    print_value "Transaction ID" "$txn_id"
    print_value "Method" "$method"
    print_value "Amount" "$(format_currency $amount)"
    print_value "Status" "CAPTURED"
    print_value "Authorization Code" "$auth_code"
    print_value "Processor" "WIA-Payment-Gateway"
    print_value "Timestamp" "$(date '+%Y-%m-%d %H:%M:%S')"
}

# Inventory commands
cmd_inventory_check() {
    print_section "Check Inventory"

    local sku=""
    local store=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --sku) sku="$2"; shift 2 ;;
            --store) store="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$sku" ]; then
        print_error "SKU is required"
        exit 1
    fi

    # Simulate inventory check
    local available=$((RANDOM % 100 + 10))
    local reserved=$((RANDOM % 20))
    local in_transit=$((RANDOM % 30))
    local reorder_point=20

    print_success "Inventory found"
    print_value "SKU" "$sku"
    print_value "Location" "${store:-STORE-001}"
    print_value "Available" "$available units"
    print_value "Reserved" "$reserved units"
    print_value "In Transit" "$in_transit units"
    print_value "Reorder Point" "$reorder_point units"

    if [ $available -lt $reorder_point ]; then
        echo ""
        print_warning "Stock level below reorder point!"
        print_info "Recommended to reorder: $((reorder_point * 2)) units"
    fi
}

cmd_inventory_update() {
    print_section "Update Inventory"

    local sku=""
    local quantity=0
    local operation="add"
    local reason="adjustment"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --sku) sku="$2"; shift 2 ;;
            --quantity) quantity="$2"; shift 2 ;;
            --operation) operation="$2"; shift 2 ;;
            --reason) reason="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$sku" ]; then
        print_error "SKU is required"
        exit 1
    fi

    local movement_id=$(generate_id "STK")

    print_success "Inventory updated"
    print_value "Movement ID" "$movement_id"
    print_value "SKU" "$sku"
    print_value "Operation" "$operation"
    print_value "Quantity" "$quantity"
    print_value "Reason" "$reason"
    print_value "Timestamp" "$(date '+%Y-%m-%d %H:%M:%S')"
}

# Loyalty commands
cmd_loyalty_add_points() {
    print_section "Add Loyalty Points"

    local customer=""
    local points=0
    local reference=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --customer) customer="$2"; shift 2 ;;
            --points) points="$2"; shift 2 ;;
            --reference) reference="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$customer" ]; then
        print_error "Customer ID is required"
        exit 1
    fi

    local loyalty_id=$(generate_id "LOY")
    local new_balance=$((RANDOM % 5000 + points))

    print_success "Loyalty points added"
    print_value "Transaction ID" "$loyalty_id"
    print_value "Customer ID" "$customer"
    print_value "Points Added" "+$points"
    print_value "New Balance" "$new_balance points"
    if [ -n "$reference" ]; then
        print_value "Reference" "$reference"
    fi
    print_value "Timestamp" "$(date '+%Y-%m-%d %H:%M:%S')"
}

cmd_loyalty_redeem_points() {
    print_section "Redeem Loyalty Points"

    local customer=""
    local points=0

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --customer) customer="$2"; shift 2 ;;
            --points) points="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$customer" ]; then
        print_error "Customer ID is required"
        exit 1
    fi

    local discount=$(echo "scale=2; $points * 0.01" | bc -l)
    local loyalty_id=$(generate_id "LOY")

    print_success "Loyalty points redeemed"
    print_value "Transaction ID" "$loyalty_id"
    print_value "Customer ID" "$customer"
    print_value "Points Redeemed" "-$points"
    print_value "Discount Value" "$(format_currency $discount)"
    print_value "Timestamp" "$(date '+%Y-%m-%d %H:%M:%S')"
}

# Discount commands
cmd_discount_apply() {
    print_section "Apply Discount"

    local code=""
    local type="percentage"
    local value=0
    local subtotal=100

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --code) code="$2"; shift 2 ;;
            --type) type="$2"; shift 2 ;;
            --value) value="$2"; shift 2 ;;
            --subtotal) subtotal="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    local discount_amount=0

    case $type in
        percentage)
            discount_amount=$(echo "scale=2; $subtotal * ($value / 100)" | bc -l)
            ;;
        fixed_amount)
            discount_amount=$value
            ;;
    esac

    local new_total=$(echo "$subtotal - $discount_amount" | bc -l)

    print_success "Discount applied"
    if [ -n "$code" ]; then
        print_value "Discount Code" "$code"
    fi
    print_value "Type" "$type"
    print_value "Value" "$value"
    print_value "Original Subtotal" "$(format_currency $subtotal)"
    print_value "Discount Amount" "-$(format_currency $discount_amount)"
    print_value "New Subtotal" "$(format_currency $new_total)"
}

# Return commands
cmd_return_process() {
    print_section "Process Return"

    local txn=""
    local reason="changed_mind"
    local sku=""
    local quantity=1

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --transaction) txn="$2"; shift 2 ;;
            --reason) reason="$2"; shift 2 ;;
            --sku) sku="$2"; shift 2 ;;
            --quantity) quantity="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$txn" ]; then
        print_error "Transaction ID is required"
        exit 1
    fi

    local return_id=$(generate_id "RET")
    local refund_amount=$(echo "scale=2; 99.99 * $quantity" | bc -l)

    print_success "Return processed"
    print_value "Return ID" "$return_id"
    print_value "Original Transaction" "$txn"
    if [ -n "$sku" ]; then
        print_value "SKU" "$sku"
        print_value "Quantity" "$quantity"
    fi
    print_value "Reason" "$reason"
    print_value "Refund Amount" "$(format_currency $refund_amount)"
    print_value "Status" "APPROVED"
    print_value "Timestamp" "$(date '+%Y-%m-%d %H:%M:%S')"
}

# Report commands
cmd_report_sales() {
    print_section "Sales Report"

    local start_date=""
    local end_date=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --start-date) start_date="$2"; shift 2 ;;
            --end-date) end_date="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    # Simulate sales data
    local total_sales=$(echo "scale=2; $RANDOM * 10" | bc -l)
    local total_txn=$((RANDOM % 500 + 100))
    local avg_txn=$(echo "scale=2; $total_sales / $total_txn" | bc -l)
    local total_items=$((total_txn * 2))
    local total_tax=$(echo "scale=2; $total_sales * 0.08" | bc -l)
    local total_discount=$(echo "scale=2; $total_sales * 0.05" | bc -l)

    print_success "Sales report generated"
    echo ""
    print_value "Period" "${start_date:-2025-01-01} to ${end_date:-2025-01-31}"
    echo ""
    print_value "Total Sales" "$(format_currency $total_sales)"
    print_value "Total Transactions" "$total_txn"
    print_value "Average Transaction Value" "$(format_currency $avg_txn)"
    print_value "Total Items Sold" "$total_items"
    print_value "Total Tax Collected" "$(format_currency $total_tax)"
    print_value "Total Discounts" "$(format_currency $total_discount)"

    echo ""
    print_info "Sales by Channel:"
    print_value "  In-Store" "65%"
    print_value "  Online" "25%"
    print_value "  Mobile App" "10%"
}

cmd_report_inventory() {
    print_section "Inventory Report"

    local total_skus=$((RANDOM % 1000 + 500))
    local total_units=$((RANDOM % 50000 + 10000))
    local inventory_value=$(echo "scale=2; $RANDOM * 1000" | bc -l)
    local out_of_stock=$((RANDOM % 50))
    local low_stock=$((RANDOM % 100))

    print_success "Inventory report generated"
    echo ""
    print_value "Report Date" "$(date '+%Y-%m-%d')"
    echo ""
    print_value "Total SKUs" "$total_skus"
    print_value "Total Units" "$total_units"
    print_value "Total Inventory Value" "$(format_currency $inventory_value)"
    print_value "Out of Stock Items" "$out_of_stock"
    print_value "Low Stock Items" "$low_stock"
    print_value "Inventory Turnover Ratio" "4.5"
    print_value "Days Inventory Outstanding" "81"

    if [ $out_of_stock -gt 0 ]; then
        echo ""
        print_warning "$out_of_stock items are out of stock!"
    fi
}

# Version command
cmd_version() {
    echo "WIA-IND-020 Retail Tech CLI v$VERSION"
    echo "弘益人間 · Benefit All Humanity"
}

# Help command
cmd_help() {
    print_header

    echo "Usage: wia-ind-020 <command> [options]"
    echo ""
    echo -e "${CYAN}Transaction Commands:${RESET}"
    echo "  transaction create          Create new transaction"
    echo "    --store STORE-ID          Store ID"
    echo "    --register POS-ID         Register ID"
    echo ""
    echo "  transaction add-item        Add item to transaction"
    echo "    --txn TXN-ID              Transaction ID"
    echo "    --sku SKU-123             Product SKU"
    echo "    --quantity N              Quantity"
    echo "    --price AMOUNT            Price per unit"
    echo ""
    echo -e "${CYAN}Payment Commands:${RESET}"
    echo "  payment process             Process payment"
    echo "    --txn TXN-ID              Transaction ID"
    echo "    --method METHOD           Payment method"
    echo "    --amount AMOUNT           Payment amount"
    echo ""
    echo -e "${CYAN}Inventory Commands:${RESET}"
    echo "  inventory check             Check inventory levels"
    echo "    --sku SKU-123             Product SKU"
    echo "    --store STORE-ID          Store ID"
    echo ""
    echo "  inventory update            Update inventory"
    echo "    --sku SKU-123             Product SKU"
    echo "    --quantity N              Quantity change"
    echo "    --operation add|subtract  Operation type"
    echo "    --reason REASON           Reason for change"
    echo ""
    echo -e "${CYAN}Loyalty Commands:${RESET}"
    echo "  loyalty add-points          Add loyalty points"
    echo "    --customer CUST-ID        Customer ID"
    echo "    --points N                Points to add"
    echo "    --reference REF           Reference ID"
    echo ""
    echo "  loyalty redeem-points       Redeem loyalty points"
    echo "    --customer CUST-ID        Customer ID"
    echo "    --points N                Points to redeem"
    echo ""
    echo -e "${CYAN}Discount Commands:${RESET}"
    echo "  discount apply              Apply discount"
    echo "    --code CODE               Discount code"
    echo "    --type TYPE               percentage|fixed_amount"
    echo "    --value VALUE             Discount value"
    echo "    --subtotal AMOUNT         Subtotal amount"
    echo ""
    echo -e "${CYAN}Return Commands:${RESET}"
    echo "  return process              Process return"
    echo "    --transaction TXN-ID      Original transaction ID"
    echo "    --reason REASON           Return reason"
    echo "    --sku SKU-123             Product SKU (optional)"
    echo "    --quantity N              Quantity to return"
    echo ""
    echo -e "${CYAN}Report Commands:${RESET}"
    echo "  report sales                Generate sales report"
    echo "    --start-date DATE         Start date (YYYY-MM-DD)"
    echo "    --end-date DATE           End date (YYYY-MM-DD)"
    echo ""
    echo "  report inventory            Generate inventory report"
    echo ""
    echo -e "${CYAN}General Commands:${RESET}"
    echo "  version                     Show version"
    echo "  help                        Show this help"
    echo ""
    echo -e "${GRAY}Examples:${RESET}"
    echo "  wia-ind-020 transaction create --store STORE-001 --register POS-03"
    echo "  wia-ind-020 inventory check --sku PROD-12345"
    echo "  wia-ind-020 loyalty add-points --customer CUST-9876 --points 100"
    echo "  wia-ind-020 report sales --start-date 2025-01-01 --end-date 2025-01-31"
    echo ""
    echo -e "${AMBER}弘익人間 (Benefit All Humanity)${RESET}"
}

# Main command dispatcher
main() {
    load_config

    if [ $# -eq 0 ]; then
        cmd_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        transaction)
            if [ $# -eq 0 ]; then
                print_error "Transaction command requires a subcommand"
                echo "Use: wia-ind-020 transaction <create|add-item>"
                exit 1
            fi
            local subcommand=$1
            shift
            case $subcommand in
                create) cmd_transaction_create "$@" ;;
                add-item) cmd_transaction_add_item "$@" ;;
                *) print_error "Unknown transaction subcommand: $subcommand"; exit 1 ;;
            esac
            ;;
        payment)
            if [ $# -eq 0 ]; then
                print_error "Payment command requires a subcommand"
                exit 1
            fi
            local subcommand=$1
            shift
            case $subcommand in
                process) cmd_payment_process "$@" ;;
                *) print_error "Unknown payment subcommand: $subcommand"; exit 1 ;;
            esac
            ;;
        inventory)
            if [ $# -eq 0 ]; then
                print_error "Inventory command requires a subcommand"
                exit 1
            fi
            local subcommand=$1
            shift
            case $subcommand in
                check) cmd_inventory_check "$@" ;;
                update) cmd_inventory_update "$@" ;;
                *) print_error "Unknown inventory subcommand: $subcommand"; exit 1 ;;
            esac
            ;;
        loyalty)
            if [ $# -eq 0 ]; then
                print_error "Loyalty command requires a subcommand"
                exit 1
            fi
            local subcommand=$1
            shift
            case $subcommand in
                add-points) cmd_loyalty_add_points "$@" ;;
                redeem-points) cmd_loyalty_redeem_points "$@" ;;
                *) print_error "Unknown loyalty subcommand: $subcommand"; exit 1 ;;
            esac
            ;;
        discount)
            if [ $# -eq 0 ]; then
                print_error "Discount command requires a subcommand"
                exit 1
            fi
            local subcommand=$1
            shift
            case $subcommand in
                apply) cmd_discount_apply "$@" ;;
                *) print_error "Unknown discount subcommand: $subcommand"; exit 1 ;;
            esac
            ;;
        return)
            if [ $# -eq 0 ]; then
                print_error "Return command requires a subcommand"
                exit 1
            fi
            local subcommand=$1
            shift
            case $subcommand in
                process) cmd_return_process "$@" ;;
                *) print_error "Unknown return subcommand: $subcommand"; exit 1 ;;
            esac
            ;;
        report)
            if [ $# -eq 0 ]; then
                print_error "Report command requires a subcommand"
                exit 1
            fi
            local subcommand=$1
            shift
            case $subcommand in
                sales) cmd_report_sales "$@" ;;
                inventory) cmd_report_inventory "$@" ;;
                *) print_error "Unknown report subcommand: $subcommand"; exit 1 ;;
            esac
            ;;
        version) cmd_version ;;
        help|--help|-h) cmd_help ;;
        *)
            print_error "Unknown command: $command"
            echo "Use 'wia-ind-020 help' for usage information"
            exit 1
            ;;
    esac
}

# Run main
main "$@"

# 弘益인간) · Benefit All Humanity
