#!/bin/bash
# WIA-FINANCIAL_INCLUSION CLI Tool
# Version: 1.0.0
# Philosophy: 弘益人間 (Benefit All Humanity)
# Description: Command-line interface for financial inclusion operations

set -e

VERSION="1.0.0"
CONFIG_FILE="$HOME/.wia-financial-inclusion/config.json"
DATA_DIR="$HOME/.wia-financial-inclusion/data"
API_BASE_URL="${WIA_API_URL:-https://api.wia.org/v1/financial-inclusion}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Initialize data directory
init_dirs() {
    mkdir -p "$DATA_DIR"
    mkdir -p "$(dirname "$CONFIG_FILE")"
    if [ ! -f "$CONFIG_FILE" ]; then
        echo '{"api_key":"","user_id":"","default_currency":"USD"}' > "$CONFIG_FILE"
    fi
}

# Print colored output
print_success() { echo -e "${GREEN}✓${NC} $1"; }
print_error() { echo -e "${RED}✗${NC} $1" >&2; }
print_info() { echo -e "${BLUE}ℹ${NC} $1"; }
print_warning() { echo -e "${YELLOW}⚠${NC} $1"; }

# Generate UUID
generate_uuid() {
    cat /proc/sys/kernel/random/uuid 2>/dev/null || uuidgen 2>/dev/null || echo "$(date +%s)-$(($RANDOM * $RANDOM))"
}

# Get current timestamp
get_timestamp() {
    date -u +"%Y-%m-%dT%H:%M:%SZ"
}

# Read config value
get_config() {
    local key="$1"
    grep -o "\"$key\":\"[^\"]*\"" "$CONFIG_FILE" | cut -d'"' -f4
}

# Set config value
set_config() {
    local key="$1"
    local value="$2"
    local temp_file=$(mktemp)
    jq ".$key = \"$value\"" "$CONFIG_FILE" > "$temp_file" 2>/dev/null && mv "$temp_file" "$CONFIG_FILE" || {
        # Fallback if jq not available
        sed -i "s/\"$key\":\"[^\"]*\"/\"$key\":\"$value\"/" "$CONFIG_FILE"
    }
    print_success "Configuration updated: $key"
}

# Create new account
cmd_account_create() {
    local full_name="$1"
    local phone="$2"
    local email="${3:-}"
    local account_type="${4:-basic}"

    if [ -z "$full_name" ] || [ -z "$phone" ]; then
        print_error "Usage: $0 account-create <full_name> <phone> [email] [account_type]"
        return 1
    fi

    local account_id=$(generate_uuid)
    local timestamp=$(get_timestamp)

    local account_file="$DATA_DIR/account-$account_id.json"

    cat > "$account_file" <<EOF
{
  "account_id": "$account_id",
  "type": "digital_banking_account",
  "created_at": "$timestamp",
  "status": "pending_verification",
  "holder": {
    "full_name": "$full_name",
    "phone": "$phone",
    "email": "$email"
  },
  "account_type": "$account_type",
  "balance": 0.00,
  "currency": "USD",
  "kyc_status": "pending",
  "credit_score": null,
  "access_level": "unbanked_to_banked"
}
EOF

    print_success "Account created successfully!"
    print_info "Account ID: $account_id"
    print_info "Status: pending_verification"
    print_info "Next step: Run KYC verification using: $0 kyc-verify $account_id"
    echo "$account_id"
}

# Verify KYC
cmd_kyc_verify() {
    local account_id="$1"
    local id_type="${2:-national_id}"
    local id_number="${3:-SAMPLE-$(date +%s)}"

    if [ -z "$account_id" ]; then
        print_error "Usage: $0 kyc-verify <account_id> [id_type] [id_number]"
        return 1
    fi

    local account_file="$DATA_DIR/account-$account_id.json"

    if [ ! -f "$account_file" ]; then
        print_error "Account not found: $account_id"
        return 1
    fi

    print_info "Performing KYC verification..."
    print_info "ID Type: $id_type"
    print_info "ID Number: $id_number"

    # Simulate KYC verification process
    sleep 1

    # Update account status
    local temp_file=$(mktemp)
    if command -v jq &> /dev/null; then
        jq '.kyc_status = "verified" | .status = "active" | .kyc_verified_at = "'$(get_timestamp)'"' "$account_file" > "$temp_file" && mv "$temp_file" "$account_file"
    else
        sed -i 's/"kyc_status": "pending"/"kyc_status": "verified"/' "$account_file"
        sed -i 's/"status": "pending_verification"/"status": "active"/' "$account_file"
    fi

    print_success "KYC verification completed!"
    print_success "Account is now ACTIVE and ready for transactions"
    print_info "You can now: check balance, send money, get credit score"
}

# Calculate/view credit score
cmd_credit_score() {
    local account_id="$1"
    local recalculate="${2:-false}"

    if [ -z "$account_id" ]; then
        print_error "Usage: $0 credit-score <account_id> [recalculate]"
        return 1
    fi

    local account_file="$DATA_DIR/account-$account_id.json"

    if [ ! -f "$account_file" ]; then
        print_error "Account not found: $account_id"
        return 1
    fi

    print_info "Calculating credit score using alternative data..."

    # Simulate credit scoring algorithm
    # Factors: mobile money history, utility payments, social connections, transaction patterns
    local base_score=300
    local mobile_history_score=$((RANDOM % 200))
    local payment_history_score=$((RANDOM % 200))
    local social_score=$((RANDOM % 150))

    local total_score=$((base_score + mobile_history_score + payment_history_score + social_score))

    print_info "Credit Score Components:"
    print_info "  Base Score: $base_score"
    print_info "  Mobile Money History: +$mobile_history_score"
    print_info "  Payment History: +$payment_history_score"
    print_info "  Social Trust Score: +$social_score"
    echo ""
    print_success "Total Credit Score: $total_score"

    local rating
    if [ $total_score -ge 700 ]; then
        rating="Excellent"
    elif [ $total_score -ge 600 ]; then
        rating="Good"
    elif [ $total_score -ge 500 ]; then
        rating="Fair"
    else
        rating="Building"
    fi

    print_info "Rating: $rating"
    echo "$total_score"
}

# Make transaction
cmd_transaction() {
    local account_id="$1"
    local amount="$2"
    local type="${3:-deposit}"
    local description="${4:-Transaction}"

    if [ -z "$account_id" ] || [ -z "$amount" ]; then
        print_error "Usage: $0 transaction <account_id> <amount> [type] [description]"
        print_info "Types: deposit, withdrawal, payment, transfer"
        return 1
    fi

    local account_file="$DATA_DIR/account-$account_id.json"

    if [ ! -f "$account_file" ]; then
        print_error "Account not found: $account_id"
        return 1
    fi

    local tx_id=$(generate_uuid)
    local timestamp=$(get_timestamp)

    local tx_file="$DATA_DIR/tx-$tx_id.json"

    cat > "$tx_file" <<EOF
{
  "transaction_id": "$tx_id",
  "account_id": "$account_id",
  "type": "$type",
  "amount": $amount,
  "currency": "USD",
  "description": "$description",
  "timestamp": "$timestamp",
  "status": "completed",
  "channel": "mobile_app"
}
EOF

    print_success "Transaction recorded!"
    print_info "Transaction ID: $tx_id"
    print_info "Type: $type"
    print_info "Amount: \$$amount USD"
    print_info "Status: completed"
}

# Check balance
cmd_balance() {
    local account_id="$1"

    if [ -z "$account_id" ]; then
        print_error "Usage: $0 balance <account_id>"
        return 1
    fi

    local account_file="$DATA_DIR/account-$account_id.json"

    if [ ! -f "$account_file" ]; then
        print_error "Account not found: $account_id"
        return 1
    fi

    print_info "Retrieving balance..."

    # Calculate balance from transactions
    local balance=0
    for tx in "$DATA_DIR"/tx-*.json; do
        if [ -f "$tx" ]; then
            if grep -q "\"account_id\": \"$account_id\"" "$tx"; then
                local amount=$(grep -o '"amount": [0-9.]*' "$tx" | cut -d' ' -f2)
                local type=$(grep -o '"type": "[^"]*"' "$tx" | cut -d'"' -f4)

                if [ "$type" = "deposit" ] || [ "$type" = "receive" ]; then
                    balance=$(echo "$balance + $amount" | bc 2>/dev/null || echo $((balance + ${amount%.*})))
                elif [ "$type" = "withdrawal" ] || [ "$type" = "send" ] || [ "$type" = "payment" ]; then
                    balance=$(echo "$balance - $amount" | bc 2>/dev/null || echo $((balance - ${amount%.*})))
                fi
            fi
        fi
    done

    print_success "Current Balance: \$$balance USD"
    echo "$balance"
}

# Send money
cmd_send_money() {
    local from_account="$1"
    local to_account="$2"
    local amount="$3"
    local note="${4:-P2P Transfer}"

    if [ -z "$from_account" ] || [ -z "$to_account" ] || [ -z "$amount" ]; then
        print_error "Usage: $0 send-money <from_account> <to_account> <amount> [note]"
        return 1
    fi

    print_info "Initiating money transfer..."
    print_info "From: $from_account"
    print_info "To: $to_account"
    print_info "Amount: \$$amount USD"

    # Record debit transaction
    cmd_transaction "$from_account" "$amount" "send" "Transfer to $to_account: $note" > /dev/null

    # Record credit transaction
    cmd_transaction "$to_account" "$amount" "receive" "Transfer from $from_account: $note" > /dev/null

    print_success "Transfer completed successfully!"
    print_info "Low-cost remittance processed"
}

# Configure CLI
cmd_config() {
    local action="$1"
    local key="$2"
    local value="$3"

    if [ "$action" = "set" ]; then
        if [ -z "$key" ] || [ -z "$value" ]; then
            print_error "Usage: $0 config set <key> <value>"
            return 1
        fi
        set_config "$key" "$value"
    elif [ "$action" = "get" ]; then
        if [ -z "$key" ]; then
            cat "$CONFIG_FILE"
        else
            get_config "$key"
        fi
    else
        print_error "Usage: $0 config {get|set} [key] [value]"
        return 1
    fi
}

# Show help
cmd_help() {
    cat <<EOF
WIA-FINANCIAL_INCLUSION CLI v$VERSION
弘益人間 (Benefit All Humanity)

USAGE:
    $(basename "$0") <command> [options]

COMMANDS:
    account-create <name> <phone> [email] [type]
        Create a new digital banking account
        Helps bring unbanked populations into financial system

    kyc-verify <account_id> [id_type] [id_number]
        Verify customer identity using simplified KYC
        Enables access to financial services

    credit-score <account_id> [recalculate]
        Calculate credit score using alternative data
        Uses mobile money, payments, social trust

    transaction <account_id> <amount> [type] [description]
        Record a financial transaction
        Types: deposit, withdrawal, payment, transfer

    balance <account_id>
        Check account balance

    send-money <from_account> <to_account> <amount> [note]
        Send money between accounts (P2P, remittance)
        Low-cost cross-border transfers

    config {get|set} [key] [value]
        Manage CLI configuration

    help
        Show this help message

EXAMPLES:
    # Create new account
    $(basename "$0") account-create "John Doe" "+1234567890" "john@example.com"

    # Verify KYC
    $(basename "$0") kyc-verify <account_id> national_id ID123456

    # Check credit score
    $(basename "$0") credit-score <account_id>

    # Send money
    $(basename "$0") send-money <from_id> <to_id> 50.00 "Family support"

    # Check balance
    $(basename "$0") balance <account_id>

MISSION:
    Providing financial access to 1.4 billion unbanked people worldwide
    through digital banking, mobile money, and simplified KYC processes.

For more information: https://github.com/WIA-Official/wia-standards
EOF
}

# Main command router
main() {
    init_dirs

    local command="${1:-help}"
    shift || true

    case "$command" in
        account-create)
            cmd_account_create "$@"
            ;;
        kyc-verify)
            cmd_kyc_verify "$@"
            ;;
        credit-score)
            cmd_credit_score "$@"
            ;;
        transaction)
            cmd_transaction "$@"
            ;;
        balance)
            cmd_balance "$@"
            ;;
        send-money)
            cmd_send_money "$@"
            ;;
        config)
            cmd_config "$@"
            ;;
        help|--help|-h)
            cmd_help
            ;;
        version|--version|-v)
            echo "WIA-FINANCIAL_INCLUSION CLI v$VERSION"
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            cmd_help
            return 1
            ;;
    esac
}

main "$@"
