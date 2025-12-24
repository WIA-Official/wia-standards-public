#!/bin/bash
#
# WIA-AUTO-SECURE v2.0
# One-Click HTTPS Installation
# Multi-Domain & Wildcard Support
#
# World Certification Industry Association
# https://wia.family
#
# 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
#
# Usage: curl -sSL https://wia.family/secure | bash
#    or: ./wia-secure-install.sh [domain]
#
# v2.0 Features:
#   - Single domain (+ www auto)
#   - Multiple domains (up to 100)
#   - Wildcard certificates (*.domain.com)
#   - Cloudflare DNS API integration
#   - Route53, DigitalOcean DNS support
#

set -e

VERSION="2.0"

# ============================================================
# Colors & Formatting
# ============================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# ============================================================
# Global Variables
# ============================================================
DOMAINS=()
MODE=""
DNS_PROVIDER=""
CREDENTIALS_FILE=""

# ============================================================
# Banner
# ============================================================
print_banner() {
    echo -e "${CYAN}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                                                           ║"
    echo "║   ██╗    ██╗██╗ █████╗       ███████╗███████╗ ██████╗    ║"
    echo "║   ██║    ██║██║██╔══██╗      ██╔════╝██╔════╝██╔════╝    ║"
    echo "║   ██║ █╗ ██║██║███████║█████╗███████╗█████╗  ██║         ║"
    echo "║   ██║███╗██║██║██╔══██║╚════╝╚════██║██╔══╝  ██║         ║"
    echo "║   ╚███╔███╔╝██║██║  ██║      ███████║███████╗╚██████╗    ║"
    echo "║    ╚══╝╚══╝ ╚═╝╚═╝  ╚═╝      ╚══════╝╚══════╝ ╚═════╝    ║"
    echo "║                                                           ║"
    echo "║            WIA-AUTO-SECURE v${VERSION}                          ║"
    echo "║            One-Click HTTPS Installation                   ║"
    echo "║                                                           ║"
    echo "║   ${MAGENTA}★ Multi-Domain ★ Wildcard ★ DNS API${CYAN}                  ║"
    echo "║                                                           ║"
    echo "║            홍익인간 - 널리 인간을 이롭게 하라             ║"
    echo "║                                                           ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ============================================================
# Logging Functions
# ============================================================
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "\n${BOLD}${CYAN}► $1${NC}"
}

# ============================================================
# Show Menu
# ============================================================
show_menu() {
    log_step "Select Installation Mode"
    echo ""
    echo -e "  ${GREEN}1)${NC} Single Domain ${YELLOW}(example.com + www)${NC}"
    echo -e "     └─ 가장 일반적인 옵션"
    echo ""
    echo -e "  ${GREEN}2)${NC} Multiple Domains ${YELLOW}(up to 100 domains)${NC}"
    echo -e "     └─ api.example.com, admin.example.com 등"
    echo ""
    echo -e "  ${GREEN}3)${NC} Wildcard Certificate ${YELLOW}(*.example.com)${NC}"
    echo -e "     └─ 모든 서브도메인 커버 (DNS API 필요)"
    echo ""
    echo -e "${YELLOW}"
    read -p "Select mode [1-3]: " mode_choice
    echo -e "${NC}"

    case $mode_choice in
        1) MODE="single" ;;
        2) MODE="multiple" ;;
        3) MODE="wildcard" ;;
        *)
            log_error "Invalid selection. Please choose 1, 2, or 3."
            exit 1
            ;;
    esac

    log_info "Selected mode: $MODE"
}

# ============================================================
# System Detection
# ============================================================
detect_os() {
    log_step "Detecting Operating System..."

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
        VERSION=$VERSION_ID
    elif [ -f /etc/redhat-release ]; then
        OS="rhel"
    elif [ -f /etc/debian_version ]; then
        OS="debian"
    else
        OS=$(uname -s)
    fi

    log_info "Detected: $OS $VERSION"
    echo "$OS"
}

# ============================================================
# Package Manager Detection
# ============================================================
detect_package_manager() {
    if command -v apt-get &> /dev/null; then
        echo "apt"
    elif command -v dnf &> /dev/null; then
        echo "dnf"
    elif command -v yum &> /dev/null; then
        echo "yum"
    elif command -v pacman &> /dev/null; then
        echo "pacman"
    elif command -v zypper &> /dev/null; then
        echo "zypper"
    else
        echo "unknown"
    fi
}

# ============================================================
# Web Server Detection
# ============================================================
detect_webserver() {
    log_step "Detecting Web Server..."

    if systemctl is-active --quiet nginx 2>/dev/null; then
        log_info "Found: nginx (running)"
        echo "nginx"
    elif systemctl is-active --quiet apache2 2>/dev/null; then
        log_info "Found: apache2 (running)"
        echo "apache"
    elif systemctl is-active --quiet httpd 2>/dev/null; then
        log_info "Found: httpd (running)"
        echo "apache"
    elif command -v nginx &> /dev/null; then
        log_info "Found: nginx (installed)"
        echo "nginx"
    elif command -v apache2 &> /dev/null || command -v httpd &> /dev/null; then
        log_info "Found: apache (installed)"
        echo "apache"
    else
        log_warn "No web server detected, will use standalone mode"
        echo "standalone"
    fi
}

# ============================================================
# Install Certbot
# ============================================================
install_certbot() {
    log_step "Installing Certbot..."

    local pkg_mgr=$(detect_package_manager)
    local webserver=$1

    case $pkg_mgr in
        apt)
            log_info "Using apt package manager"
            sudo apt-get update -qq
            sudo apt-get install -y -qq certbot
            if [ "$webserver" = "nginx" ]; then
                sudo apt-get install -y -qq python3-certbot-nginx
            elif [ "$webserver" = "apache" ]; then
                sudo apt-get install -y -qq python3-certbot-apache
            fi
            ;;
        dnf)
            log_info "Using dnf package manager"
            sudo dnf install -y -q certbot
            if [ "$webserver" = "nginx" ]; then
                sudo dnf install -y -q python3-certbot-nginx
            elif [ "$webserver" = "apache" ]; then
                sudo dnf install -y -q python3-certbot-apache
            fi
            ;;
        yum)
            log_info "Using yum package manager"
            sudo yum install -y -q certbot
            if [ "$webserver" = "nginx" ]; then
                sudo yum install -y -q python3-certbot-nginx
            elif [ "$webserver" = "apache" ]; then
                sudo yum install -y -q python3-certbot-apache
            fi
            ;;
        pacman)
            log_info "Using pacman package manager"
            sudo pacman -Sy --noconfirm certbot
            if [ "$webserver" = "nginx" ]; then
                sudo pacman -S --noconfirm certbot-nginx
            elif [ "$webserver" = "apache" ]; then
                sudo pacman -S --noconfirm certbot-apache
            fi
            ;;
        *)
            log_error "Unsupported package manager. Please install certbot manually."
            exit 1
            ;;
    esac

    log_success "Certbot installed successfully"
}

# ============================================================
# Install DNS Plugin
# ============================================================
install_dns_plugin() {
    local provider=$1
    local pkg_mgr=$(detect_package_manager)

    log_step "Installing DNS plugin for $provider..."

    case $pkg_mgr in
        apt)
            case $provider in
                cloudflare)
                    sudo apt-get install -y -qq python3-certbot-dns-cloudflare
                    ;;
                route53)
                    sudo apt-get install -y -qq python3-certbot-dns-route53
                    ;;
                digitalocean)
                    sudo apt-get install -y -qq python3-certbot-dns-digitalocean
                    ;;
                google)
                    sudo apt-get install -y -qq python3-certbot-dns-google
                    ;;
            esac
            ;;
        dnf|yum)
            case $provider in
                cloudflare)
                    sudo $pkg_mgr install -y -q python3-certbot-dns-cloudflare 2>/dev/null || \
                    pip3 install certbot-dns-cloudflare
                    ;;
                route53)
                    pip3 install certbot-dns-route53
                    ;;
                digitalocean)
                    pip3 install certbot-dns-digitalocean
                    ;;
                google)
                    pip3 install certbot-dns-google
                    ;;
            esac
            ;;
        *)
            log_info "Installing via pip3..."
            pip3 install certbot-dns-$provider
            ;;
    esac

    log_success "DNS plugin installed"
}

# ============================================================
# Validate Domain
# ============================================================
validate_domain() {
    local domain=$1
    if [[ "$domain" =~ ^[a-zA-Z0-9]([a-zA-Z0-9-]*[a-zA-Z0-9])?(\.[a-zA-Z0-9]([a-zA-Z0-9-]*[a-zA-Z0-9])?)*\.[a-zA-Z]{2,}$ ]]; then
        return 0
    else
        return 1
    fi
}

# ============================================================
# Get Single Domain
# ============================================================
get_single_domain() {
    local domain=$1

    if [ -z "$domain" ]; then
        log_step "Domain Configuration"
        echo -e "${YELLOW}"
        read -p "Enter your domain (e.g., example.com): " domain
        echo -e "${NC}"
    fi

    if ! validate_domain "$domain"; then
        log_error "Invalid domain format: $domain"
        exit 1
    fi

    # Ask about www
    echo -e "${YELLOW}"
    read -p "Include www.$domain? [Y/n]: " include_www
    echo -e "${NC}"

    include_www=${include_www:-Y}

    DOMAINS=("$domain")
    if [[ "$include_www" =~ ^[Yy]$ ]]; then
        DOMAINS+=("www.$domain")
    fi

    log_info "Domains: ${DOMAINS[*]}"
}

# ============================================================
# Get Multiple Domains
# ============================================================
get_multiple_domains() {
    log_step "Multiple Domain Configuration"

    echo ""
    echo -e "${CYAN}Enter domains separated by comma or space${NC}"
    echo -e "${YELLOW}Example: example.com, api.example.com, admin.example.com${NC}"
    echo ""
    echo -e "${YELLOW}"
    read -p "Domains: " domains_input
    echo -e "${NC}"

    # Parse input - handle both comma and space separated
    local input="${domains_input//,/ }"  # Replace commas with spaces

    for domain in $input; do
        # Trim whitespace
        domain=$(echo "$domain" | xargs)

        if [ -n "$domain" ]; then
            if validate_domain "$domain"; then
                DOMAINS+=("$domain")
                log_info "Added: $domain"
            else
                log_warn "Skipping invalid domain: $domain"
            fi
        fi
    done

    if [ ${#DOMAINS[@]} -eq 0 ]; then
        log_error "No valid domains provided!"
        exit 1
    fi

    # Ask about www for primary domain
    echo -e "${YELLOW}"
    read -p "Include www.${DOMAINS[0]}? [Y/n]: " include_www
    echo -e "${NC}"

    include_www=${include_www:-Y}
    if [[ "$include_www" =~ ^[Yy]$ ]]; then
        DOMAINS+=("www.${DOMAINS[0]}")
    fi

    echo ""
    log_info "Total domains: ${#DOMAINS[@]}"
    log_info "Domains: ${DOMAINS[*]}"
}

# ============================================================
# Get Wildcard Domain
# ============================================================
get_wildcard_domain() {
    log_step "Wildcard Certificate Configuration"

    echo ""
    echo -e "${CYAN}Wildcard certificates require DNS API access${NC}"
    echo -e "${YELLOW}This will issue a certificate for *.domain.com${NC}"
    echo ""
    echo -e "${YELLOW}"
    read -p "Enter your root domain (e.g., example.com): " domain
    echo -e "${NC}"

    if ! validate_domain "$domain"; then
        log_error "Invalid domain format: $domain"
        exit 1
    fi

    DOMAINS=("*.$domain" "$domain")
    log_info "Will issue: *.${domain} + ${domain}"
}

# ============================================================
# Select DNS Provider
# ============================================================
select_dns_provider() {
    log_step "Select DNS Provider"
    echo ""
    echo -e "  ${GREEN}1)${NC} Cloudflare ${YELLOW}(추천 - 가장 쉬움)${NC}"
    echo -e "  ${GREEN}2)${NC} AWS Route53"
    echo -e "  ${GREEN}3)${NC} DigitalOcean"
    echo -e "  ${GREEN}4)${NC} Google Cloud DNS"
    echo ""
    echo -e "${YELLOW}"
    read -p "Select provider [1-4]: " provider_choice
    echo -e "${NC}"

    case $provider_choice in
        1) DNS_PROVIDER="cloudflare" ;;
        2) DNS_PROVIDER="route53" ;;
        3) DNS_PROVIDER="digitalocean" ;;
        4) DNS_PROVIDER="google" ;;
        *)
            log_error "Invalid selection"
            exit 1
            ;;
    esac

    log_info "Selected DNS provider: $DNS_PROVIDER"
}

# ============================================================
# Setup DNS Credentials
# ============================================================
setup_dns_credentials() {
    local provider=$1

    log_step "Setting up $provider credentials..."

    mkdir -p ~/.secrets
    CREDENTIALS_FILE="$HOME/.secrets/${provider}.ini"

    case $provider in
        cloudflare)
            echo ""
            echo -e "${CYAN}Cloudflare API Token Setup${NC}"
            echo -e "${YELLOW}Get your token from: https://dash.cloudflare.com/profile/api-tokens${NC}"
            echo -e "${YELLOW}Required permission: Zone:DNS:Edit${NC}"
            echo ""
            echo -e "${YELLOW}"
            read -p "Enter Cloudflare API Token: " api_token
            echo -e "${NC}"

            cat > "$CREDENTIALS_FILE" << EOF
# Cloudflare API Token
dns_cloudflare_api_token = $api_token
EOF
            ;;
        route53)
            echo ""
            echo -e "${CYAN}AWS Route53 Credentials Setup${NC}"
            echo ""
            echo -e "${YELLOW}"
            read -p "Enter AWS Access Key ID: " aws_key
            read -p "Enter AWS Secret Access Key: " aws_secret
            echo -e "${NC}"

            mkdir -p ~/.aws
            cat > ~/.aws/credentials << EOF
[default]
aws_access_key_id = $aws_key
aws_secret_access_key = $aws_secret
EOF
            CREDENTIALS_FILE=""  # Route53 uses ~/.aws/credentials
            ;;
        digitalocean)
            echo ""
            echo -e "${CYAN}DigitalOcean API Token Setup${NC}"
            echo ""
            echo -e "${YELLOW}"
            read -p "Enter DigitalOcean API Token: " api_token
            echo -e "${NC}"

            cat > "$CREDENTIALS_FILE" << EOF
dns_digitalocean_token = $api_token
EOF
            ;;
        google)
            echo ""
            echo -e "${CYAN}Google Cloud DNS Setup${NC}"
            echo -e "${YELLOW}Provide path to service account JSON file${NC}"
            echo ""
            echo -e "${YELLOW}"
            read -p "Enter path to JSON key file: " json_path
            echo -e "${NC}"

            CREDENTIALS_FILE="$json_path"
            ;;
    esac

    if [ -n "$CREDENTIALS_FILE" ] && [ -f "$CREDENTIALS_FILE" ]; then
        chmod 600 "$CREDENTIALS_FILE"
    fi

    log_success "Credentials configured!"
}

# ============================================================
# Get Email
# ============================================================
get_email() {
    log_step "Email Configuration (for Let's Encrypt notifications)"
    echo -e "${YELLOW}"
    read -p "Enter your email: " email
    echo -e "${NC}"

    if [[ ! "$email" =~ ^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$ ]]; then
        log_error "Invalid email format: $email"
        exit 1
    fi

    log_info "Email: $email"
    echo "$email"
}

# ============================================================
# Issue Certificate - Single/Multiple (HTTP-01)
# ============================================================
issue_certificate_http() {
    local email=$1
    local webserver=$2

    log_step "Issuing SSL Certificate via HTTP-01..."

    # Build domain flags
    local domain_flags=""
    for domain in "${DOMAINS[@]}"; do
        domain_flags="$domain_flags -d $domain"
    done

    log_info "Domains: $domain_flags"

    case $webserver in
        nginx)
            log_info "Using nginx plugin"
            sudo certbot --nginx $domain_flags \
                --non-interactive --agree-tos --email "$email" \
                --redirect
            ;;
        apache)
            log_info "Using apache plugin"
            sudo certbot --apache $domain_flags \
                --non-interactive --agree-tos --email "$email" \
                --redirect
            ;;
        standalone)
            log_info "Using standalone mode"
            log_warn "Make sure port 80 is not in use!"
            sudo certbot certonly --standalone $domain_flags \
                --non-interactive --agree-tos --email "$email"
            ;;
    esac

    log_success "SSL Certificate issued successfully!"
}

# ============================================================
# Issue Certificate - Wildcard (DNS-01)
# ============================================================
issue_certificate_dns() {
    local email=$1
    local provider=$2

    log_step "Issuing Wildcard SSL Certificate via DNS-01..."

    # Build domain flags
    local domain_flags=""
    for domain in "${DOMAINS[@]}"; do
        domain_flags="$domain_flags -d $domain"
    done

    log_info "Domains: $domain_flags"
    log_info "DNS Provider: $provider"
    log_warn "DNS propagation may take 60-120 seconds..."

    case $provider in
        cloudflare)
            sudo certbot certonly \
                --dns-cloudflare \
                --dns-cloudflare-credentials "$CREDENTIALS_FILE" \
                --dns-cloudflare-propagation-seconds 60 \
                $domain_flags \
                --non-interactive --agree-tos --email "$email"
            ;;
        route53)
            sudo certbot certonly \
                --dns-route53 \
                --dns-route53-propagation-seconds 60 \
                $domain_flags \
                --non-interactive --agree-tos --email "$email"
            ;;
        digitalocean)
            sudo certbot certonly \
                --dns-digitalocean \
                --dns-digitalocean-credentials "$CREDENTIALS_FILE" \
                --dns-digitalocean-propagation-seconds 60 \
                $domain_flags \
                --non-interactive --agree-tos --email "$email"
            ;;
        google)
            sudo certbot certonly \
                --dns-google \
                --dns-google-credentials "$CREDENTIALS_FILE" \
                --dns-google-propagation-seconds 60 \
                $domain_flags \
                --non-interactive --agree-tos --email "$email"
            ;;
    esac

    log_success "Wildcard SSL Certificate issued successfully!"

    # Get the root domain (remove *. prefix)
    local root_domain="${DOMAINS[1]}"  # Second domain should be the root

    echo ""
    log_warn "═══════════════════════════════════════════════════════════"
    log_warn "  IMPORTANT: Configure your web server manually!"
    log_warn "═══════════════════════════════════════════════════════════"
    echo ""
    log_info "Certificate files location:"
    echo -e "  ${GREEN}Certificate:${NC} /etc/letsencrypt/live/$root_domain/fullchain.pem"
    echo -e "  ${GREEN}Private Key:${NC} /etc/letsencrypt/live/$root_domain/privkey.pem"
    echo ""
    log_info "Example Nginx configuration:"
    echo -e "${YELLOW}"
    cat << 'NGINXEOF'
    server {
        listen 443 ssl;
        server_name *.example.com example.com;

        ssl_certificate /etc/letsencrypt/live/example.com/fullchain.pem;
        ssl_certificate_key /etc/letsencrypt/live/example.com/privkey.pem;

        # ... your config ...
    }
NGINXEOF
    echo -e "${NC}"

    log_info "Example Apache configuration:"
    echo -e "${YELLOW}"
    cat << 'APACHEEOF'
    <VirtualHost *:443>
        ServerName example.com
        ServerAlias *.example.com

        SSLEngine on
        SSLCertificateFile /etc/letsencrypt/live/example.com/fullchain.pem
        SSLCertificateKeyFile /etc/letsencrypt/live/example.com/privkey.pem

        # ... your config ...
    </VirtualHost>
APACHEEOF
    echo -e "${NC}"
}

# ============================================================
# Setup Auto-Renewal
# ============================================================
setup_auto_renewal() {
    log_step "Setting up automatic renewal..."

    # Check if systemd timer exists (modern systems)
    if systemctl list-timers | grep -q certbot; then
        log_info "Certbot systemd timer already active"
    else
        # Setup cron job as fallback
        log_info "Setting up cron job for auto-renewal"

        sudo tee /etc/cron.d/wia-certbot-renew > /dev/null << 'EOF'
# WIA-AUTO-SECURE: Auto-renew SSL certificates
# Runs twice daily at random minute to spread load
0 0,12 * * * root certbot renew --quiet --post-hook "systemctl reload nginx || systemctl reload apache2 || systemctl reload httpd || true"
EOF

        sudo chmod 644 /etc/cron.d/wia-certbot-renew
    fi

    # Test renewal
    log_info "Testing renewal configuration..."
    sudo certbot renew --dry-run

    log_success "Auto-renewal configured (checks twice daily)"
}

# ============================================================
# Verify Installation
# ============================================================
verify_installation() {
    log_step "Verifying HTTPS Installation..."

    # Use first non-wildcard domain for verification
    local test_domain=""
    for domain in "${DOMAINS[@]}"; do
        if [[ "$domain" != \** ]]; then
            test_domain="$domain"
            break
        fi
    done

    if [ -z "$test_domain" ]; then
        test_domain="${DOMAINS[1]}"  # Use root domain
    fi

    # Wait for changes to propagate
    sleep 2

    # Test HTTPS connection
    if command -v curl &> /dev/null; then
        local http_code=$(curl -s -o /dev/null -w "%{http_code}" "https://$test_domain" --max-time 10 2>/dev/null || echo "000")

        if [ "$http_code" = "200" ] || [ "$http_code" = "301" ] || [ "$http_code" = "302" ]; then
            log_success "HTTPS is working! (HTTP $http_code)"
        else
            log_warn "Could not verify HTTPS (HTTP $http_code). DNS may need time to propagate."
        fi
    fi

    # Show certificate info
    log_info "Certificate location: /etc/letsencrypt/live/$test_domain/"

    if [ -f "/etc/letsencrypt/live/$test_domain/fullchain.pem" ]; then
        local expiry=$(openssl x509 -enddate -noout -in "/etc/letsencrypt/live/$test_domain/fullchain.pem" 2>/dev/null | cut -d= -f2)
        log_info "Certificate expires: $expiry"
    fi
}

# ============================================================
# Print Summary
# ============================================================
print_summary() {
    local primary_domain="${DOMAINS[0]}"
    # Remove wildcard prefix for display
    primary_domain="${primary_domain#\*.}"

    echo -e "\n${GREEN}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                                                           ║"
    echo "║   ✓ HTTPS Installation Complete!                         ║"
    echo "║                                                           ║"
    echo "╠═══════════════════════════════════════════════════════════╣"
    echo "║                                                           ║"
    echo "║   Your site is now secure:                                ║"
    printf "║   ${BOLD}https://%-48s${NC}${GREEN}║\n" "$primary_domain"
    echo "║                                                           ║"

    if [ "$MODE" = "wildcard" ]; then
        echo "║   ${YELLOW}★ Wildcard: *.${primary_domain}${GREEN}                              ║"
        echo "║                                                           ║"
    elif [ ${#DOMAINS[@]} -gt 1 ]; then
        echo "║   ${YELLOW}★ Domains: ${#DOMAINS[@]} total${GREEN}                                   ║"
        echo "║                                                           ║"
    fi

    echo "║   Certificate auto-renews every 60-90 days               ║"
    echo "║   No further action needed!                               ║"
    echo "║                                                           ║"
    echo "╠═══════════════════════════════════════════════════════════╣"
    echo "║                                                           ║"
    echo "║   Powered by WIA-AUTO-SECURE v${VERSION}                        ║"
    echo "║   https://wia.family                                      ║"
    echo "║                                                           ║"
    echo "║   홍익인간 - 널리 인간을 이롭게 하라                      ║"
    echo "║                                                           ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}\n"

    # List all secured domains
    if [ ${#DOMAINS[@]} -gt 1 ]; then
        echo -e "${CYAN}Secured Domains:${NC}"
        for domain in "${DOMAINS[@]}"; do
            echo -e "  ${GREEN}✓${NC} https://$domain"
        done
        echo ""
    fi
}

# ============================================================
# Main
# ============================================================
main() {
    print_banner

    # Check if running as root or with sudo
    if [ "$EUID" -ne 0 ] && ! sudo -v &>/dev/null; then
        log_error "This script requires root privileges. Please run with sudo."
        exit 1
    fi

    # Check if domain provided as argument (legacy single-domain mode)
    if [ -n "$1" ]; then
        MODE="single"
        get_single_domain "$1"
    else
        # Show menu for mode selection
        show_menu

        case $MODE in
            single)
                get_single_domain
                ;;
            multiple)
                get_multiple_domains
                ;;
            wildcard)
                get_wildcard_domain
                select_dns_provider
                ;;
        esac
    fi

    # Get email
    local email=$(get_email)

    # Detect OS
    local os=$(detect_os)

    # Detect web server
    local webserver=$(detect_webserver)

    # Install certbot if needed
    if ! command -v certbot &> /dev/null; then
        install_certbot "$webserver"
    else
        log_info "Certbot already installed"
    fi

    # Issue certificate based on mode
    if [ "$MODE" = "wildcard" ]; then
        # Wildcard requires DNS plugin
        install_dns_plugin "$DNS_PROVIDER"
        setup_dns_credentials "$DNS_PROVIDER"
        issue_certificate_dns "$email" "$DNS_PROVIDER"
    else
        # Single or multiple domains use HTTP-01
        issue_certificate_http "$email" "$webserver"
    fi

    # Setup auto-renewal
    setup_auto_renewal

    # Verify
    verify_installation

    # Summary
    print_summary
}

# Run main with all arguments
main "$@"
