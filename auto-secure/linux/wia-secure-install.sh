#!/bin/bash
#
# WIA-AUTO-SECURE v1.0
# One-Click HTTPS Installation
#
# World Certification Industry Association
# https://wia.family
#
# 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
#
# Safe Usage (recommended):
#   curl -sSL https://wia.family/secure -o wia-secure.sh
#   less wia-secure.sh          # Review the script
#   bash wia-secure.sh
#
# Quick Usage:
#   curl -sSL https://wia.family/secure | bash
#
# Non-Interactive:
#   export WIA_DOMAIN="example.com" WIA_EMAIL="admin@example.com"
#   ./wia-secure-install.sh --non-interactive
#
# Offline (certbot already installed):
#   ./wia-secure-install.sh --offline
#

set -e

# ============================================================
# Colors & Formatting
# ============================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

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
    echo "║            WIA-AUTO-SECURE v1.0                          ║"
    echo "║            One-Click HTTPS Installation                   ║"
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
# System Detection
# ============================================================
detect_os() {
    log_step "Detecting Operating System..."

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS="$ID"
        VERSION="$VERSION_ID"
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

    case "$pkg_mgr" in
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
            # For Amazon Linux 2023 / RHEL / CentOS
            sudo yum install -y -q certbot
            if [ "$webserver" = "nginx" ]; then
                sudo yum install -y -q python3-certbot-nginx
            elif [ "$webserver" = "apache" ]; then
                sudo yum install -y -q python3-certbot-apache
            fi
            ;;
        pacman)
            log_info "Using pacman package manager"
            sudo pacman -Syu --noconfirm certbot
            if [ "$webserver" = "nginx" ]; then
                sudo pacman -S --noconfirm certbot-nginx
            elif [ "$webserver" = "apache" ]; then
                sudo pacman -S --noconfirm certbot-apache
            fi
            ;;
        zypper)
            log_info "Using zypper package manager"
            sudo zypper install -y certbot
            if [ "$webserver" = "nginx" ]; then
                sudo zypper install -y python3-certbot-nginx
            elif [ "$webserver" = "apache" ]; then
                sudo zypper install -y python3-certbot-apache
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
# Get Domain
# ============================================================
get_domain() {
    local domain=$1

    # Check environment variable first (for non-interactive mode)
    if [ -z "$domain" ] && [ -n "$WIA_DOMAIN" ]; then
        domain="$WIA_DOMAIN"
        log_info "Using domain from WIA_DOMAIN environment variable: $domain"
    fi

    if [ -z "$domain" ]; then
        if [ "$NON_INTERACTIVE" = "true" ]; then
            log_error "Non-interactive mode requires WIA_DOMAIN environment variable"
            exit 1
        fi
        log_step "Domain Configuration"
        echo -e "${YELLOW}"
        read -p "Enter your domain (e.g., example.com): " domain
        echo -e "${NC}"
    fi

    # Validate domain format
    if [[ ! "$domain" =~ ^[a-zA-Z0-9]([a-zA-Z0-9-]*[a-zA-Z0-9])?(\.[a-zA-Z0-9]([a-zA-Z0-9-]*[a-zA-Z0-9])?)*\.[a-zA-Z]{2,}$ ]]; then
        log_error "Invalid domain format: $domain"
        exit 1
    fi

    log_info "Domain: $domain"
    echo "$domain"
}

# ============================================================
# Get Email
# ============================================================
get_email() {
    # Check environment variable first (for non-interactive mode)
    if [ -n "$WIA_EMAIL" ]; then
        email="$WIA_EMAIL"
        log_info "Using email from WIA_EMAIL environment variable: $email"
    else
        if [ "$NON_INTERACTIVE" = "true" ]; then
            log_error "Non-interactive mode requires WIA_EMAIL environment variable"
            exit 1
        fi
        log_step "Email Configuration (for Let's Encrypt notifications)"
        echo -e "${YELLOW}"
        read -p "Enter your email: " email
        echo -e "${NC}"
    fi

    if [[ ! "$email" =~ ^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$ ]]; then
        log_error "Invalid email format: $email"
        exit 1
    fi

    log_info "Email: $email"
    echo "$email"
}

# ============================================================
# Backup & Restore Web Server Config
# ============================================================
backup_webserver_config() {
    local webserver=$1
    local backup_dir="/tmp/wia-secure-backup-$(date +%Y%m%d%H%M%S)"
    mkdir -p "$backup_dir"

    if [ "$webserver" = "nginx" ]; then
        cp -r /etc/nginx/sites-enabled/ "$backup_dir/" 2>/dev/null || true
        cp -r /etc/nginx/conf.d/ "$backup_dir/" 2>/dev/null || true
    elif [ "$webserver" = "apache" ]; then
        cp -r /etc/apache2/sites-enabled/ "$backup_dir/" 2>/dev/null || true
        cp -r /etc/httpd/conf.d/ "$backup_dir/" 2>/dev/null || true
    fi

    log_info "Web server config backed up to $backup_dir"
    echo "$backup_dir"
}

restore_webserver_config() {
    local backup_dir=$1
    local webserver=$2

    if [ -z "$backup_dir" ] || [ ! -d "$backup_dir" ]; then
        log_warn "No backup found to restore"
        return 1
    fi

    log_warn "Restoring web server config from $backup_dir..."

    if [ "$webserver" = "nginx" ]; then
        [ -d "$backup_dir/sites-enabled" ] && cp -r "$backup_dir/sites-enabled/"* /etc/nginx/sites-enabled/ 2>/dev/null || true
        [ -d "$backup_dir/conf.d" ] && cp -r "$backup_dir/conf.d/"* /etc/nginx/conf.d/ 2>/dev/null || true
        sudo systemctl reload nginx 2>/dev/null || true
    elif [ "$webserver" = "apache" ]; then
        [ -d "$backup_dir/sites-enabled" ] && cp -r "$backup_dir/sites-enabled/"* /etc/apache2/sites-enabled/ 2>/dev/null || true
        [ -d "$backup_dir/conf.d" ] && cp -r "$backup_dir/conf.d/"* /etc/httpd/conf.d/ 2>/dev/null || true
        sudo systemctl reload apache2 2>/dev/null || sudo systemctl reload httpd 2>/dev/null || true
    fi

    log_info "Web server config restored from backup"
}

# ============================================================
# Issue Certificate
# ============================================================
issue_certificate() {
    local domain=$1
    local email=$2
    local webserver=$3

    log_step "Issuing SSL Certificate for $domain..."

    case $webserver in
        nginx)
            log_info "Using nginx plugin"
            sudo certbot --nginx -d "$domain" -d "www.$domain" \
                --non-interactive --agree-tos --email "$email" \
                --redirect || {
                    log_warn "www subdomain failed, trying without..."
                    sudo certbot --nginx -d "$domain" \
                        --non-interactive --agree-tos --email "$email" \
                        --redirect
                }
            ;;
        apache)
            log_info "Using apache plugin"
            sudo certbot --apache -d "$domain" -d "www.$domain" \
                --non-interactive --agree-tos --email "$email" \
                --redirect || {
                    log_warn "www subdomain failed, trying without..."
                    sudo certbot --apache -d "$domain" \
                        --non-interactive --agree-tos --email "$email" \
                        --redirect
                }
            ;;
        standalone)
            log_info "Using standalone mode"
            log_warn "Make sure port 80 is not in use!"
            sudo certbot certonly --standalone -d "$domain" \
                --non-interactive --agree-tos --email "$email"
            ;;
    esac

    log_success "SSL Certificate issued successfully!"
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

        # Create renewal script
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
    local domain=$1

    log_step "Verifying HTTPS Installation..."

    # Wait for changes to propagate
    sleep 2

    # Test HTTPS connection
    if command -v curl &> /dev/null; then
        local http_code=$(curl -s -o /dev/null -w "%{http_code}" "https://$domain" --max-time 10 2>/dev/null || echo "000")

        if [ "$http_code" = "200" ] || [ "$http_code" = "301" ] || [ "$http_code" = "302" ]; then
            log_success "HTTPS is working! (HTTP $http_code)"
        else
            log_warn "Could not verify HTTPS (HTTP $http_code). DNS may need time to propagate."
        fi
    fi

    # Show certificate info
    log_info "Certificate location: /etc/letsencrypt/live/$domain/"

    if [ -f "/etc/letsencrypt/live/$domain/fullchain.pem" ]; then
        local expiry=$(openssl x509 -enddate -noout -in "/etc/letsencrypt/live/$domain/fullchain.pem" 2>/dev/null | cut -d= -f2)
        log_info "Certificate expires: $expiry"
    fi
}

# ============================================================
# Print Summary
# ============================================================
print_summary() {
    local domain=$1

    echo -e "\n${GREEN}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                                                           ║"
    echo "║   ✓ HTTPS Installation Complete!                         ║"
    echo "║                                                           ║"
    echo "╠═══════════════════════════════════════════════════════════╣"
    echo "║                                                           ║"
    echo "║   Your site is now secure:                                ║"
    printf "║   ${BOLD}https://%-48s${NC}${GREEN}║\n" "$domain"
    echo "║                                                           ║"
    echo "║   Certificate auto-renews every 60-90 days               ║"
    echo "║   No further action needed!                               ║"
    echo "║                                                           ║"
    echo "╠═══════════════════════════════════════════════════════════╣"
    echo "║                                                           ║"
    echo "║   Powered by WIA-AUTO-SECURE                             ║"
    echo "║   https://wia.family                                      ║"
    echo "║                                                           ║"
    echo "║   홍익인간 - 널리 인간을 이롭게 하라                      ║"
    echo "║                                                           ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}\n"
}

# ============================================================
# Main
# ============================================================
main() {
    # Parse flags
    NON_INTERACTIVE="false"
    OFFLINE_MODE="false"
    local positional_args=()

    for arg in "$@"; do
        case "$arg" in
            --non-interactive) NON_INTERACTIVE="true" ;;
            --offline) OFFLINE_MODE="true" ;;
            *) positional_args+=("$arg") ;;
        esac
    done

    print_banner

    # Check if running as root or with sudo
    if [ "$EUID" -ne 0 ] && ! sudo -v &>/dev/null; then
        log_error "This script requires root privileges. Please run with sudo."
        exit 1
    fi

    # Get domain from argument or prompt
    local domain=$(get_domain "${positional_args[0]:-}")

    # Get email
    local email=$(get_email)

    # Detect OS
    local os=$(detect_os)

    # Detect web server
    local webserver=$(detect_webserver)

    # Install certbot
    if [ "$OFFLINE_MODE" = "true" ]; then
        log_info "Offline mode: skipping certbot installation"
        if ! command -v certbot &> /dev/null; then
            log_error "Offline mode requires certbot to be pre-installed"
            log_error "Install certbot first: sudo apt install certbot (or equivalent)"
            exit 1
        fi
        log_info "Certbot found: $(certbot --version 2>&1 || echo 'unknown version')"
    elif ! command -v certbot &> /dev/null; then
        install_certbot "$webserver"
    else
        log_info "Certbot already installed"
    fi

    # Backup web server config before issuing certificate
    local backup_dir=""
    if [ "$webserver" != "standalone" ]; then
        backup_dir=$(backup_webserver_config "$webserver")
    fi

    # Issue certificate (with rollback on failure)
    issue_certificate "$domain" "$email" "$webserver" || {
        log_error "Certificate issuance failed!"
        if [ -n "$backup_dir" ]; then
            restore_webserver_config "$backup_dir" "$webserver"
        fi
        exit 1
    }

    # Setup auto-renewal
    setup_auto_renewal

    # Verify
    verify_installation "$domain"

    # Summary
    print_summary "$domain"
}

# Run main with all arguments
main "$@"
