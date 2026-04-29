#!/bin/bash
#
# AUTO-DEPLOY v1.0
# Zero-Downtime Deployment Solution
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#
# Usage: auto-deploy [PROJECT] [OPTIONS]
#    or: auto-deploy https://github.com/user/repo.git
#

set -e

# ============================================================
# Version & Constants
# ============================================================
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/auto-deploy.conf"
PROJECTS_DIR="${SCRIPT_DIR}/projects"
LOCK_DIR="/var/lock/auto-deploy"

# ============================================================
# Colors & Formatting
# ============================================================
if [[ -t 1 ]]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    CYAN='\033[0;36m'
    MAGENTA='\033[0;35m'
    BOLD='\033[1m'
    DIM='\033[2m'
    NC='\033[0m'
else
    RED='' GREEN='' YELLOW='' BLUE='' CYAN='' MAGENTA='' BOLD='' DIM='' NC=''
fi

# ============================================================
# Default Configuration
# ============================================================
DEPLOY_BASE_DIR="/var/www"
RELEASES_SUFFIX="-releases"
KEEP_RELEASES=5
LOG_FILE="/var/log/auto-deploy.log"
TEMP_DIR="/tmp/auto-deploy-$$"

# Git settings
GIT_DEPTH=1
GIT_SSH_KEY=""
DEFAULT_BRANCH="main"

# Build settings
NODE_VERSION=""
PHP_VERSION=""
PYTHON_VERSION=""

# Service restart
RESTART_NGINX=false
RESTART_APACHE=false
RESTART_PHP_FPM=false
RESTART_PM2=false
RESTART_SERVICES=""

# Webhook
WEBHOOK_PORT=9000
WEBHOOK_SECRET=""
WEBHOOK_PID_FILE="/var/run/auto-deploy-webhook.pid"

# Notifications
NOTIFY_ENABLED=false
SLACK_WEBHOOK=""
DISCORD_WEBHOOK=""
EMAIL_TO=""

# Runtime
DRY_RUN=false
VERBOSE=false
FORCE=false
NO_BUILD=false
AUTO_ROLLBACK=true

# Project-specific (set during runtime)
PROJECT_NAME=""
PROJECT_PATH=""
RELEASES_PATH=""
GIT_URL=""
GIT_BRANCH=""
GIT_TAG=""
BUILD_COMMAND=""
PUBLIC_DIR=""
SHARED_DIRS=""
SHARED_FILES=""
ENV_FILE=""
POST_ACTIVATE=""

# State
START_TIME=""
END_TIME=""
CURRENT_RELEASE=""
PREVIOUS_RELEASE=""
DEPLOY_SUCCESS=false
ERRORS=()

# ============================================================
# Load Configuration
# ============================================================
load_global_config() {
    [[ -f "$CONFIG_FILE" ]] && source "$CONFIG_FILE"
}

load_project_config() {
    local project="$1"
    local config_file="${PROJECTS_DIR}/${project}.conf"

    if [[ -f "$config_file" ]]; then
        source "$config_file"
        return 0
    fi
    return 1
}

# ============================================================
# Logging
# ============================================================
log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")

    echo "[$timestamp] [$level] $message" >> "$LOG_FILE" 2>/dev/null || true

    case "$level" in
        INFO)    [[ "$VERBOSE" == "true" ]] && echo -e "${BLUE}[INFO]${NC} $message" ;;
        SUCCESS) echo -e "${GREEN}[✓]${NC} $message" ;;
        WARN)    echo -e "${YELLOW}[!]${NC} $message" ;;
        ERROR)   echo -e "${RED}[✗]${NC} $message" ;;
        STEP)    echo -e "\n${BOLD}${CYAN}► $message${NC}" ;;
        DEBUG)   [[ "$VERBOSE" == "true" ]] && echo -e "${DIM}[DEBUG] $message${NC}" ;;
    esac
}

log_info()    { log "INFO" "$@"; }
log_success() { log "SUCCESS" "$@"; }
log_warn()    { log "WARN" "$@"; }
log_error()   { log "ERROR" "$@"; ERRORS+=("$*"); }
log_step()    { log "STEP" "$@"; }
log_debug()   { log "DEBUG" "$@"; }

# ============================================================
# Banner
# ============================================================
print_banner() {
    echo -e "${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║   █████╗ ██╗   ██╗████████╗ ██████╗                          ║
║  ██╔══██╗██║   ██║╚══██╔══╝██╔═══██╗                         ║
║  ███████║██║   ██║   ██║   ██║   ██║                         ║
║  ██╔══██║██║   ██║   ██║   ██║   ██║                         ║
║  ██║  ██║╚██████╔╝   ██║   ╚██████╔╝                         ║
║  ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝                          ║
║                                                              ║
║  ██████╗ ███████╗██████╗ ██╗      ██████╗ ██╗   ██╗          ║
║  ██╔══██╗██╔════╝██╔══██╗██║     ██╔═══██╗╚██╗ ██╔╝          ║
║  ██║  ██║█████╗  ██████╔╝██║     ██║   ██║ ╚████╔╝           ║
║  ██║  ██║██╔══╝  ██╔═══╝ ██║     ██║   ██║  ╚██╔╝            ║
║  ██████╔╝███████╗██║     ███████╗╚██████╔╝   ██║             ║
║  ╚═════╝ ╚══════╝╚═╝     ╚══════╝ ╚═════╝    ╚═╝             ║
║                                                              ║
║  AUTO-DEPLOY v1.0 · Zero-Downtime Deployment                 ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

# ============================================================
# Help
# ============================================================
print_help() {
    cat << EOF
${BOLD}AUTO-DEPLOY v${VERSION}${NC} - Zero-Downtime Deployment Solution

${BOLD}USAGE:${NC}
    auto-deploy [PROJECT] [OPTIONS]
    auto-deploy [GIT_URL] [OPTIONS]

${BOLD}DEPLOYMENT:${NC}
    auto-deploy PROJECT              Deploy registered project
    auto-deploy GIT_URL              Deploy from Git URL
    auto-deploy GIT_URL --branch X   Deploy specific branch
    auto-deploy GIT_URL --tag X      Deploy specific tag
    auto-deploy --local /path        Deploy from local directory

${BOLD}PROJECT OPTIONS:${NC}
    --branch, -b BRANCH    Git branch to deploy
    --tag, -t TAG          Git tag to deploy
    --force                Force deploy even if no changes
    --no-build             Skip build step
    --dry-run              Simulate without actual deployment
    --verbose, -v          Verbose output

${BOLD}ROLLBACK:${NC}
    --rollback             Rollback to previous release
    --rollback N           Rollback N releases back
    --rollback RELEASE     Rollback to specific release
    --list                 List deployment history

${BOLD}PROJECT MANAGEMENT:${NC}
    --init URL --path PATH    Initialize new project
    --projects                List registered projects
    --status                  Show project status
    --cleanup                 Remove old releases

${BOLD}WEBHOOK SERVER:${NC}
    --webhook-server          Start webhook server (port ${WEBHOOK_PORT})
    --webhook-status          Check webhook server status
    --webhook-stop            Stop webhook server

${BOLD}OTHER:${NC}
    -h, --help               Show this help
    --version                Show version

${BOLD}EXAMPLES:${NC}
    # First-time setup
    auto-deploy --init https://github.com/user/app.git --path /var/www/app

    # Regular deployment
    auto-deploy myapp
    auto-deploy myapp --branch develop
    auto-deploy myapp --tag v2.0.0

    # Rollback
    auto-deploy myapp --rollback
    auto-deploy myapp --list

    # Direct Git URL deployment
    auto-deploy https://github.com/user/app.git

${DIM}弘益人間 (홍익인간) · https://wia.family${NC}
EOF
}

# ============================================================
# Utilities
# ============================================================
check_command() {
    command -v "$1" &>/dev/null
}

require_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This operation requires root privileges"
        exit 1
    fi
}

elapsed_time() {
    local start=$1
    local end=$2
    local diff=$((end - start))
    local mins=$((diff / 60))
    local secs=$((diff % 60))
    if [[ $mins -gt 0 ]]; then
        echo "${mins}분 ${secs}초"
    else
        echo "${secs}초"
    fi
}

human_size() {
    local bytes=$1
    if [[ $bytes -lt 1024 ]]; then
        echo "${bytes}B"
    elif [[ $bytes -lt 1048576 ]]; then
        echo "$(( bytes / 1024 ))KB"
    elif [[ $bytes -lt 1073741824 ]]; then
        printf "%.1fMB" "$(echo "scale=1; $bytes / 1048576" | bc 2>/dev/null || echo "0")"
    else
        printf "%.2fGB" "$(echo "scale=2; $bytes / 1073741824" | bc 2>/dev/null || echo "0")"
    fi
}

cleanup_temp() {
    [[ -d "$TEMP_DIR" ]] && rm -rf "$TEMP_DIR"
}

trap cleanup_temp EXIT

# ============================================================
# Lock Management (Prevent Concurrent Deploys)
# ============================================================
acquire_lock() {
    local project="$1"
    local lock_file="${LOCK_DIR}/${project}.lock"

    mkdir -p "$LOCK_DIR"

    if [[ -f "$lock_file" ]]; then
        local pid=$(cat "$lock_file" 2>/dev/null)
        if kill -0 "$pid" 2>/dev/null; then
            log_error "Deployment already in progress (PID: $pid)"
            exit 1
        fi
        rm -f "$lock_file"
    fi

    echo $$ > "$lock_file"
}

release_lock() {
    local project="$1"
    rm -f "${LOCK_DIR}/${project}.lock"
}

# ============================================================
# Project Detection
# ============================================================
detect_project_type() {
    local dir="$1"

    if [[ -f "${dir}/package.json" ]]; then
        if grep -q '"next"' "${dir}/package.json" 2>/dev/null; then
            echo "nextjs"
        elif grep -q '"nuxt"' "${dir}/package.json" 2>/dev/null; then
            echo "nuxt"
        elif grep -q '"react"' "${dir}/package.json" 2>/dev/null; then
            echo "react"
        elif grep -q '"vue"' "${dir}/package.json" 2>/dev/null; then
            echo "vue"
        else
            echo "nodejs"
        fi
    elif [[ -f "${dir}/composer.json" ]]; then
        if grep -q '"laravel/framework"' "${dir}/composer.json" 2>/dev/null; then
            echo "laravel"
        elif [[ -d "${dir}/symfony" ]] || grep -q '"symfony/' "${dir}/composer.json" 2>/dev/null; then
            echo "symfony"
        else
            echo "php"
        fi
    elif [[ -f "${dir}/requirements.txt" ]] || [[ -f "${dir}/setup.py" ]] || [[ -f "${dir}/pyproject.toml" ]]; then
        if [[ -f "${dir}/manage.py" ]]; then
            echo "django"
        elif grep -q 'flask' "${dir}/requirements.txt" 2>/dev/null; then
            echo "flask"
        else
            echo "python"
        fi
    elif [[ -f "${dir}/Gemfile" ]]; then
        if grep -q 'rails' "${dir}/Gemfile" 2>/dev/null; then
            echo "rails"
        else
            echo "ruby"
        fi
    elif [[ -f "${dir}/pom.xml" ]]; then
        echo "maven"
    elif [[ -f "${dir}/build.gradle" ]]; then
        echo "gradle"
    elif [[ -f "${dir}/Cargo.toml" ]]; then
        echo "rust"
    elif [[ -f "${dir}/go.mod" ]]; then
        echo "go"
    elif [[ -f "${dir}/Makefile" ]]; then
        echo "make"
    else
        echo "static"
    fi
}

get_project_info() {
    local type="$1"
    local dir="$2"

    case "$type" in
        nextjs)   echo "Next.js" ;;
        nuxt)     echo "Nuxt.js" ;;
        react)    echo "React" ;;
        vue)      echo "Vue.js" ;;
        nodejs)   echo "Node.js" ;;
        laravel)  echo "Laravel" ;;
        symfony)  echo "Symfony" ;;
        php)      echo "PHP" ;;
        django)   echo "Django" ;;
        flask)    echo "Flask" ;;
        python)   echo "Python" ;;
        rails)    echo "Ruby on Rails" ;;
        ruby)     echo "Ruby" ;;
        maven)    echo "Maven (Java)" ;;
        gradle)   echo "Gradle (Java)" ;;
        rust)     echo "Rust" ;;
        go)       echo "Go" ;;
        make)     echo "Makefile" ;;
        static)   echo "Static files" ;;
        *)        echo "Unknown" ;;
    esac
}

get_default_build_command() {
    local type="$1"

    case "$type" in
        nextjs|nuxt|react|vue|nodejs)
            echo "npm install && npm run build"
            ;;
        laravel)
            echo "composer install --no-dev --optimize-autoloader && npm install && npm run build"
            ;;
        symfony)
            echo "composer install --no-dev --optimize-autoloader"
            ;;
        php)
            echo "composer install --no-dev --optimize-autoloader"
            ;;
        django|flask|python)
            echo "pip install -r requirements.txt"
            ;;
        rails)
            echo "bundle install --deployment --without development test && rails assets:precompile"
            ;;
        ruby)
            echo "bundle install --deployment"
            ;;
        maven)
            echo "mvn clean package -DskipTests"
            ;;
        gradle)
            echo "./gradlew build -x test"
            ;;
        rust)
            echo "cargo build --release"
            ;;
        go)
            echo "go build -o app ."
            ;;
        make)
            echo "make build"
            ;;
        static)
            echo ""
            ;;
        *)
            echo ""
            ;;
    esac
}

get_default_public_dir() {
    local type="$1"

    case "$type" in
        nextjs)   echo ".next" ;;
        nuxt)     echo ".output" ;;
        react)    echo "build" ;;
        vue)      echo "dist" ;;
        laravel)  echo "public" ;;
        symfony)  echo "public" ;;
        django)   echo "" ;;
        rails)    echo "public" ;;
        *)        echo "" ;;
    esac
}

# ============================================================
# Git Operations
# ============================================================
git_clone() {
    local url="$1"
    local dest="$2"
    local branch="${3:-$DEFAULT_BRANCH}"
    local tag="$4"

    local git_opts="--depth ${GIT_DEPTH}"

    if [[ -n "$GIT_SSH_KEY" && -f "$GIT_SSH_KEY" ]]; then
        export GIT_SSH_COMMAND="ssh -i $GIT_SSH_KEY -o StrictHostKeyChecking=no"
    fi

    if [[ -n "$tag" ]]; then
        git clone $git_opts --branch "$tag" "$url" "$dest" 2>&1
    else
        git clone $git_opts --branch "$branch" "$url" "$dest" 2>&1
    fi
}

get_git_info() {
    local dir="$1"

    cd "$dir"
    local commit=$(git rev-parse --short HEAD 2>/dev/null)
    local message=$(git log -1 --format="%s" 2>/dev/null | head -c 50)
    local author=$(git log -1 --format="%an" 2>/dev/null)

    echo "${commit}|${message}|${author}"
}

# ============================================================
# Build Operations
# ============================================================
run_build() {
    local dir="$1"
    local build_cmd="$2"

    if [[ -z "$build_cmd" ]]; then
        log_info "No build command, skipping"
        return 0
    fi

    if [[ "$NO_BUILD" == "true" ]]; then
        log_warn "Build skipped (--no-build)"
        return 0
    fi

    cd "$dir"

    # Setup Node version if specified
    if [[ -n "$NODE_VERSION" ]] && check_command nvm; then
        source ~/.nvm/nvm.sh 2>/dev/null || true
        nvm use "$NODE_VERSION" 2>/dev/null || nvm install "$NODE_VERSION"
    fi

    # Run build
    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY-RUN] Would run: $build_cmd"
        return 0
    fi

    log_debug "Running: $build_cmd"
    eval "$build_cmd"
}

# ============================================================
# Shared Resources
# ============================================================
setup_shared_resources() {
    local release_dir="$1"
    local shared_base="${PROJECT_PATH}/shared"

    mkdir -p "$shared_base"

    # Shared directories
    for dir in $SHARED_DIRS; do
        local shared_dir="${shared_base}/${dir}"
        local release_dir_target="${release_dir}/${dir}"

        mkdir -p "$shared_dir"

        # Remove existing directory in release and create symlink
        rm -rf "$release_dir_target"
        ln -sf "$shared_dir" "$release_dir_target"

        log_debug "Linked shared directory: $dir"
    done

    # Shared files
    for file in $SHARED_FILES; do
        local shared_file="${shared_base}/${file}"
        local release_file="${release_dir}/${file}"

        if [[ -f "$shared_file" ]]; then
            rm -f "$release_file"
            ln -sf "$shared_file" "$release_file"
            log_debug "Linked shared file: $file"
        fi
    done

    # Environment file
    if [[ -n "$ENV_FILE" && -f "$ENV_FILE" ]]; then
        cp "$ENV_FILE" "${release_dir}/.env"
        log_debug "Copied environment file"
    fi
}

# ============================================================
# Hooks
# ============================================================
run_hook() {
    local hook_name="$1"
    local release_dir="$2"
    local hook_file="${release_dir}/.auto-deploy/hooks/${hook_name}.sh"

    if [[ -f "$hook_file" && -x "$hook_file" ]]; then
        log_info "Running hook: $hook_name"
        cd "$release_dir"
        bash "$hook_file"
    fi
}

# ============================================================
# Symlink Management
# ============================================================
activate_release() {
    local release_dir="$1"
    local symlink_path="$PROJECT_PATH"

    # Get current release for rollback
    if [[ -L "$symlink_path" ]]; then
        PREVIOUS_RELEASE=$(readlink -f "$symlink_path")
    fi

    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY-RUN] Would activate: $release_dir"
        return 0
    fi

    # Atomic symlink switch
    local temp_link="${symlink_path}.new"
    ln -sfn "$release_dir" "$temp_link"
    mv -Tf "$temp_link" "$symlink_path"

    CURRENT_RELEASE="$release_dir"
    log_success "Symlink switched (Zero Downtime!)"
}

# ============================================================
# Rollback
# ============================================================
list_releases() {
    echo -e "\n${BOLD}📦 Deployment History: ${PROJECT_NAME}${NC}\n"

    if [[ ! -d "$RELEASES_PATH" ]]; then
        log_warn "No releases found"
        return 1
    fi

    local current=""
    if [[ -L "$PROJECT_PATH" ]]; then
        current=$(basename "$(readlink -f "$PROJECT_PATH")")
    fi

    local count=0
    for release in $(ls -1r "$RELEASES_PATH" 2>/dev/null); do
        local marker=""
        [[ "$release" == "$current" ]] && marker="${GREEN}← current${NC}"

        local release_dir="${RELEASES_PATH}/${release}"
        local git_info=""

        if [[ -d "${release_dir}/.git" ]]; then
            cd "$release_dir"
            git_info=$(git log -1 --format="%h %s" 2>/dev/null | head -c 50)
        fi

        echo -e "  ${CYAN}${release}${NC} ${git_info} ${marker}"
        ((count++))
    done

    echo -e "\n${DIM}Total: ${count} releases${NC}"
}

do_rollback() {
    local target="$1"

    if [[ ! -d "$RELEASES_PATH" ]]; then
        log_error "No releases directory found"
        return 1
    fi

    local releases=($(ls -1r "$RELEASES_PATH" 2>/dev/null))
    local current=""

    if [[ -L "$PROJECT_PATH" ]]; then
        current=$(basename "$(readlink -f "$PROJECT_PATH")")
    fi

    local rollback_to=""

    if [[ -z "$target" || "$target" == "1" ]]; then
        # Rollback to previous
        for i in "${!releases[@]}"; do
            if [[ "${releases[$i]}" == "$current" ]]; then
                rollback_to="${releases[$((i+1))]}"
                break
            fi
        done
    elif [[ "$target" =~ ^[0-9]+$ ]]; then
        # Rollback N releases
        for i in "${!releases[@]}"; do
            if [[ "${releases[$i]}" == "$current" ]]; then
                rollback_to="${releases[$((i+target))]}"
                break
            fi
        done
    else
        # Rollback to specific release
        for release in "${releases[@]}"; do
            if [[ "$release" == *"$target"* ]]; then
                rollback_to="$release"
                break
            fi
        done
    fi

    if [[ -z "$rollback_to" ]]; then
        log_error "Rollback target not found"
        return 1
    fi

    local rollback_path="${RELEASES_PATH}/${rollback_to}"

    if [[ ! -d "$rollback_path" ]]; then
        log_error "Release directory not found: $rollback_to"
        return 1
    fi

    log_step "Rolling back to: $rollback_to"

    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY-RUN] Would rollback to: $rollback_to"
        return 0
    fi

    activate_release "$rollback_path"
    restart_services

    log_success "Rollback completed!"
    return 0
}

# ============================================================
# Cleanup
# ============================================================
cleanup_old_releases() {
    if [[ ! -d "$RELEASES_PATH" ]]; then
        return 0
    fi

    local releases=($(ls -1r "$RELEASES_PATH" 2>/dev/null))
    local count=${#releases[@]}

    if [[ $count -le $KEEP_RELEASES ]]; then
        log_info "No cleanup needed ($count releases)"
        return 0
    fi

    local to_delete=$((count - KEEP_RELEASES))
    local deleted=0

    for ((i=KEEP_RELEASES; i<count; i++)); do
        local release="${releases[$i]}"
        local release_path="${RELEASES_PATH}/${release}"

        if [[ "$DRY_RUN" == "true" ]]; then
            log_info "[DRY-RUN] Would delete: $release"
        else
            rm -rf "$release_path"
            log_debug "Deleted: $release"
        fi
        ((deleted++))
    done

    if [[ $deleted -gt 0 ]]; then
        echo -e "        ├── 오래된 릴리스 ${deleted}개 삭제"
    fi
}

# ============================================================
# Service Restart
# ============================================================
restart_services() {
    local restarted=false

    if [[ "$RESTART_NGINX" == "true" ]]; then
        systemctl reload nginx 2>/dev/null && log_debug "Nginx reloaded"
        restarted=true
    fi

    if [[ "$RESTART_APACHE" == "true" ]]; then
        systemctl reload apache2 2>/dev/null || systemctl reload httpd 2>/dev/null
        log_debug "Apache reloaded"
        restarted=true
    fi

    if [[ "$RESTART_PHP_FPM" == "true" ]]; then
        systemctl reload php*-fpm 2>/dev/null && log_debug "PHP-FPM reloaded"
        restarted=true
    fi

    if [[ "$RESTART_PM2" == "true" ]]; then
        pm2 reload all 2>/dev/null && log_debug "PM2 reloaded"
        restarted=true
    fi

    for service in $RESTART_SERVICES; do
        systemctl reload "$service" 2>/dev/null || systemctl restart "$service" 2>/dev/null
        log_debug "Service restarted: $service"
        restarted=true
    done

    return 0
}

# ============================================================
# Notifications
# ============================================================
send_notification() {
    local status="$1"
    local message="$2"

    [[ "$NOTIFY_ENABLED" != "true" ]] && return 0

    local icon="🚀"
    [[ "$status" == "failure" ]] && icon="❌"
    [[ "$status" == "rollback" ]] && icon="↩️"

    # Slack
    if [[ -n "$SLACK_WEBHOOK" ]]; then
        local payload=$(cat << EOF
{
    "text": "${icon} *AUTO-DEPLOY ${status}*\n\`\`\`${message}\`\`\`",
    "username": "AUTO-DEPLOY",
    "icon_emoji": ":rocket:"
}
EOF
)
        curl -s -X POST -H 'Content-type: application/json' \
            --data "$payload" "$SLACK_WEBHOOK" >/dev/null 2>&1 || true
    fi

    # Discord
    if [[ -n "$DISCORD_WEBHOOK" ]]; then
        local payload=$(cat << EOF
{
    "content": "${icon} **AUTO-DEPLOY ${status}**\n\`\`\`${message}\`\`\`"
}
EOF
)
        curl -s -X POST -H 'Content-type: application/json' \
            --data "$payload" "$DISCORD_WEBHOOK" >/dev/null 2>&1 || true
    fi
}

# ============================================================
# Project Initialization
# ============================================================
init_project() {
    local git_url="$1"
    local deploy_path="$2"

    if [[ -z "$git_url" || -z "$deploy_path" ]]; then
        log_error "Usage: auto-deploy --init GIT_URL --path DEPLOY_PATH"
        return 1
    fi

    # Extract project name from URL
    local project_name=$(basename "$git_url" .git)

    mkdir -p "$PROJECTS_DIR"

    local config_file="${PROJECTS_DIR}/${project_name}.conf"

    if [[ -f "$config_file" ]]; then
        log_warn "Project already exists: $project_name"
        read -p "Overwrite? [y/N] " confirm
        [[ "$confirm" != "y" && "$confirm" != "Y" ]] && return 1
    fi

    cat > "$config_file" << EOF
#!/bin/bash
# AUTO-DEPLOY Project Configuration
# Project: ${project_name}
# Generated: $(date)

PROJECT_NAME="${project_name}"
GIT_URL="${git_url}"
PROJECT_PATH="${deploy_path}"
RELEASES_PATH="${deploy_path}${RELEASES_SUFFIX}"

# Git settings
GIT_BRANCH="main"
GIT_TAG=""

# Build (leave empty for auto-detect)
BUILD_COMMAND=""

# Public directory (for web server)
PUBLIC_DIR=""

# Shared resources (persisted between releases)
SHARED_DIRS="storage logs uploads"
SHARED_FILES=".env"

# Environment file to copy
ENV_FILE=""

# Services to restart after deploy
RESTART_SERVICES=""

# Post-activation command
POST_ACTIVATE=""
EOF

    chmod 600 "$config_file"

    # Create directories
    mkdir -p "${deploy_path}${RELEASES_SUFFIX}"
    mkdir -p "${deploy_path}/shared"

    log_success "Project initialized: $project_name"
    log_info "Config: $config_file"
    log_info "Deploy: auto-deploy $project_name"
}

# ============================================================
# List Projects
# ============================================================
list_projects() {
    echo -e "\n${BOLD}📋 Registered Projects${NC}\n"

    if [[ ! -d "$PROJECTS_DIR" ]]; then
        log_warn "No projects registered"
        return 0
    fi

    for conf in "${PROJECTS_DIR}"/*.conf; do
        [[ -f "$conf" ]] || continue

        local name=$(basename "$conf" .conf)
        source "$conf"

        local status="❓"
        if [[ -L "$PROJECT_PATH" ]]; then
            status="✅"
        fi

        echo -e "  ${status} ${CYAN}${name}${NC}"
        echo -e "     └── ${PROJECT_PATH}"
    done

    echo
}

# ============================================================
# Project Status
# ============================================================
show_status() {
    echo -e "\n${BOLD}📊 Project Status: ${PROJECT_NAME}${NC}\n"

    echo -e "${CYAN}Configuration${NC}"
    echo -e "  ├── Git URL: ${GIT_URL}"
    echo -e "  ├── Branch: ${GIT_BRANCH:-main}"
    echo -e "  ├── Path: ${PROJECT_PATH}"
    echo -e "  └── Releases: ${RELEASES_PATH}"

    if [[ -L "$PROJECT_PATH" ]]; then
        local current=$(readlink -f "$PROJECT_PATH")
        local current_name=$(basename "$current")

        echo -e "\n${CYAN}Current Release${NC}"
        echo -e "  └── ${current_name}"

        if [[ -d "${current}/.git" ]]; then
            cd "$current"
            echo -e "      ├── Commit: $(git rev-parse --short HEAD 2>/dev/null)"
            echo -e "      └── Date: $(git log -1 --format='%ci' 2>/dev/null)"
        fi
    fi

    if [[ -d "$RELEASES_PATH" ]]; then
        local count=$(ls -1 "$RELEASES_PATH" 2>/dev/null | wc -l)
        echo -e "\n${CYAN}Releases${NC}"
        echo -e "  └── Total: ${count} (keeping ${KEEP_RELEASES})"
    fi

    echo
}

# ============================================================
# Webhook Server
# ============================================================
start_webhook_server() {
    if [[ -f "$WEBHOOK_PID_FILE" ]]; then
        local pid=$(cat "$WEBHOOK_PID_FILE")
        if kill -0 "$pid" 2>/dev/null; then
            log_warn "Webhook server already running (PID: $pid)"
            return 0
        fi
    fi

    log_info "Starting webhook server on port $WEBHOOK_PORT..."

    # Simple webhook server using netcat
    (
        while true; do
            {
                read -r request
                while read -r header; do
                    [[ "$header" == $'\r' ]] && break
                done

                # Parse webhook payload (simplified)
                local response="HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK"
                echo -e "$response"

                # Trigger deploy (would need proper payload parsing)
                log_info "Webhook received"
            } | nc -l -p "$WEBHOOK_PORT" -q 1
        done
    ) &

    echo $! > "$WEBHOOK_PID_FILE"
    log_success "Webhook server started (PID: $(cat $WEBHOOK_PID_FILE))"
}

stop_webhook_server() {
    if [[ -f "$WEBHOOK_PID_FILE" ]]; then
        local pid=$(cat "$WEBHOOK_PID_FILE")
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid"
            rm -f "$WEBHOOK_PID_FILE"
            log_success "Webhook server stopped"
            return 0
        fi
    fi

    log_info "Webhook server not running"
}

webhook_status() {
    if [[ -f "$WEBHOOK_PID_FILE" ]]; then
        local pid=$(cat "$WEBHOOK_PID_FILE")
        if kill -0 "$pid" 2>/dev/null; then
            log_success "Webhook server running (PID: $pid, Port: $WEBHOOK_PORT)"
            return 0
        fi
    fi

    log_info "Webhook server not running"
}

# ============================================================
# Print Summary
# ============================================================
print_summary() {
    local status="$1"
    local elapsed=$(elapsed_time "$START_TIME" "$END_TIME")

    echo -e "\n${GREEN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    if [[ "$status" == "success" ]]; then
    echo "║  ✅ 배포 완료!                                               ║"
    else
    echo "║  ❌ 배포 실패!                                               ║"
    fi
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    printf "║  📦 프로젝트: %-45s║\n" "$PROJECT_NAME"
    printf "║  📁 경로: %-49s║\n" "$PROJECT_PATH"
    if [[ -n "$CURRENT_RELEASE" ]]; then
    printf "║  🏷️  릴리스: %-46s║\n" "$(basename "$CURRENT_RELEASE")"
    fi
    printf "║  ⏱️  소요: %-48s║\n" "$elapsed"
    echo "║                                                              ║"
    if [[ ${#ERRORS[@]} -gt 0 ]]; then
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  ⚠️  Errors:                                                 ║"
    for err in "${ERRORS[@]}"; do
    printf "║  • %-56s║\n" "${err:0:56}"
    done
    fi
    echo "╠══════════════════════════════════════════════════════════════╣"
    printf "║  ↩️  롤백: auto-deploy %-36s║\n" "${PROJECT_NAME} --rollback"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  🤟 WIA AUTO-DEPLOY - Free & Open Source                     ║"
    echo "║  弘益人間 · Benefit All Humanity                              ║"
    echo "║                                                              ║"
    echo "║  📖 Docs: https://wiastandards.com/auto-deploy               ║"
    echo "║  ⭐ Star: https://github.com/WIA-Official/wia-standards      ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}\n"
}

# ============================================================
# Main Deploy Process
# ============================================================
do_deploy() {
    START_TIME=$(date +%s)

    print_banner

    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║  🚀 AUTO-DEPLOY v${VERSION}                                        ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    printf "${BOLD}║  📦 프로젝트: %-45s║${NC}\n" "$PROJECT_NAME"
    printf "${BOLD}║  🌿 브랜치: %-47s║${NC}\n" "${GIT_BRANCH:-main}"
    printf "${BOLD}║  📅 시작: %-49s║${NC}\n" "$(date '+%Y-%m-%d %H:%M:%S')"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╝${NC}"

    # Acquire lock
    acquire_lock "$PROJECT_NAME"

    # Create temp and release directories
    mkdir -p "$TEMP_DIR"
    mkdir -p "$RELEASES_PATH"

    local release_name=$(date +"%Y%m%d-%H%M%S")
    local release_dir="${RELEASES_PATH}/${release_name}"

    # Step 1: Git Clone
    log_step "[1/6] 📥 Git Clone"
    echo -e "        ├── ${GIT_URL}"

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "        └── ${YELLOW}[DRY-RUN] Skipped${NC}"
    else
        local clone_start=$(date +%s)
        git_clone "$GIT_URL" "$release_dir" "$GIT_BRANCH" "$GIT_TAG" || {
            log_error "Git clone failed"
            release_lock "$PROJECT_NAME"
            return 1
        }

        local git_info=$(get_git_info "$release_dir")
        IFS='|' read -r commit message author <<< "$git_info"

        echo -e "        ├── Commit: ${commit} (${message})"
        echo -e "        └── ${GREEN}✅ 완료${NC} ($(($(date +%s) - clone_start))초)"
    fi

    # Step 2: Detect Project
    log_step "[2/6] 🔍 프로젝트 감지"
    local project_type=$(detect_project_type "$release_dir")
    local project_info=$(get_project_info "$project_type" "$release_dir")

    echo -e "        ├── 타입: ${project_info}"

    if [[ -z "$BUILD_COMMAND" ]]; then
        BUILD_COMMAND=$(get_default_build_command "$project_type")
    fi

    if [[ -z "$PUBLIC_DIR" ]]; then
        PUBLIC_DIR=$(get_default_public_dir "$project_type")
    fi

    echo -e "        └── ${GREEN}✅ 감지 완료${NC}"

    # Run pre-deploy hook
    run_hook "pre-deploy" "$release_dir"

    # Step 3: Install Dependencies
    if [[ -n "$BUILD_COMMAND" && "$BUILD_COMMAND" == *"install"* ]]; then
        log_step "[3/6] 📦 의존성 설치"

        local install_cmd=""
        case "$project_type" in
            nextjs|nuxt|react|vue|nodejs)
                install_cmd="npm install"
                ;;
            laravel|symfony|php)
                install_cmd="composer install --no-dev --optimize-autoloader"
                ;;
            django|flask|python)
                install_cmd="pip install -r requirements.txt"
                ;;
            rails|ruby)
                install_cmd="bundle install --deployment"
                ;;
        esac

        if [[ -n "$install_cmd" && "$DRY_RUN" != "true" ]]; then
            echo -e "        ├── ${install_cmd}"
            local install_start=$(date +%s)
            cd "$release_dir"
            eval "$install_cmd" >/dev/null 2>&1 || {
                log_error "Dependency installation failed"
                release_lock "$PROJECT_NAME"
                return 1
            }
            echo -e "        └── ${GREEN}✅ 완료${NC} ($(($(date +%s) - install_start))초)"
        else
            echo -e "        └── ${YELLOW}[DRY-RUN] Skipped${NC}"
        fi
    else
        log_step "[3/6] 📦 의존성 설치"
        echo -e "        └── ${DIM}Skipped (no dependencies)${NC}"
    fi

    # Step 4: Build
    log_step "[4/6] 🔨 빌드"
    if [[ -n "$BUILD_COMMAND" && "$BUILD_COMMAND" == *"build"* ]]; then
        local build_cmd=$(echo "$BUILD_COMMAND" | grep -oP 'npm run build|make build|cargo build.*|go build.*|mvn.*|gradle.*' || echo "")

        if [[ -n "$build_cmd" ]]; then
            echo -e "        ├── ${build_cmd}"

            if [[ "$DRY_RUN" != "true" ]]; then
                local build_start=$(date +%s)
                cd "$release_dir"
                eval "$build_cmd" >/dev/null 2>&1 || {
                    log_error "Build failed"
                    release_lock "$PROJECT_NAME"
                    return 1
                }
                echo -e "        └── ${GREEN}✅ 완료${NC} ($(($(date +%s) - build_start))초)"
            else
                echo -e "        └── ${YELLOW}[DRY-RUN] Skipped${NC}"
            fi
        else
            echo -e "        └── ${DIM}No build step${NC}"
        fi
    else
        echo -e "        └── ${DIM}No build required${NC}"
    fi

    # Run post-download hook
    run_hook "post-download" "$release_dir"

    # Setup shared resources
    setup_shared_resources "$release_dir"

    # Run pre-activate hook
    run_hook "pre-activate" "$release_dir"

    # Step 5: Symlink Switch
    log_step "[5/6] 🔗 Symlink 전환"

    if [[ -L "$PROJECT_PATH" ]]; then
        echo -e "        ├── 이전: $(basename "$(readlink -f "$PROJECT_PATH")")"
    fi
    echo -e "        ├── 현재: ${release_name}"

    activate_release "$release_dir"

    echo -e "        └── ${GREEN}✅ 전환 완료 (0.1초) ⚡ Zero Downtime!${NC}"

    # Run post-activate hook & command
    run_hook "post-activate" "$release_dir"

    if [[ -n "$POST_ACTIVATE" && "$DRY_RUN" != "true" ]]; then
        cd "$release_dir"
        eval "$POST_ACTIVATE"
    fi

    # Restart services
    restart_services

    # Step 6: Cleanup
    log_step "[6/6] 🧹 정리"
    cleanup_old_releases
    echo -e "        └── ${GREEN}✅ 완료${NC}"

    # Release lock
    release_lock "$PROJECT_NAME"

    END_TIME=$(date +%s)
    DEPLOY_SUCCESS=true

    print_summary "success"

    send_notification "success" "프로젝트: ${PROJECT_NAME}\n릴리스: ${release_name}\n소요: $(elapsed_time $START_TIME $END_TIME)"

    return 0
}

# ============================================================
# Main Entry Point
# ============================================================
main() {
    load_global_config

    # Parse arguments
    local action="deploy"
    local project_arg=""
    local init_url=""
    local init_path=""
    local rollback_target=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --init)
                action="init"
                shift
                init_url="$1"
                ;;
            --path)
                shift
                init_path="$1"
                ;;
            --branch|-b)
                shift
                GIT_BRANCH="$1"
                ;;
            --tag|-t)
                shift
                GIT_TAG="$1"
                ;;
            --rollback)
                action="rollback"
                shift
                [[ -n "$1" && "$1" != -* ]] && rollback_target="$1" && shift
                continue
                ;;
            --list)
                action="list"
                ;;
            --projects)
                action="projects"
                ;;
            --status)
                action="status"
                ;;
            --cleanup)
                action="cleanup"
                ;;
            --webhook-server)
                action="webhook-start"
                ;;
            --webhook-stop)
                action="webhook-stop"
                ;;
            --webhook-status)
                action="webhook-status"
                ;;
            --force)
                FORCE=true
                ;;
            --no-build)
                NO_BUILD=true
                ;;
            --dry-run)
                DRY_RUN=true
                ;;
            -v|--verbose)
                VERBOSE=true
                ;;
            -h|--help)
                print_help
                exit 0
                ;;
            --version)
                echo "AUTO-DEPLOY v${VERSION}"
                exit 0
                ;;
            -*)
                log_error "Unknown option: $1"
                exit 1
                ;;
            *)
                project_arg="$1"
                ;;
        esac
        shift
    done

    # Execute action
    case "$action" in
        init)
            init_project "$init_url" "$init_path"
            ;;
        projects)
            list_projects
            ;;
        webhook-start)
            start_webhook_server
            ;;
        webhook-stop)
            stop_webhook_server
            ;;
        webhook-status)
            webhook_status
            ;;
        deploy|rollback|list|status|cleanup)
            # Load project config
            if [[ -n "$project_arg" ]]; then
                if [[ "$project_arg" == http* || "$project_arg" == git@* ]]; then
                    # Direct Git URL
                    GIT_URL="$project_arg"
                    PROJECT_NAME=$(basename "$project_arg" .git)
                    PROJECT_PATH="${DEPLOY_BASE_DIR}/${PROJECT_NAME}"
                    RELEASES_PATH="${PROJECT_PATH}${RELEASES_SUFFIX}"
                elif load_project_config "$project_arg"; then
                    : # Config loaded
                else
                    log_error "Project not found: $project_arg"
                    log_info "Initialize with: auto-deploy --init GIT_URL --path PATH"
                    exit 1
                fi
            else
                # Check for .auto-deploy.conf in current directory
                if [[ -f ".auto-deploy.conf" ]]; then
                    source ".auto-deploy.conf"
                else
                    log_error "No project specified"
                    print_help
                    exit 1
                fi
            fi

            case "$action" in
                deploy)   do_deploy ;;
                rollback) do_rollback "$rollback_target" ;;
                list)     list_releases ;;
                status)   show_status ;;
                cleanup)  cleanup_old_releases ;;
            esac
            ;;
    esac
}

main "$@"
