# WIA-AUTO-SECURE

> One-Click HTTPS Installation
>
> 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

## What is this?

HTTP → HTTPS in **one command**. No configuration needed.

```
Before: http://your-site.com  (insecure)
After:  https://your-site.com (secure, auto-renewing)
```

## Quick Install

### Linux (Ubuntu, Debian, CentOS, Amazon Linux, RHEL, Arch)

```bash
# Option 1: One-liner (coming soon)
curl -sSL https://wia.family/secure | bash

# Option 2: Download and run
wget https://raw.githubusercontent.com/WIA-Official/wia-standards/main/auto-secure/linux/wia-secure-install.sh
chmod +x wia-secure-install.sh
./wia-secure-install.sh
```

### What it does

1. **Detects** your OS (Ubuntu, CentOS, Amazon Linux, etc.)
2. **Detects** your web server (nginx, Apache, or standalone)
3. **Installs** certbot automatically
4. **Issues** SSL certificate from Let's Encrypt
5. **Configures** your web server for HTTPS
6. **Sets up** auto-renewal (every 60-90 days)

Total time: ~2 minutes

## Supported Systems

| OS | Package Manager | Status |
|----|-----------------|--------|
| Ubuntu 18.04+ | apt | ✓ |
| Debian 10+ | apt | ✓ |
| CentOS 7+ | yum | ✓ |
| RHEL 7+ | yum | ✓ |
| Amazon Linux 2023 | dnf/yum | ✓ |
| Fedora | dnf | ✓ |
| Arch Linux | pacman | ✓ |
| openSUSE | zypper | ✓ |

| Web Server | Status |
|------------|--------|
| nginx | ✓ Auto-configured |
| Apache/httpd | ✓ Auto-configured |
| None (standalone) | ✓ Certificate only |

## Requirements

- Linux server with root/sudo access
- Domain pointing to your server (A record)
- Port 80 open (for Let's Encrypt verification)
- Port 443 open (for HTTPS)

## Usage

### Basic Usage

```bash
./wia-secure-install.sh
# Then enter your domain when prompted
```

### With Domain Argument

```bash
./wia-secure-install.sh example.com
```

### Non-Interactive (CI/CD)

```bash
export WIA_DOMAIN="example.com"
export WIA_EMAIL="admin@example.com"
./wia-secure-install.sh --non-interactive
```

## What Happens After Installation?

1. **Your site is HTTPS** - immediately accessible via https://
2. **HTTP redirects to HTTPS** - automatic redirect configured
3. **Auto-renewal** - certificate renews automatically before expiry
4. **No maintenance needed** - set it and forget it

## Certificate Location

```
/etc/letsencrypt/live/your-domain.com/
├── cert.pem       # Your certificate
├── chain.pem      # Intermediate certificates
├── fullchain.pem  # cert.pem + chain.pem
└── privkey.pem    # Private key
```

## Manual Renewal (if needed)

```bash
# Test renewal
sudo certbot renew --dry-run

# Force renewal
sudo certbot renew --force-renewal
```

## Troubleshooting

### "Domain not pointing to this server"

Make sure your DNS A record points to your server's IP:

```bash
# Check DNS
dig +short your-domain.com

# Should return your server IP
```

### "Port 80 is in use"

Stop the service using port 80 temporarily:

```bash
sudo systemctl stop nginx  # or apache2/httpd
./wia-secure-install.sh
sudo systemctl start nginx
```

### "Certificate limit reached"

Let's Encrypt has rate limits (50 certs/domain/week). Wait and retry.

## Coming Soon

| Platform | Status |
|----------|--------|
| Windows (.exe) | Planned |
| macOS (.pkg) | Planned |
| Docker | Planned |

## Part of WIA Standards

This is part of the WIA security standards suite:

| Standard | Description | Status |
|----------|-------------|--------|
| **WIA-AUTO-SECURE** | One-click HTTPS | ✓ Ready |
| WIA-TLS-LITE | Lightweight TLS for IoT | In Progress |
| WIA-DPKI | Decentralized PKI | Planned |
| WIA-PQ-CRYPTO | Post-Quantum Cryptography | Planned |

## License

MIT License - Free for everyone, forever.

## Links

- Website: https://wia.family
- GitHub: https://github.com/WIA-Official/wia-standards
- Issues: https://github.com/WIA-Official/wia-standards/issues

---

**World Certification Industry Association**

홍익인간 (弘益人間) - Benefit All Humanity
