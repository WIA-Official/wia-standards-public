#!/bin/bash

# WIA-CHILD-001: Online Safety Standard - Installation Script
# 弘益人間 - Benefit All Humanity
# © 2025 SmileStory Inc. / WIA

set -e

echo "=================================="
echo "WIA-CHILD-001: Online Safety"
echo "Child Protection Standard"
echo "弘益人間 - Benefit All Humanity"
echo "=================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root or with sudo"
    exit 1
fi

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    OS="windows"
else
    echo "Unsupported OS: $OSTYPE"
    exit 1
fi

echo "Detected OS: $OS"
echo ""

# Install dependencies
echo "Installing dependencies..."
if [ "$OS" == "linux" ]; then
    apt-get update -qq
    apt-get install -y curl git nodejs npm python3 python3-pip
elif [ "$OS" == "macos" ]; then
    if ! command -v brew &> /dev/null; then
        echo "Homebrew not found. Installing..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    fi
    brew install node python3
fi

# Install TypeScript SDK
echo ""
echo "Installing TypeScript SDK..."
cd api/typescript
npm install
npm run build
npm link
cd ../..

# Install CLI tools
echo ""
echo "Installing CLI tools..."
cp cli/wia-child-001.sh /usr/local/bin/wia-child-001
chmod +x /usr/local/bin/wia-child-001

# Create config directory
echo ""
echo "Creating configuration directory..."
mkdir -p /etc/wia/child-001
mkdir -p /var/log/wia/child-001
mkdir -p ~/.wia/child-001

# Set permissions
chmod 755 /usr/local/bin/wia-child-001
chmod 755 /etc/wia/child-001
chmod 755 /var/log/wia/child-001

# Create default config
cat > /etc/wia/child-001/config.json <<EOF
{
  "version": "1.0.0",
  "environment": "production",
  "apiEndpoint": "https://api.wia-standards.org/child-001",
  "monitoring": {
    "enabled": true,
    "realTime": true,
    "logLevel": "info"
  },
  "privacy": {
    "mode": "strict",
    "dataRetention": 0,
    "encryption": "AES-256"
  }
}
EOF

echo ""
echo "=================================="
echo "Installation Complete!"
echo "=================================="
echo ""
echo "Next steps:"
echo "1. Configure your API key: wia-child-001 config --api-key YOUR_KEY"
echo "2. Create a child profile: wia-child-001 profile create"
echo "3. Start monitoring: wia-child-001 monitor start"
echo ""
echo "Documentation: file://$(pwd)/ebook/en/index.html"
echo "Simulator: file://$(pwd)/simulator/index.html"
echo ""
echo "For support: child-safety@wia-standards.org"
echo "Emergency: +1-800-CHILD-SAFE"
echo ""
echo "弘益人間 - Protecting children in the digital age"
echo "© 2025 SmileStory Inc. / WIA"
echo ""
