#!/bin/bash

# setup_aws_ecr.sh
# Automates AWS CLI installation and ECR credential configuration for the ETHRC Drone Racing team.

set -e

# --- Configuration (Temporary Read-Only Keys) ---
DEFAULT_ACCESS_KEY="[AWS ACCESS KEY]"
DEFAULT_SECRET_KEY="[AWS SECRET ACCESS KEY]"
DEFAULT_REGION="us-east-2"
DEFAULT_FORMAT="json"
REGISTRY="916780037007.dkr.ecr.us-east-2.amazonaws.com"

echo "--------------------------------------------------------"
echo "🚀 ETHRC Drone Racing - AWS & ECR Setup"
echo "--------------------------------------------------------"

# 1. Install Dependencies
echo "[1/4] Checking dependencies..."
for cmd in curl unzip docker make; do
    if ! command -v "$cmd" &> /dev/null; then
        echo "Error: $cmd is not installed. Please install it with: sudo apt install $cmd"
        exit 1
    fi
done

# 2. Install AWS CLI v2 if missing
if ! command -v aws &> /dev/null; then
    echo "[2/4] AWS CLI not found. Installing..."
    
    # Detect architecture
    ARCH=$(uname -m)
    case $ARCH in
        x86_64) AWS_ARCH="x86_64" ;;
        aarch64) AWS_ARCH="aarch64" ;;
        *)
            echo "Error: Unsupported architecture $ARCH"
            exit 1
            ;;
    esac

    echo "Detected architecture: $ARCH"
    
    TEMP_DIR=$(mktemp -d)
    cd "$TEMP_DIR"
    curl "https://awscli.amazonaws.com/awscli-exe-linux-${AWS_ARCH}.zip" -o "awscliv2.zip"
    unzip -q awscliv2.zip
    sudo ./aws/install
    cd - > /dev/null
    rm -rf "$TEMP_DIR"
    echo "✅ AWS CLI installed successfully."
else
    echo "[2/4] AWS CLI is already installed ($(aws --version | awk '{print $1}'))"
fi

# 3. Configure Credentials
echo "[3/4] Configuring AWS credentials..."

# Check if user wants to provide custom keys
read -p "Use temporary project read-only keys? (Y/n): " use_temp
use_temp=${use_temp:-Y}

if [[ $use_temp =~ ^[Yy]$ ]]; then
    ACCESS_KEY=$DEFAULT_ACCESS_KEY
    SECRET_KEY=$DEFAULT_SECRET_KEY
else
    read -p "Enter AWS Access Key ID: " ACCESS_KEY
    read -p "Enter AWS Secret Access Key: " SECRET_KEY
fi

aws configure set aws_access_key_id "$ACCESS_KEY"
aws configure set aws_secret_access_key "$SECRET_KEY"
aws configure set region "$DEFAULT_REGION"
aws configure set output "$DEFAULT_FORMAT"

echo "✅ AWS configuration updated in ~/.aws/config and ~/.aws/credentials"

# 4. Verify & Login to ECR
echo "[4/4] Authenticating Docker to ECR..."
if [ -f "Makefile" ]; then
    make login
else
    aws ecr get-login-password --region "$DEFAULT_REGION" | docker login --username AWS --password-stdin "$REGISTRY"
fi

echo "--------------------------------------------------------"
echo "✅ Setup Complete!"
echo "--------------------------------------------------------"
echo "💡 You can now pull images directly from ECR."
echo ""
echo "To use pre-built images with Docker Compose without a local build, use the 'Tag Trick':"
echo "  1. make pull REPO=aeronautics/droneracing/main/dev TAG=latest"
echo "  2. docker tag $REGISTRY/aeronautics/droneracing/main/dev:latest ethrc-drone-racing-dev:latest"
echo "  3. docker compose up -d dev"
echo "--------------------------------------------------------"
