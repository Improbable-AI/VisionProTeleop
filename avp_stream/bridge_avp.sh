#!/bin/bash
set -e

echo "🔧 Vision Pro High-Speed Bridge Setup"
echo "======================================"
echo ""
echo "⚠️  IMPORTANT: Plug in your Vision Pro BEFORE running this script!"
echo ""
read -p "Is your Vision Pro plugged in? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Please plug in your Vision Pro, wait 5 seconds, then run this script again."
    exit 0
fi

echo ""
echo "Detecting configuration..."

# Detect primary network interface
PRIMARY_INTERFACE=$(route -n get default 2>/dev/null | grep interface | awk '{print $2}')

if [ -z "$PRIMARY_INTERFACE" ]; then
    echo "❌ Error: Could not detect primary network interface"
    exit 1
fi

# Get network info
NETWORK_INFO=$(ifconfig "$PRIMARY_INTERFACE" | grep "inet " | awk '{print $2, $4}')
if [ -z "$NETWORK_INFO" ]; then
    echo "❌ Error: No IP address on $PRIMARY_INTERFACE"
    exit 1
fi

IP_ADDR=$(echo $NETWORK_INFO | awk '{print $1}')
NETWORK_BASE=$(echo $IP_ADDR | cut -d. -f1-3)
BRIDGE_IP="${NETWORK_BASE}.200"

echo ""
echo "Configuration:"
echo "  Primary interface: $PRIMARY_INTERFACE"
echo "  Your Mac's IP: $IP_ADDR"
echo "  Network: ${NETWORK_BASE}.0/24"
echo "  Bridge IP: $BRIDGE_IP"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 0
fi

echo ""
echo "Setting up bridge..."

# 1. Enable IP forwarding
echo "  • Enabling IP forwarding..."
sudo sysctl -w net.inet.ip.forwarding=1 > /dev/null

# 2. Configure bridge0
echo "  • Configuring bridge0..."
sudo ifconfig bridge0 inet $BRIDGE_IP netmask 255.255.255.0
sleep 1

# 3. Add primary interface to bridge0
echo "  • Adding $PRIMARY_INTERFACE to bridge0..."
sudo ifconfig bridge0 addm $PRIMARY_INTERFACE 2>/dev/null || true
sleep 2

# 4. Set up NAT
echo "  • Configuring NAT..."
sudo pfctl -e 2>/dev/null || true
echo "nat on $PRIMARY_INTERFACE inet from bridge0:network to any -> ($PRIMARY_INTERFACE)" | sudo pfctl -f - 2>/dev/null
sleep 2

echo ""
echo "✅ Setup complete!"
echo ""
echo "⏳ Network should be active in ~5 seconds..."
echo ""
echo "Your Vision Pro should now have:"
echo "  • ~10 Gbps local network speeds (test with iperf)"
echo "  • Full internet access"
echo ""
echo "🧪 To test internet: Try loading a webpage on Vision Pro"
echo "🧹 To cleanup when done: Run ~/vision-pro-cleanup.sh"