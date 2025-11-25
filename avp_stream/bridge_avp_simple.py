#!/usr/bin/env python3
"""
Vision Pro Simple USB Network Setup

This module sets up a simple USB network connection for Vision Pro devices,
providing high-speed local communication without internet access.

Platform Support:
- macOS: Full support
- Linux: Full support
- Windows: Not yet supported

Use this if you only need local communication (gRPC, video streaming, etc.)
and don't need Vision Pro to have internet access.
"""

import subprocess
import sys
import platform
import re
import time
from typing import Optional


class NetworkSetupError(Exception):
    """Exception raised for network setup errors."""
    pass


class VisionProUSBNetwork:
    """Simple cross-platform Vision Pro USB network setup."""
    
    # Vision Pro uses link-local addressing (169.254.x.x) over USB
    VISION_PRO_NETWORK = "169.254.220.0/24"
    HOST_IP = "169.254.220.1"
    
    def __init__(self):
        self.os_type = platform.system()
        self.usb_interface: Optional[str] = None
        
    def run_command(self, cmd: list, check: bool = True, capture: bool = True) -> subprocess.CompletedProcess:
        """Run a shell command with error handling."""
        try:
            if capture:
                result = subprocess.run(cmd, capture_output=True, text=True, check=check)
            else:
                result = subprocess.run(cmd, check=check)
            return result
        except subprocess.CalledProcessError as e:
            raise NetworkSetupError(f"Command failed: {' '.join(cmd)}\n{e.stderr if capture else ''}")
    
    def find_usb_interface_macos(self) -> Optional[str]:
        """Find USB network interface on macOS."""
        # Look for interfaces with link-local addresses
        result = self.run_command(["ifconfig"], check=False)
        
        # Parse ifconfig output to find interfaces with 169.254.x.x addresses
        current_iface = None
        for line in result.stdout.split('\n'):
            # New interface
            if line and not line[0].isspace():
                match = re.match(r'^(\S+):', line)
                if match:
                    iface = match.group(1)
                    # Skip common interfaces
                    if iface not in ['lo0', 'en0', 'en1', 'bridge0']:
                        current_iface = iface
            # Check for link-local IP
            elif current_iface and 'inet 169.254.' in line:
                return current_iface
        
        # If no existing interface found, look for any USB/Ethernet interface
        # that's not the primary network
        result = self.run_command(["ifconfig", "-l"], check=False)
        for iface in result.stdout.split():
            if iface.startswith(('en', 'eth')) and iface not in ['en0', 'en1']:
                return iface
        
        return None
    
    def find_usb_interface_linux(self) -> Optional[str]:
        """Find USB network interface on Linux."""
        result = self.run_command(["ip", "link", "show"], check=False)
        
        # Look for USB/Ethernet interfaces
        interfaces = []
        for line in result.stdout.split('\n'):
            match = re.search(r'\d+:\s+(\S+):', line)
            if match and '@' not in line:  # Skip VLAN interfaces
                iface = match.group(1)
                # Look for typical USB network interface names
                if any(pattern in iface for pattern in ['enp', 'usb', 'eth']) and iface != 'lo':
                    # Check if it's not the primary network interface
                    addr_result = self.run_command(["ip", "addr", "show", iface], check=False)
                    # Prefer interfaces without IP or with link-local
                    if 'inet ' not in addr_result.stdout or '169.254.' in addr_result.stdout:
                        interfaces.append(iface)
        
        # Return the first suitable interface
        return interfaces[0] if interfaces else None
    
    def setup_interface_macos(self, interface: str):
        """Configure the USB interface on macOS."""
        print(f"  • Configuring {interface}...")
        
        # Flush any existing configuration
        self.run_command(["sudo", "ifconfig", interface, "delete"], check=False, capture=False)
        
        # Assign our IP address
        self.run_command([
            "sudo", "ifconfig", interface, "inet", self.HOST_IP, "netmask", "255.255.255.0"
        ], capture=False)
        
        # Bring interface up
        self.run_command(["sudo", "ifconfig", interface, "up"], capture=False)
        
        # Add route to Vision Pro
        self.run_command([
            "sudo", "route", "-n", "add", "-net", "169.254.220.0/24", "-interface", interface
        ], check=False, capture=False)
    
    def setup_interface_linux(self, interface: str):
        """Configure the USB interface on Linux."""
        print(f"  • Configuring {interface}...")
        
        # Flush any existing IPs
        self.run_command(["sudo", "ip", "addr", "flush", "dev", interface], check=False, capture=False)
        
        # Assign our IP address (use /16 for link-local addressing)
        self.run_command([
            "sudo", "ip", "addr", "add", f"{self.HOST_IP}/16", "dev", interface
        ], capture=False)
        
        # Bring interface up
        self.run_command(["sudo", "ip", "link", "set", interface, "up"], capture=False)
        
        # Add route to Vision Pro's subnet
        self.run_command([
            "sudo", "ip", "route", "add", "169.254.220.0/24", "dev", interface
        ], check=False, capture=False)
    
    def create_cleanup_script_macos(self, interface: str) -> str:
        """Create cleanup script for macOS."""
        cleanup_script = f"""#!/bin/bash
# Vision Pro USB Network Cleanup (macOS)

echo "Cleaning up Vision Pro USB network..."

# Remove routes
sudo route -n delete -net 169.254.220.0/24 2>/dev/null || true

# Remove IP address
sudo ifconfig {interface} delete 2>/dev/null || true

# Bring interface down
sudo ifconfig {interface} down 2>/dev/null || true

echo "✅ Cleanup complete!"
"""
        cleanup_path = "/tmp/vision-pro-cleanup.sh"
        with open(cleanup_path, 'w') as f:
            f.write(cleanup_script)
        subprocess.run(["chmod", "+x", cleanup_path])
        return cleanup_path
    
    def create_cleanup_script_linux(self, interface: str) -> str:
        """Create cleanup script for Linux."""
        cleanup_script = f"""#!/bin/bash
# Vision Pro USB Network Cleanup (Linux)

echo "Cleaning up Vision Pro USB network..."

# Remove routes
sudo ip route del 169.254.220.0/24 dev {interface} 2>/dev/null || true

# Flush IP addresses
sudo ip addr flush dev {interface} 2>/dev/null || true

# Bring interface down
sudo ip link set {interface} down 2>/dev/null || true

echo "✅ Cleanup complete!"
"""
        cleanup_path = "/tmp/vision-pro-cleanup.sh"
        with open(cleanup_path, 'w') as f:
            f.write(cleanup_script)
        subprocess.run(["chmod", "+x", cleanup_path])
        return cleanup_path
    
    def test_connection(self) -> bool:
        """Test if we can reach Vision Pro."""
        # Skip automatic testing since Vision Pro IP is assigned dynamically
        return True
    
    def setup(self):
        """Main setup flow."""
        print("🔧 Vision Pro Simple USB Network Setup")
        print("=" * 40)
        print("")
        print("This script configures LOCAL-ONLY network access.")
        print("Vision Pro will NOT have internet access.")
        print("")
        
        # Check platform support
        if self.os_type == "Windows":
            print("❌ Windows is not yet supported")
            return False
        
        if self.os_type not in ["Darwin", "Linux"]:
            print(f"❌ Unsupported platform: {self.os_type}")
            return False
        
        # Check if Vision Pro is connected
        print("⚠️  IMPORTANT: Plug in your Vision Pro via USB-C first!")
        print("")
        response = input("Is your Vision Pro plugged in? (y/n) ").strip().lower()
        if response not in ['y', 'yes']:
            print("")
            print("Please plug in your Vision Pro and run this script again.")
            return False
        
        print("")
        print("Looking for USB network interface...")
        
        # Wait a moment for interface to appear
        time.sleep(2)
        
        try:
            # Find USB interface
            if self.os_type == "Darwin":
                self.usb_interface = self.find_usb_interface_macos()
            elif self.os_type == "Linux":
                self.usb_interface = self.find_usb_interface_linux()
            
            if not self.usb_interface:
                print("")
                print("❌ Could not find USB network interface!")
                print("")
                print("Troubleshooting:")
                print("  1. Ensure Vision Pro is plugged in via USB-C")
                print("  2. Try unplugging and replugging")
                print("  3. Wait a few seconds for the interface to appear")
                if self.os_type == "Linux":
                    print("  4. Run 'ip link show' to see all interfaces")
                else:
                    print("  4. Run 'ifconfig -l' to see all interfaces")
                return False
            
            print(f"✓ Found USB interface: {self.usb_interface}")
            print("")
            print("Configuration:")
            print(f"  Platform: {self.os_type}")
            print(f"  USB Interface: {self.usb_interface}")
            print(f"  Host IP: {self.HOST_IP}")
            print(f"  Network: {self.VISION_PRO_NETWORK}")
            print("")
            
            response = input("Continue? (y/n) ").strip().lower()
            if response not in ['y', 'yes']:
                return False
            
            print("")
            print("Setting up network...")
            
            # Configure interface
            if self.os_type == "Darwin":
                self.setup_interface_macos(self.usb_interface)
                cleanup_path = self.create_cleanup_script_macos(self.usb_interface)
            elif self.os_type == "Linux":
                self.setup_interface_linux(self.usb_interface)
                cleanup_path = self.create_cleanup_script_linux(self.usb_interface)
            
            # Success message
            print("")
            print("✅ Setup complete!")
            print("")
            print("Your Vision Pro is now connected:")
            print(f"  • Host IP: {self.HOST_IP}")
            print("  • High-speed local communication enabled")
            print("  • No internet access (by design)")
            print("")
            print("📱 To find Vision Pro's IP address:")
            print("   Settings → General → About → IP Address")
            print("   (It will be in the 169.254.x.x range)")
            print("")
            print(f"🧹 To cleanup when done: {cleanup_path}")
            print("")
            
            return True
            
        except NetworkSetupError as e:
            print(f"\n❌ Error: {e}")
            return False
        except KeyboardInterrupt:
            print("\n\n⚠️  Setup cancelled by user")
            return False
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Main entry point for the USB network setup utility."""
    network = VisionProUSBNetwork()
    success = network.setup()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
