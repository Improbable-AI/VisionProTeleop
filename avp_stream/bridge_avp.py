#!/usr/bin/env python3
"""
Vision Pro Static IP Network Setup

This module sets up a simple point-to-point network connection for Vision Pro devices
using static IP addresses over USB-C connection.

Network Configuration:
- Vision Pro (host): 169.254.1.1/16
- Computer: 169.254.1.2/16
- Subnet mask: 255.255.0.0
- Optional: NAT for internet sharing

Platform Support:
- macOS: Full support using ifconfig
- Linux: Full support using ip commands
- Windows: Not yet supported
"""

import subprocess
import sys
import platform
import re
import shutil
from typing import Optional, Tuple


class BridgeSetupError(Exception):
    """Exception raised for bridge setup errors."""
    pass


class VisionProBridge:
    """Cross-platform Vision Pro network bridge setup."""
    
    def __init__(self):
        self.os_type = platform.system()
        self.primary_interface: Optional[str] = None
        self.ip_addr: Optional[str] = None
        self.network_base: Optional[str] = None
        self.bridge_ip: Optional[str] = None
        
    def check_prerequisites(self) -> bool:
        """Check if required tools are available for the current platform."""
        if self.os_type == "Darwin":  # macOS
            required_commands = ["ifconfig", "route", "sysctl"]
        elif self.os_type == "Linux":
            required_commands = ["ip", "sysctl"]
        else:
            return False
            
        for cmd in required_commands:
            if not shutil.which(cmd):
                print(f"❌ Error: Required command '{cmd}' not found")
                return False
        return True
    
    def run_command(self, cmd: list, check: bool = True, capture: bool = True) -> subprocess.CompletedProcess:
        """Run a shell command with error handling."""
        try:
            if capture:
                result = subprocess.run(cmd, capture_output=True, text=True, check=check)
            else:
                result = subprocess.run(cmd, check=check)
            return result
        except subprocess.CalledProcessError as e:
            raise BridgeSetupError(f"Command failed: {' '.join(cmd)}\n{e.stderr if capture else ''}")
    
    def detect_primary_interface_macos(self) -> Tuple[str, str]:
        """Detect primary network interface on macOS."""
        try:
            # Get default route interface
            result = self.run_command(["route", "-n", "get", "default"])
            match = re.search(r'interface:\s+(\S+)', result.stdout)
            if not match:
                raise BridgeSetupError("Could not detect primary network interface")
            
            interface = match.group(1)
            
            # Get IP address
            result = self.run_command(["ifconfig", interface])
            match = re.search(r'inet\s+(\d+\.\d+\.\d+\.\d+)', result.stdout)
            if not match:
                raise BridgeSetupError(f"No IP address found on {interface}")
            
            ip_addr = match.group(1)
            return interface, ip_addr
            
        except Exception as e:
            raise BridgeSetupError(f"Failed to detect network configuration: {e}")
    
    def detect_primary_interface_linux(self) -> Tuple[str, str]:
        """Detect primary network interface on Linux."""
        try:
            # Get default route interface
            result = self.run_command(["ip", "route", "show", "default"])
            match = re.search(r'dev\s+(\S+)', result.stdout)
            if not match:
                raise BridgeSetupError("Could not detect primary network interface")
            
            interface = match.group(1)
            
            # Get IP address
            result = self.run_command(["ip", "addr", "show", interface])
            match = re.search(r'inet\s+(\d+\.\d+\.\d+\.\d+)', result.stdout)
            if not match:
                raise BridgeSetupError(f"No IP address found on {interface}")
            
            ip_addr = match.group(1)
            return interface, ip_addr
            
        except Exception as e:
            raise BridgeSetupError(f"Failed to detect network configuration: {e}")
    
    def is_wireless_interface(self, interface: str) -> bool:
        """Check if an interface is wireless."""
        if self.os_type == "Linux":
            # Check if wireless directory exists
            wireless_path = f"/sys/class/net/{interface}/wireless"
            import os
            return os.path.exists(wireless_path)
        return False
    
    def detect_configuration(self):
        """Detect network configuration for the current platform."""
        print("Detecting configuration...")
        
        if self.os_type == "Darwin":
            self.primary_interface, self.ip_addr = self.detect_primary_interface_macos()
        elif self.os_type == "Linux":
            self.primary_interface, self.ip_addr = self.detect_primary_interface_linux()
        else:
            raise BridgeSetupError(f"Unsupported platform: {self.os_type}")
        
        # Simple static IP setup: Vision Pro is host at 169.254.1.1
        # This computer gets 169.254.1.2 for direct point-to-point connection
        self.network_base = "169.254.1"
        self.bridge_ip = "169.254.1.2"  # Computer's IP (Vision Pro is .1)
    
    def setup_bridge_macos(self):
        """Set up simple static IP configuration on macOS."""
        print("\nSetting up static IP configuration...")
        
        # 1. Find USB network interface
        print("  • Looking for USB network interface...")
        result = self.run_command(["ifconfig", "-a"])
        
        # Look for interfaces that aren't the primary one and aren't loopback
        usb_interface = None
        for line in result.stdout.split('\n'):
            match = re.search(r'^(en\d+):', line)
            if match:
                iface = match.group(1)
                if iface != self.primary_interface and iface != 'lo0':
                    # Check if it has a 169.254 address or no address (new connection)
                    iface_info = self.run_command(["ifconfig", iface])
                    if '169.254' in iface_info.stdout or 'inet ' not in iface_info.stdout:
                        usb_interface = iface
                        break
        
        if not usb_interface:
            print("\n  ⚠️  No USB network interface detected!")
            print("  • Please ensure Vision Pro is plugged in via USB-C")
            print("  • Wait 5 seconds after plugging in, then run this script again")
            print("  • Run 'ifconfig -a' to see all interfaces")
            raise BridgeSetupError("USB network interface not detected")
        
        print(f"  • Found USB interface: {usb_interface}")
        
        # 2. Configure static IP on USB interface
        print(f"  • Configuring {usb_interface} with IP {self.bridge_ip}...")
        print(f"    (Vision Pro should be at 169.254.1.1)")
        
        # Remove any existing IP addresses
        self.run_command([
            "sudo", "ifconfig", usb_interface, "inet", "0.0.0.0", "delete"
        ], check=False, capture=False)
        
        # Set static IP with /16 subnet mask (255.255.0.0)
        self.run_command([
            "sudo", "ifconfig", usb_interface, "inet", self.bridge_ip,
            "netmask", "255.255.0.0"
        ], capture=False)
        
        subprocess.run(["sleep", "1"])
        
        # 3. Optional: Enable IP forwarding and NAT for internet access
        response = input("\n  Enable internet sharing for Vision Pro? (y/n) ").strip().lower()
        if response in ['y', 'yes']:
            print("  • Enabling IP forwarding...")
            self.run_command(["sudo", "sysctl", "-w", "net.inet.ip.forwarding=1"], capture=False)
            
            print("  • Configuring NAT...")
            self.run_command(["sudo", "pfctl", "-e"], check=False, capture=False)
            
            nat_rule = f"nat on {self.primary_interface} from 169.254.0.0/16 to any -> ({self.primary_interface})"
            proc = subprocess.Popen(
                ["sudo", "pfctl", "-f", "-"],
                stdin=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
            proc.communicate(input=nat_rule.encode())
            self.nat_enabled = True
        else:
            self.nat_enabled = False
        
        # Store USB interface for cleanup
        self.usb_interface = usb_interface
        subprocess.run(["sleep", "1"])
    
    def setup_bridge_linux(self):
        """Set up simple static IP configuration on Linux."""
        print("\nSetting up static IP configuration...")
        
        # 1. Find USB network interface
        print("  • Looking for USB network interface...")
        result = self.run_command(["ip", "link", "show"], check=False)
        
        # Show all interfaces for debugging
        print("\n  Available network interfaces:")
        all_interfaces = []
        for line in result.stdout.split('\n'):
            match = re.search(r'\d+:\s+(\S+):', line)
            if match and '@' not in line:  # Skip VLAN interfaces
                iface = match.group(1)
                if iface not in ['lo', self.primary_interface]:
                    all_interfaces.append(iface)
                    print(f"    - {iface}")
        
        # Wait a moment for interface to appear if none found
        if not all_interfaces:
            print("  • Waiting 3 seconds for USB interface to appear...")
            subprocess.run(["sleep", "3"])
            result = self.run_command(["ip", "link", "show"], check=False)
            for line in result.stdout.split('\n'):
                match = re.search(r'\d+:\s+(\S+):', line)
                if match and '@' not in line:
                    iface = match.group(1)
                    if iface not in ['lo', self.primary_interface]:
                        all_interfaces.append(iface)
        
        if not all_interfaces:
            print("\n  ⚠️  No USB network interface detected!")
            print("  • Please ensure Vision Pro is plugged in via USB-C")
            print("  • Wait 5 seconds after plugging in, then run this script again")
            print("  • Run 'ip link show' to see all interfaces")
            raise BridgeSetupError("USB network interface not detected")
        
        # Prefer interfaces that look like USB/ethernet
        usb_interface = None
        for iface in all_interfaces:
            if any(pattern in iface for pattern in ['enp', 'usb', 'eth', 'en']):
                usb_interface = iface
                break
        
        # If no specific USB pattern found, use the first available
        if not usb_interface:
            usb_interface = all_interfaces[0]
        
        print(f"\n  • Using interface: {usb_interface}")
        
        # 2. Configure static IP on USB interface
        print(f"  • Configuring {usb_interface} with IP {self.bridge_ip}...")
        print(f"    (Vision Pro should be at 169.254.1.1)")
        
        # Remove any existing IPs
        self.run_command(["sudo", "ip", "addr", "flush", "dev", usb_interface], check=False, capture=False)
        
        # Add our IP with /16 subnet mask (255.255.0.0)
        self.run_command([
            "sudo", "ip", "addr", "add", f"{self.bridge_ip}/16", "dev", usb_interface
        ], check=False, capture=False)
        
        # Bring interface up
        self.run_command(["sudo", "ip", "link", "set", usb_interface, "up"], capture=False)
        
        subprocess.run(["sleep", "1"])
        
        # 3. Optional: Enable IP forwarding and NAT for internet access
        response = input("\n  Enable internet sharing for Vision Pro? (y/n) ").strip().lower()
        if response in ['y', 'yes']:
            print("  • Enabling IP forwarding...")
            self.run_command(["sudo", "sysctl", "-w", "net.ipv4.ip_forward=1"], capture=False)
            
            print("  • Configuring NAT...")
            self.run_command([
                "sudo", "iptables", "-t", "nat", "-A", "POSTROUTING",
                "-s", "169.254.0.0/16",
                "-o", self.primary_interface, "-j", "MASQUERADE"
            ], check=False, capture=False)
            
            # Allow forwarding
            self.run_command([
                "sudo", "iptables", "-A", "FORWARD", "-i", usb_interface,
                "-o", self.primary_interface, "-j", "ACCEPT"
            ], check=False, capture=False)
            
            self.run_command([
                "sudo", "iptables", "-A", "FORWARD", "-i", self.primary_interface,
                "-o", usb_interface, "-m", "state", "--state", "RELATED,ESTABLISHED",
                "-j", "ACCEPT"
            ], check=False, capture=False)
            self.nat_enabled = True
        else:
            self.nat_enabled = False
        
        # Store USB interface for cleanup
        self.usb_interface = usb_interface
        subprocess.run(["sleep", "1"])
    
    def create_cleanup_script_macos(self):
        """Create cleanup script for macOS."""
        nat_cleanup = ""
        if hasattr(self, 'nat_enabled') and self.nat_enabled:
            nat_cleanup = f"""
# Disable IP forwarding
sudo sysctl -w net.inet.ip.forwarding=0 > /dev/null

# Flush pfctl rules
sudo pfctl -F all 2>/dev/null || true
sudo pfctl -d 2>/dev/null || true
"""
        
        cleanup_script = f"""#!/bin/bash
# Vision Pro Static IP Cleanup Script (macOS)

echo "Cleaning up Vision Pro static IP configuration..."

# Remove static IP from USB interface
sudo ifconfig {self.usb_interface} inet {self.bridge_ip} delete 2>/dev/null || true
{nat_cleanup}
echo "✅ Cleanup complete!"
"""
        cleanup_path = "/tmp/vision-pro-cleanup.sh"
        with open(cleanup_path, 'w') as f:
            f.write(cleanup_script)
        subprocess.run(["chmod", "+x", cleanup_path])
        return cleanup_path
    
    def create_cleanup_script_linux(self):
        """Create cleanup script for Linux."""
        nat_cleanup = ""
        if hasattr(self, 'nat_enabled') and self.nat_enabled:
            nat_cleanup = f"""
# Remove iptables rules
sudo iptables -t nat -D POSTROUTING -s 169.254.0.0/16 -o {self.primary_interface} -j MASQUERADE 2>/dev/null || true
sudo iptables -D FORWARD -i {self.usb_interface} -o {self.primary_interface} -j ACCEPT 2>/dev/null || true
sudo iptables -D FORWARD -i {self.primary_interface} -o {self.usb_interface} -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

# Disable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=0 > /dev/null
"""
        
        cleanup_script = f"""#!/bin/bash
# Vision Pro Static IP Cleanup Script (Linux)

echo "Cleaning up Vision Pro static IP configuration..."

# Remove static IP from USB interface
sudo ip addr flush dev {self.usb_interface} 2>/dev/null || true
sudo ip link set {self.usb_interface} down 2>/dev/null || true
{nat_cleanup}
echo "✅ Cleanup complete!"
"""
        cleanup_path = "/tmp/vision-pro-cleanup.sh"
        with open(cleanup_path, 'w') as f:
            f.write(cleanup_script)
        subprocess.run(["chmod", "+x", cleanup_path])
        return cleanup_path
    
    def setup(self):
        """Main setup flow."""
        print("🔧 Vision Pro Static IP Network Setup")
        print("=" * 38)
        print("")
        
        # Check platform support
        if self.os_type == "Windows":
            print("❌ Windows is not yet supported")
            print("   This tool currently supports macOS and Linux only.")
            return False
        
        if self.os_type not in ["Darwin", "Linux"]:
            print(f"❌ Unsupported platform: {self.os_type}")
            return False
        
        # Check prerequisites
        if not self.check_prerequisites():
            return False
        
        # Warn about Vision Pro
        print("⚠️  IMPORTANT:")
        print("   1. Plug in your Vision Pro via USB-C BEFORE running this script")
        print("   2. Configure Vision Pro with static IP: 169.254.1.1, subnet: 255.255.0.0")
        print("")
        response = input("Is your Vision Pro plugged in and configured? (y/n) ").strip().lower()
        if response not in ['y', 'yes']:
            print("")
            print("Please:")
            print("  • Plug in your Vision Pro via USB-C")
            print("  • Set static IP to 169.254.1.1 with subnet mask 255.255.0.0")
            print("  • Wait 5 seconds, then run this script again")
            return False
        
        print("")
        
        try:
            # Detect configuration
            self.detect_configuration()
            
            # Show configuration
            print("")
            print("Configuration:")
            print(f"  Platform: {self.os_type}")
            print(f"  Primary interface: {self.primary_interface}")
            print(f"  Your machine's IP: {self.ip_addr}")
            print(f"  Network: {self.network_base}.0/24")
            print(f"  Bridge IP: {self.bridge_ip}")
            print("")
            
            response = input("Continue? (y/n) ").strip().lower()
            if response not in ['y', 'yes']:
                return False
            
            # Set up bridge based on platform
            if self.os_type == "Darwin":
                self.setup_bridge_macos()
                cleanup_path = self.create_cleanup_script_macos()
            elif self.os_type == "Linux":
                self.setup_bridge_linux()
                cleanup_path = self.create_cleanup_script_linux()
            
            # Success message
            print("")
            print("✅ Setup complete!")
            print("")
            print("⏳ Connection should be active in ~3 seconds...")
            print("")
            print("Network Configuration:")
            print(f"  • Vision Pro IP: 169.254.1.1 (host)")
            print(f"  • Your computer IP: {self.bridge_ip}")
            print(f"  • Subnet mask: 255.255.0.0 (/16)")
            print("")
            if hasattr(self, 'nat_enabled') and self.nat_enabled:
                print("  • Internet sharing: ENABLED")
                print("  • Vision Pro has internet access through your computer")
            else:
                print("  • Internet sharing: DISABLED")
                print("  • Direct point-to-point connection only")
            print("")
            print("🧪 To test connection: ping 169.254.1.1")
            print(f"🧹 To cleanup when done: {cleanup_path}")
            
            return True
            
        except BridgeSetupError as e:
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
    """Main entry point for the bridge setup utility."""
    bridge = VisionProBridge()
    success = bridge.setup()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
