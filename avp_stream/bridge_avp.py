#!/usr/bin/env python3
"""
Vision Pro High-Speed Bridge Setup

This module sets up a high-speed network bridge for Vision Pro devices,
providing ~10 Gbps local network speeds with full internet access.

Platform Support:
- macOS: Full support using pfctl and bridge interfaces
- Linux: Support using iptables and bridge-utils
- Windows: Not yet supported (requires netsh and bridge configuration)
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
            required_commands = ["ifconfig", "route", "sysctl", "pfctl"]
        elif self.os_type == "Linux":
            required_commands = ["ip", "brctl", "sysctl", "iptables"]
        else:
            return False
            
        for cmd in required_commands:
            if not shutil.which(cmd):
                print(f"❌ Error: Required command '{cmd}' not found")
                if self.os_type == "Linux" and cmd == "brctl":
                    print("   Install with: sudo apt-get install bridge-utils")
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
        
        # For USB connections, Vision Pro uses link-local addressing (169.254.x.x)
        # We should use a compatible address on the same subnet
        self.network_base = "169.254.220"
        self.bridge_ip = "169.254.220.1"  # Use .1 as the host IP
    
    def setup_bridge_macos(self):
        """Set up bridge on macOS."""
        print("\nSetting up bridge...")
        
        # 1. Enable IP forwarding
        print("  • Enabling IP forwarding...")
        self.run_command(["sudo", "sysctl", "-w", "net.inet.ip.forwarding=1"], capture=False)
        
        # 2. Configure bridge0
        print("  • Configuring bridge0...")
        self.run_command([
            "sudo", "ifconfig", "bridge0", "inet", self.bridge_ip, 
            "netmask", "255.255.255.0"
        ], capture=False)
        subprocess.run(["sleep", "1"])
        
        # 3. Add primary interface to bridge0
        print(f"  • Adding {self.primary_interface} to bridge0...")
        self.run_command([
            "sudo", "ifconfig", "bridge0", "addm", self.primary_interface
        ], check=False, capture=False)
        subprocess.run(["sleep", "2"])
        
        # 4. Set up NAT
        print("  • Configuring NAT...")
        # Enable pfctl
        self.run_command(["sudo", "pfctl", "-e"], check=False, capture=False)
        
        # Create NAT rule
        nat_rule = f"nat on {self.primary_interface} inet from bridge0:network to any -> ({self.primary_interface})"
        proc = subprocess.Popen(
            ["sudo", "pfctl", "-f", "-"],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        proc.communicate(input=nat_rule.encode())
        subprocess.run(["sleep", "2"])
    
    def setup_bridge_linux(self):
        """Set up bridge on Linux.
        
        For wired interfaces: Creates a true bridge
        For wireless interfaces: Uses USB network interface directly with NAT
        """
        print("\nSetting up bridge...")
        
        # Check if primary interface is wireless
        is_wireless = self.is_wireless_interface(self.primary_interface)
        
        # 1. Enable IP forwarding
        print("  • Enabling IP forwarding...")
        self.run_command(["sudo", "sysctl", "-w", "net.ipv4.ip_forward=1"], capture=False)
        
        if is_wireless:
            # For wireless: Skip bridge creation, use USB network interface directly
            print(f"  ⚠️  Detected wireless interface ({self.primary_interface})")
            print("  • Setting up direct USB network routing (wireless can't bridge)")
            print("  • Looking for USB network interface...")
            
            # Find USB network interfaces (usually enp*, usb*)
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
            
            # Wait a moment for interface to appear
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
            
            if all_interfaces:
                # Prefer interfaces that look like USB/ethernet
                usb_iface = None
                for iface in all_interfaces:
                    if any(pattern in iface for pattern in ['enp', 'usb', 'eth', 'en']):
                        usb_iface = iface
                        break
                
                # If no specific USB pattern found, use the first available
                if not usb_iface:
                    usb_iface = all_interfaces[0]
                
                print(f"\n  • Using interface: {usb_iface}")
                
                # Configure the USB interface
                print(f"  • Configuring {usb_iface} with IP {self.bridge_ip}...")
                print(f"    (Vision Pro should be at 169.254.220.107)")
                # Remove any existing IPs
                self.run_command(["sudo", "ip", "addr", "flush", "dev", usb_iface], check=False, capture=False)
                # Add our IP on the same link-local subnet as Vision Pro
                # Use /16 for link-local addressing
                self.run_command([
                    "sudo", "ip", "addr", "add", f"{self.bridge_ip}/16", "dev", usb_iface
                ], check=False, capture=False)
                # Bring it up
                self.run_command(["sudo", "ip", "link", "set", usb_iface, "up"], capture=False)
                
                # Add route to Vision Pro's subnet
                print("  • Adding route to Vision Pro...")
                self.run_command([
                    "sudo", "ip", "route", "add", "169.254.220.0/24", "dev", usb_iface
                ], check=False, capture=False)
                
                # Set up NAT
                print("  • Configuring NAT...")
                self.run_command([
                    "sudo", "iptables", "-t", "nat", "-A", "POSTROUTING",
                    "-s", f"{self.network_base}.0/24",
                    "-o", self.primary_interface, "-j", "MASQUERADE"
                ], check=False, capture=False)
                
                # Allow forwarding
                self.run_command([
                    "sudo", "iptables", "-A", "FORWARD", "-i", usb_iface,
                    "-o", self.primary_interface, "-j", "ACCEPT"
                ], check=False, capture=False)
                
                self.run_command([
                    "sudo", "iptables", "-A", "FORWARD", "-i", self.primary_interface,
                    "-o", usb_iface, "-m", "state", "--state", "RELATED,ESTABLISHED",
                    "-j", "ACCEPT"
                ], check=False, capture=False)
                
                # Store the USB interface for cleanup
                self.usb_interface = usb_iface
            else:
                print("\n  ⚠️  No additional network interfaces found!")
                print("  • Please ensure Vision Pro is plugged in via USB-C")
                print("  • Try unplugging and replugging the Vision Pro")
                print("  • Run 'ip link show' to see all interfaces")
                raise BridgeSetupError("USB network interface not detected")
        else:
            # For wired: Create a proper bridge
            print(f"  • Using wired interface ({self.primary_interface})")
            
            # 2. Create bridge interface if it doesn't exist
            print("  • Creating bridge interface...")
            result = self.run_command(["ip", "link", "show", "br0"], check=False)
            if result.returncode != 0:
                self.run_command(["sudo", "brctl", "addbr", "br0"], capture=False)
            
            # 3. Configure bridge IP
            print("  • Configuring bridge IP...")
            # Flush any existing IPs
            self.run_command(["sudo", "ip", "addr", "flush", "dev", "br0"], check=False, capture=False)
            
            self.run_command([
                "sudo", "ip", "addr", "add", f"{self.bridge_ip}/24", "dev", "br0"
            ], check=False, capture=False)
            
            # 4. Add primary interface to bridge
            print(f"  • Adding {self.primary_interface} to bridge...")
            self.run_command([
                "sudo", "brctl", "addif", "br0", self.primary_interface
            ], check=False, capture=False)
            
            # 5. Bring up the bridge
            print("  • Bringing up bridge interface...")
            self.run_command(["sudo", "ip", "link", "set", "br0", "up"], capture=False)
            
            # 6. Set up NAT with iptables
            print("  • Configuring NAT...")
            self.run_command([
                "sudo", "iptables", "-t", "nat", "-A", "POSTROUTING",
                "-s", f"{self.network_base}.0/24",
                "-o", self.primary_interface, "-j", "MASQUERADE"
            ], check=False, capture=False)
            
            # Allow forwarding
            self.run_command([
                "sudo", "iptables", "-A", "FORWARD", "-i", "br0",
                "-o", self.primary_interface, "-j", "ACCEPT"
            ], check=False, capture=False)
            
            self.run_command([
                "sudo", "iptables", "-A", "FORWARD", "-i", self.primary_interface,
                "-o", "br0", "-m", "state", "--state", "RELATED,ESTABLISHED",
                "-j", "ACCEPT"
            ], check=False, capture=False)
        
        subprocess.run(["sleep", "2"])
    
    def create_cleanup_script_macos(self):
        """Create cleanup script for macOS."""
        cleanup_script = f"""#!/bin/bash
# Vision Pro Bridge Cleanup Script (macOS)

echo "Cleaning up Vision Pro bridge..."

# Remove bridge interface from primary interface
sudo ifconfig bridge0 deletem {self.primary_interface} 2>/dev/null || true

# Remove bridge IP
sudo ifconfig bridge0 delete {self.bridge_ip} 2>/dev/null || true

# Disable IP forwarding
sudo sysctl -w net.inet.ip.forwarding=0 > /dev/null

# Flush pfctl rules
sudo pfctl -F all 2>/dev/null || true
sudo pfctl -d 2>/dev/null || true

echo "✅ Cleanup complete!"
"""
        cleanup_path = "/tmp/vision-pro-cleanup.sh"
        with open(cleanup_path, 'w') as f:
            f.write(cleanup_script)
        subprocess.run(["chmod", "+x", cleanup_path])
        return cleanup_path
    
    def create_cleanup_script_linux(self):
        """Create cleanup script for Linux."""
        is_wireless = self.is_wireless_interface(self.primary_interface)
        
        cleanup_script = f"""#!/bin/bash
# Vision Pro Bridge Cleanup Script (Linux)

echo "Cleaning up Vision Pro bridge..."

# Remove iptables rules
sudo iptables -t nat -D POSTROUTING -s {self.network_base}.0/24 -o {self.primary_interface} -j MASQUERADE 2>/dev/null || true

"""
        
        if is_wireless:
            # For wireless setup, clean up USB interface
            cleanup_script += """# Find and clean up USB network interfaces
for iface in $(ip link show | grep -E 'enp|usb|eth' | awk -F': ' '{print $2}'); do
    if [ "$iface" != "$primary_iface" ]; then
        sudo iptables -D FORWARD -i $iface -o """ + self.primary_interface + """ -j ACCEPT 2>/dev/null || true
        sudo iptables -D FORWARD -i """ + self.primary_interface + """ -o $iface -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
        sudo ip addr flush dev $iface 2>/dev/null || true
        sudo ip link set $iface down 2>/dev/null || true
    fi
done

"""
        else:
            # For wired setup, clean up bridge
            cleanup_script += f"""sudo iptables -D FORWARD -i br0 -o {self.primary_interface} -j ACCEPT 2>/dev/null || true
sudo iptables -D FORWARD -i {self.primary_interface} -o br0 -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

# Remove interface from bridge
sudo brctl delif br0 {self.primary_interface} 2>/dev/null || true

# Bring down and delete bridge
sudo ip link set br0 down 2>/dev/null || true
sudo brctl delbr br0 2>/dev/null || true

"""
        
        cleanup_script += """# Disable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=0 > /dev/null

echo "✅ Cleanup complete!"
"""
        cleanup_path = "/tmp/vision-pro-cleanup.sh"
        with open(cleanup_path, 'w') as f:
            f.write(cleanup_script)
        subprocess.run(["chmod", "+x", cleanup_path])
        return cleanup_path
    
    def setup(self):
        """Main setup flow."""
        print("🔧 Vision Pro High-Speed Bridge Setup")
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
        print("⚠️  IMPORTANT: Plug in your Vision Pro BEFORE running this script!")
        print("")
        response = input("Is your Vision Pro plugged in? (y/n) ").strip().lower()
        if response not in ['y', 'yes']:
            print("")
            print("Please plug in your Vision Pro, wait 5 seconds, then run this script again.")
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
            print("⏳ Network should be active in ~5 seconds...")
            print("")
            print("Your Vision Pro should now have:")
            print("  • ~10 Gbps local network speeds (test with iperf)")
            print("  • Full internet access")
            print("")
            print("🧪 To test internet: Try loading a webpage on Vision Pro")
            print(f"🧹 To cleanup when done: Run {cleanup_path}")
            
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
