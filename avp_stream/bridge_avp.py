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
    
    def detect_configuration(self):
        """Detect network configuration for the current platform."""
        print("Detecting configuration...")
        
        if self.os_type == "Darwin":
            self.primary_interface, self.ip_addr = self.detect_primary_interface_macos()
        elif self.os_type == "Linux":
            self.primary_interface, self.ip_addr = self.detect_primary_interface_linux()
        else:
            raise BridgeSetupError(f"Unsupported platform: {self.os_type}")
        
        # Calculate network base and bridge IP
        parts = self.ip_addr.split('.')
        self.network_base = '.'.join(parts[:3])
        self.bridge_ip = f"{self.network_base}.200"
    
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
        """Set up bridge on Linux."""
        print("\nSetting up bridge...")
        
        # 1. Enable IP forwarding
        print("  • Enabling IP forwarding...")
        self.run_command(["sudo", "sysctl", "-w", "net.ipv4.ip_forward=1"], capture=False)
        
        # 2. Create bridge interface if it doesn't exist
        print("  • Creating bridge interface...")
        result = self.run_command(["ip", "link", "show", "br0"], check=False)
        if result.returncode != 0:
            self.run_command(["sudo", "brctl", "addbr", "br0"], capture=False)
        
        # 3. Configure bridge IP
        print("  • Configuring bridge IP...")
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
        cleanup_script = f"""#!/bin/bash
# Vision Pro Bridge Cleanup Script (Linux)

echo "Cleaning up Vision Pro bridge..."

# Remove iptables rules
sudo iptables -t nat -D POSTROUTING -s {self.network_base}.0/24 -o {self.primary_interface} -j MASQUERADE 2>/dev/null || true
sudo iptables -D FORWARD -i br0 -o {self.primary_interface} -j ACCEPT 2>/dev/null || true
sudo iptables -D FORWARD -i {self.primary_interface} -o br0 -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

# Remove interface from bridge
sudo brctl delif br0 {self.primary_interface} 2>/dev/null || true

# Bring down and delete bridge
sudo ip link set br0 down 2>/dev/null || true
sudo brctl delbr br0 2>/dev/null || true

# Disable IP forwarding
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
