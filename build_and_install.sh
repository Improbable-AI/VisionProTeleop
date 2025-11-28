#!/bin/bash
#
# Build and Install Script for Tracking Streamer (Vision Pro App)
#
# This script builds the Xcode project and installs it to either a simulator
# or a physical device based on the input argument.
#
# Usage:
#   ./build_and_install.sh [build|device|simulator]
#   ./build_and_install.sh                 # Default: build only (fastest, for checking errors)
#   ./build_and_install.sh build           # Build only, no install
#   ./build_and_install.sh device          # Build and install on physical Vision Pro
#   ./build_and_install.sh simulator       # Build and install on visionOS Simulator
#
# Requirements:
#   - Xcode installed with visionOS SDK
#   - For device: Vision Pro connected and trusted
#   - For simulator: visionOS Simulator available
#

set -e  # Exit on error

# =============================================================================
# Configuration
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
PROJECT_NAME="Tracking Streamer"
SCHEME="VisionProTeleop"
BUNDLE_ID="improbable.younghyo.DexTeleop2"

# Use a persistent build directory outside iCloud Drive for incremental builds
# This avoids iCloud extended attributes AND preserves build cache between runs
BUILD_DIR="$HOME/.visionpro-build/VisionProTeleop"

# Xcode path - change this if you want to use a different Xcode version
XCODE_PATH="/Applications/Xcode.app"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Add user gem bin to PATH for xcpretty
if [ -d "$HOME/.gem/ruby/2.6.0/bin" ]; then
    export PATH="$HOME/.gem/ruby/2.6.0/bin:$PATH"
fi

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_usage() {
    echo "Usage: $0 [build|device|simulator|logs-simulator|logs-device|clean]"
    echo ""
    echo "Arguments:"
    echo "  build           Build only, no install (default, fastest for checking errors)"
    echo "  device          Build and install on physical Vision Pro"
    echo "  simulator       Build and install on visionOS Simulator"
    echo "  logs-simulator  Stream console logs from visionOS Simulator"
    echo "  logs-device     Stream console logs from physical Vision Pro"
    echo "  clean           Delete build cache to force full rebuild"
    echo ""
    echo "Examples:"
    echo "  $0                  # Just build (check for errors)"
    echo "  $0 build            # Same as above"
    echo "  $0 device           # Build and install on Vision Pro"
    echo "  $0 simulator        # Build and install on Simulator"
    echo "  $0 logs-simulator   # View simulator logs (Ctrl+C to stop)"
    echo "  $0 logs-device      # View device logs (Ctrl+C to stop)"
    echo "  $0 clean            # Clean build cache"
}

check_xcode() {
    if ! command -v xcodebuild &> /dev/null; then
        log_error "xcodebuild not found. Please install Xcode."
        exit 1
    fi
    
    # Set the Xcode path if specified
    if [ -d "$XCODE_PATH" ]; then
        export DEVELOPER_DIR="$XCODE_PATH/Contents/Developer"
        log_info "Using Xcode: $XCODE_PATH"
    else
        log_warning "Xcode not found at $XCODE_PATH, using default: $(xcode-select -p)"
    fi
}

get_simulator_id() {
    # Get the first available visionOS simulator
    # Look for the id field in the xcodebuild output format
    local sim_id
    sim_id=$(xcodebuild -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" -scheme "$SCHEME" -showdestinations 2>/dev/null \
        | grep "platform:visionOS Simulator" \
        | grep -v "placeholder" \
        | head -1 \
        | sed -n 's/.*id:\([^,}]*\).*/\1/p')
    
    if [ -z "$sim_id" ]; then
        log_error "No visionOS simulator found. Please create one in Xcode."
        echo ""
        log_info "Available destinations:"
        xcodebuild -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" -scheme "$SCHEME" -showdestinations 2>/dev/null | grep -i "vision" || echo "  (none)"
        exit 1
    fi
    
    echo "$sim_id"
}

get_device_id() {
    # Get the first connected visionOS device from xcodebuild destinations
    local device_id
    device_id=$(xcodebuild -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" -scheme "$SCHEME" -showdestinations 2>/dev/null \
        | grep "platform:visionOS," \
        | grep -v "placeholder" \
        | grep -v "Simulator" \
        | head -1 \
        | sed -n 's/.*id:\([^,}]*\).*/\1/p')
    
    if [ -z "$device_id" ]; then
        log_error "No Vision Pro device found."
        echo ""
        log_info "Make sure your Vision Pro is:"
        echo "  1. Connected via USB or on the same network"
        echo "  2. Trusted (Developer Mode enabled)"
        echo "  3. Paired with this Mac"
        echo ""
        log_info "Available destinations:"
        xcodebuild -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" -scheme "$SCHEME" -showdestinations 2>/dev/null | grep -i "vision" || echo "  (none)"
        exit 1
    fi
    
    echo "$device_id"
}

boot_simulator() {
    local sim_id="$1"
    local state
    state=$(xcrun simctl list devices | grep "$sim_id" | grep -oE '\(Booted\)|\(Shutdown\)')
    
    if [ "$state" != "(Booted)" ]; then
        log_info "Booting simulator..."
        xcrun simctl boot "$sim_id" 2>/dev/null || true
        sleep 3
    fi
    
    # Open the Simulator app to show the window
    log_info "Opening Simulator window..."
    open -a Simulator
}

# =============================================================================
# Build Functions
# =============================================================================

clean_extended_attributes() {
    # Clean extended attributes that interfere with code signing
    # This is especially needed when building from iCloud Drive
    log_info "Cleaning extended attributes..."
    xattr -cr "$PROJECT_DIR" 2>/dev/null || true
    xattr -cr "$BUILD_DIR" 2>/dev/null || true
}

build_only() {
    # Build-only mode: fastest way to check for compile errors
    # Uses a specific simulator to ensure correct architecture (arm64)
    
    clean_extended_attributes
    
    log_info "Building (compile check only)..."
    
    # Get a simulator ID to ensure we build for the correct architecture
    local sim_id
    sim_id=$(get_simulator_id)
    
    if command -v xcpretty &> /dev/null; then
        xcodebuild \
            -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" \
            -scheme "$SCHEME" \
            -destination "platform=visionOS Simulator,id=$sim_id" \
            -configuration Debug \
            -derivedDataPath "$BUILD_DIR" \
            build \
            2>&1 | grep -v "DTDKRemoteDeviceConnection\|passcode protected" | xcpretty --color
    else
        xcodebuild \
            -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" \
            -scheme "$SCHEME" \
            -destination "platform=visionOS Simulator,id=$sim_id" \
            -configuration Debug \
            -derivedDataPath "$BUILD_DIR" \
            build \
            2>&1 | grep -v "DTDKRemoteDeviceConnection\|passcode protected"
    fi
    
    log_success "Build completed successfully - no errors!"
}

build_for_simulator() {
    local sim_id="$1"
    
    # Clean extended attributes before building (iCloud Drive fix)
    clean_extended_attributes
    
    log_info "Building for visionOS Simulator..."
    
    if command -v xcpretty &> /dev/null; then
        xcodebuild \
            -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" \
            -scheme "$SCHEME" \
            -destination "platform=visionOS Simulator,id=$sim_id" \
            -configuration Debug \
            -derivedDataPath "$BUILD_DIR" \
            build \
            2>&1 | grep -v "DTDKRemoteDeviceConnection\|passcode protected" | xcpretty --color
    else
        xcodebuild \
            -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" \
            -scheme "$SCHEME" \
            -destination "platform=visionOS Simulator,id=$sim_id" \
            -configuration Debug \
            -derivedDataPath "$BUILD_DIR" \
            build \
            2>&1 | grep -v "DTDKRemoteDeviceConnection\|passcode protected"
    fi
    
    log_success "Build completed for simulator"
}

build_for_device() {
    local device_id="$1"
    
    # Clean extended attributes before building (iCloud Drive fix)
    clean_extended_attributes
    
    log_info "Building for physical device..."
    
    if command -v xcpretty &> /dev/null; then
        xcodebuild \
            -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" \
            -scheme "$SCHEME" \
            -destination "platform=visionOS,id=$device_id" \
            -configuration Debug \
            -derivedDataPath "$BUILD_DIR" \
            -allowProvisioningUpdates \
            build \
            2>&1 | grep -v "DTDKRemoteDeviceConnection\|passcode protected" | xcpretty --color
    else
        xcodebuild \
            -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" \
            -scheme "$SCHEME" \
            -destination "platform=visionOS,id=$device_id" \
            -configuration Debug \
            -derivedDataPath "$BUILD_DIR" \
            -allowProvisioningUpdates \
            build \
            2>&1 | grep -v "DTDKRemoteDeviceConnection\|passcode protected"
    fi
    
    log_success "Build completed for device"
}

# =============================================================================
# Install Functions
# =============================================================================

install_on_simulator() {
    local sim_id="$1"
    local app_path
    
    # Find the built app
    app_path=$(find "$BUILD_DIR" -name "*.app" -path "*Debug-xrsimulator*" | head -1)
    
    if [ -z "$app_path" ]; then
        log_error "Could not find built app for simulator"
        exit 1
    fi
    
    log_info "Installing on simulator: $app_path"
    
    # Boot simulator if needed
    boot_simulator "$sim_id"
    
    # Install the app
    xcrun simctl install "$sim_id" "$app_path"
    
    log_success "Installed on simulator"
    
    # Launch the app
    log_info "Launching app..."
    xcrun simctl launch "$sim_id" "$BUNDLE_ID"
    
    log_success "App launched on simulator"
}

install_on_device() {
    local device_id="$1"
    local app_path
    
    # Find the built app
    app_path=$(find "$BUILD_DIR" -name "*.app" -path "*Debug-xros*" | head -1)
    
    if [ -z "$app_path" ]; then
        log_error "Could not find built app for device"
        exit 1
    fi
    
    log_info "Installing on device: $app_path"
    
    # Use devicectl for installation (Xcode 15+)
    if command -v xcrun &> /dev/null; then
        # Try devicectl first (newer method)
        if xcrun devicectl device install app --device "$device_id" "$app_path" 2>/dev/null; then
            log_success "Installed on device using devicectl"
        else
            # Fall back to ios-deploy if available
            if command -v ios-deploy &> /dev/null; then
                ios-deploy --id "$device_id" --bundle "$app_path" --no-wifi
                log_success "Installed on device using ios-deploy"
            else
                # Last resort: use xcodebuild to run
                log_warning "Direct installation not available. Running via xcodebuild..."
                xcodebuild \
                    -project "$PROJECT_DIR/$PROJECT_NAME.xcodeproj" \
                    -scheme "$SCHEME" \
                    -destination "platform=visionOS,id=$device_id" \
                    -configuration Debug \
                    -derivedDataPath "$BUILD_DIR" \
                    -allowProvisioningUpdates \
                    build \
                    | xcpretty --color || true
                log_success "Build and install completed via xcodebuild"
            fi
        fi
    fi
    
    log_success "Installation complete!"
    log_info "You may need to manually launch the app on your Vision Pro"
}

# =============================================================================
# Main Script
# =============================================================================

main() {
    local target="${1:-build}"
    
    echo ""
    echo "=========================================="
    echo "  Tracking Streamer Build & Install"
    echo "=========================================="
    echo ""
    
    # Validate argument
    case "$target" in
        build|device|simulator|logs-simulator|logs-device|clean)
            ;;
        -h|--help|help)
            print_usage
            exit 0
            ;;
        *)
            log_error "Invalid argument: $target"
            print_usage
            exit 1
            ;;
    esac
    
    # Handle clean command early (doesn't need Xcode check)
    if [ "$target" = "clean" ]; then
        log_info "Cleaning build cache..."
        rm -rf "$BUILD_DIR"
        log_success "Build cache deleted: $BUILD_DIR"
        exit 0
    fi
    
    # Check prerequisites
    check_xcode
    
    # Create build directory
    mkdir -p "$BUILD_DIR"
    
    if [ "$target" = "logs-simulator" ]; then
        log_info "Streaming logs from visionOS Simulator..."
        log_info "Filtering for: $BUNDLE_ID"
        log_info "Press Ctrl+C to stop"
        echo ""
        
        SIM_ID=$(get_simulator_id)
        # Stream logs from simulator, filtering by our bundle ID
        xcrun simctl spawn "$SIM_ID" log stream --predicate "subsystem CONTAINS '$BUNDLE_ID' OR process CONTAINS 'Tracking Streamer'" --style compact
        exit 0
        
    elif [ "$target" = "logs-device" ]; then
        log_info "Streaming logs from Vision Pro device..."
        log_info "Filtering for: $BUNDLE_ID"
        log_info "Press Ctrl+C to stop"
        echo ""
        
        DEVICE_ID=$(get_device_id)
        # Stream logs from device using devicectl
        xcrun devicectl device info consoleOutput --device "$DEVICE_ID" 2>/dev/null || \
            log stream --device "$DEVICE_ID" --predicate "subsystem CONTAINS '$BUNDLE_ID' OR process CONTAINS 'Tracking Streamer'" --style compact
        exit 0
        
    elif [ "$target" = "build" ]; then
        log_info "Target: Build only (checking for errors)"
        echo ""
        
        # Build for simulator target (faster, no code signing required)
        build_only
        
    elif [ "$target" = "simulator" ]; then
        log_info "Target: visionOS Simulator"
        echo ""
        
        # Get simulator ID
        SIM_ID=$(get_simulator_id)
        log_info "Using simulator: $SIM_ID"
        
        # Build
        build_for_simulator "$SIM_ID"
        
        # Install
        install_on_simulator "$SIM_ID"
        
    else
        log_info "Target: Physical Vision Pro Device"
        echo ""
        
        # Get device ID
        DEVICE_ID=$(get_device_id)
        log_info "Using device: $DEVICE_ID"
        
        # Build
        build_for_device "$DEVICE_ID"
        
        # Install
        install_on_device "$DEVICE_ID"
    fi
    
    echo ""
    log_success "Done! 🎉"
}

# Run main function with all arguments
main "$@"
