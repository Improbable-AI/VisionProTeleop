#!/bin/bash

# Script to build Sphinx documentation for VisionProTeleop
# This script builds the HTML documentation from RST source files

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building VisionProTeleop Documentation${NC}"
echo "========================================"

# Check if sphinx-build is installed
if ! command -v sphinx-build &> /dev/null; then
    echo -e "${RED}Error: sphinx-build not found${NC}"
    echo "Please install Sphinx and the RTD theme:"
    echo "  pip install sphinx sphinx_rtd_theme"
    exit 1
fi

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
DOCS_DIR="$SCRIPT_DIR/docs"
SOURCE_DIR="$DOCS_DIR/source"
BUILD_DIR="$DOCS_DIR/build"

# Check if source directory exists
if [ ! -d "$SOURCE_DIR" ]; then
    echo -e "${RED}Error: Source directory not found: $SOURCE_DIR${NC}"
    exit 1
fi

# Clean previous build
if [ -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}Cleaning previous build...${NC}"
    rm -rf "$BUILD_DIR"
fi

# Create build directory
mkdir -p "$BUILD_DIR"

# Build the documentation
echo -e "${GREEN}Building HTML documentation...${NC}"
sphinx-build -b html "$SOURCE_DIR" "$BUILD_DIR/html"

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ Documentation built successfully!${NC}"
    echo ""
    echo "Documentation location:"
    echo "  $BUILD_DIR/html/index.html"
    echo ""
    echo "To view the documentation, open in your browser:"
    echo -e "  ${YELLOW}open $BUILD_DIR/html/index.html${NC}"
    echo ""
else
    echo -e "${RED}✗ Documentation build failed${NC}"
    exit 1
fi
