#!/bin/bash

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
ARDUINO_LIBS_PATH="${HOME}/Arduino/libraries"
LIBRARIES=(
    "DShotRMT:dev:https://github.com/derdoktor667/DShotRMT.git"
    "ESP32_MPU6050:dev:https://github.com/derdoktor667/ESP32_MPU6050.git"
    "FlyskyIBUS:dev:https://github.com/derdoktor667/FlyskyIBUS.git"
)

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

print_step() {
    echo -e "\n${BLUE}→ $1${NC}"
}

# Check if git is installed
check_git() {
    if ! command -v git &> /dev/null; then
        print_error "Git is not installed. Please install git first."
        exit 1
    fi
    print_success "Git found"
}

# Check if Arduino libraries directory exists
check_arduino_dir() {
    if [ ! -d "$ARDUINO_LIBS_PATH" ]; then
        print_step "Creating Arduino libraries directory..."
        mkdir -p "$ARDUINO_LIBS_PATH"
        print_success "Directory created at $ARDUINO_LIBS_PATH"
    else
        print_success "Arduino libraries directory exists at $ARDUINO_LIBS_PATH"
    fi
}

# Install a single library
install_library() {
    lib_name=$(echo "$1" | cut -d: -f1)
    branch=$(echo "$1" | cut -d: -f2)
    repo_url=$(echo "$1" | cut -d: -f3)
    lib_path="$ARDUINO_LIBS_PATH/$lib_name"
    
    print_step "Installing $lib_name (branch: $branch)..."
    
    # Check if library already exists
    if [ -d "$lib_path" ]; then
        print_info "Library $lib_name already exists. Updating..."
        cd "$lib_path"
        
        # Fetch latest changes
        if git fetch origin &> /dev/null; then
            # Checkout the desired branch
            if git checkout "$branch" &> /dev/null; then
                if git pull origin "$branch" &> /dev/null; then
                    print_success "$lib_name updated successfully"
                else
                    print_error "Failed to pull $lib_name"
                    return 1
                fi
            else
                print_error "Failed to checkout branch $branch for $lib_name"
                return 1
            fi
        else
            print_error "Failed to fetch updates for $lib_name"
            return 1
        fi
    else
        # Clone the library
        if git clone --depth 1 --branch "$branch" "$repo_url" "$lib_path" 2>&1 | grep -v "^Cloning into" | grep -v "^Unpacking objects" | grep -v "^Receiving objects"; then
            :
        fi
        
        if [ -d "$lib_path" ]; then
            print_success "$lib_name installed successfully"
        else
            print_error "Failed to install $lib_name"
            return 1
        fi
    fi
}

# Display installation summary
display_summary() {
    print_header "Installation Summary"
    echo "Arduino Libraries Path: $ARDUINO_LIBS_PATH"
    echo ""
    echo "Libraries to install:"
    for lib in "${LIBRARIES[@]}"; do
        lib_name=$(echo "$lib" | cut -d: -f1)
        branch=$(echo "$lib" | cut -d: -f2)
        echo "  • $lib_name (branch: $branch)"
    done
    echo ""
}

# Verify installation
verify_installation() {
    print_step "Verifying installation..."
    local all_ok=true
    
    for lib in "${LIBRARIES[@]}"; do
        lib_name=$(echo "$lib" | cut -d: -f1)
        lib_path="$ARDUINO_LIBS_PATH/$lib_name"
        
        if [ -d "$lib_path" ]; then
            print_success "$lib_name found"
        else
            print_error "$lib_name not found"
            all_ok=false
        fi
    done
    
    if [ "$all_ok" = true ]; then
        print_header "✓ All libraries installed successfully!"
        return 0
    else
        print_header "✗ Some libraries failed to install"
        return 1
    fi
}

# Show next steps
show_next_steps() {
    print_header "Next Steps"
    echo "1. Open Arduino IDE and navigate to Sketch → Include Library → Manage Libraries"
    echo "2. Verify that all libraries are listed:"
    for lib in "${LIBRARIES[@]}"; do
        lib_name=$(echo "$lib" | cut -d: -f1)
        echo "   • $lib_name"
    done
    echo ""
    echo "3. Open flight32.ino in the Arduino IDE"
    echo "4. Select your ESP32 board and COM port"
    echo "5. Click Upload"
    echo ""
    echo "For more information, see README.md"
}

###############################################################################
# Main Script
###############################################################################

main() {
    clear
    print_header "Flight32 Library Installer"
    
    # Initial checks
    print_step "Performing pre-flight checks..."
    check_git
    check_arduino_dir
    
    # Display summary
    display_summary
    
    # Ask for confirmation
    read -p "Continue with installation? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Installation cancelled"
        exit 0
    fi
    
    # Install libraries
    print_header "Installing Libraries"
    local failed_libs=()
    
    for lib in "${LIBRARIES[@]}"; do
        if ! install_library "$lib"; then
            lib_name=$(echo "$lib" | cut -d: -f1)
            failed_libs+=("$lib_name")
        fi
    done
    
    # Verify installation
    echo ""
    if [ ${#failed_libs[@]} -eq 0 ]; then
        verify_installation
        show_next_steps
        exit 0
    else
        verify_installation
        print_error "The following libraries failed to install:"
        for lib in "${failed_libs[@]}"; do
            echo "  • $lib"
        done
        exit 1
    fi
}

# Run main function
main "$@"
