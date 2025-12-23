"""
Post-build script to merge ESP32 binary files into a single Intel HEX file.

This script combines bootloader.bin, partitions.bin, boot_app0.bin, and firmware.bin
into a single .hex file that can be flashed using standard programming tools.

Output filename format: Alink_{MCU}_{FREQ}_{MODULE}_333FullRes_{TAG}.hex
Example: Alink_ESP32S3_2400_TX_333FullRes_3.5.3.hex

ESP32 Flash Layout:
- bootloader.bin:  0x1000  (ESP32) / 0x0000 (ESP32-S3/C3)
- partitions.bin:  0x8000
- boot_app0.bin:   0xe000
- firmware.bin:    0x10000
"""

import os
import sys
import re
import subprocess


def git_cmd(*args):
    """Execute git command and return output."""
    try:
        return subprocess.check_output(["git"] + list(args), stderr=subprocess.DEVNULL).decode("utf-8").rstrip('\r\n')
    except:
        return None


def get_upstream_tag():
    """
    Get the latest tag from upstream (origin) repository.
    This works even when on a custom branch like 'Alink'.
    
    Returns:
        str: Version tag (e.g., "3.5.3") or "unknown" if not found
    """
    tag = "unknown"
    
    try:
        # Method 1: Try to get the most recent tag reachable from HEAD
        tag = git_cmd("describe", "--tags", "--abbrev=0")
        if tag:
            return tag
    except:
        pass
    
    try:
        # Method 2: Get all tags and find the latest version tag
        all_tags = git_cmd("tag", "-l", "--sort=-v:refname")
        if all_tags:
            # Filter for version tags (e.g., 3.5.3, v3.5.3)
            for t in all_tags.split('\n'):
                t = t.strip()
                if re.match(r'^v?\d+\.\d+\.\d+', t):
                    tag = t.lstrip('v')
                    return tag
    except:
        pass
    
    try:
        # Method 3: Try to fetch tags from origin and get latest
        git_cmd("fetch", "--tags", "--quiet")
        tag = git_cmd("describe", "--tags", "--abbrev=0")
        if tag:
            return tag.lstrip('v')
    except:
        pass
    
    return tag


def extract_target_info(env_name):
    """
    Extract MCU, frequency, and module type from environment name.
    
    Args:
        env_name: e.g., "Unified_ESP32S3_2400_TX_via_UART"
    
    Returns:
        tuple: (mcu, freq, module) e.g., ("ESP32S3", "2400", "TX")
    """
    # Remove "Unified_" prefix and "_via_*" suffix
    clean_name = re.sub(r'^Unified_', '', env_name)
    clean_name = re.sub(r'_via_.*$', '', clean_name, flags=re.IGNORECASE)
    
    # Pattern: {MCU}_{FREQ}_{MODULE}
    # Examples: ESP32S3_2400_TX, ESP32C3_900_RX, ESP32_2400_TX
    match = re.match(r'(ESP32[A-Z0-9]*|ESP8285)_(\d+)_(TX|RX)', clean_name, re.IGNORECASE)
    
    if match:
        mcu = match.group(1).upper()
        freq = match.group(2)
        module = match.group(3).upper()
        return (mcu, freq, module)
    
    # Fallback: return cleaned name parts
    parts = clean_name.split('_')
    if len(parts) >= 3:
        return (parts[0], parts[1], parts[2])
    
    return ("UNKNOWN", "UNKNOWN", "UNKNOWN")


def generate_output_filename(env_name, extension="hex"):
    """
    Generate output filename in format: Alink_{MCU}_{FREQ}_{MODULE}_333FullRes_V{TAG}.{ext}
    
    Args:
        env_name: PlatformIO environment name
        extension: File extension (hex or bin)
    
    Returns:
        str: Formatted filename
    """
    mcu, freq, module = extract_target_info(env_name)
    tag = get_upstream_tag()
    
    filename = f"Alink_{mcu}_{freq}_{module}_333FullRes_V{tag}.{extension}"
    return filename


def bin_to_hex_line(address, data):
    """Convert binary data to Intel HEX format lines."""
    lines = []
    offset = 0
    last_upper_addr = -1
    
    while offset < len(data):
        # Calculate chunk size (max 16 bytes per line)
        chunk_size = min(16, len(data) - offset)
        chunk = data[offset:offset + chunk_size]
        
        # Calculate current address
        current_addr = address + offset
        
        # Check if we need an extended address record (for addresses > 0xFFFF)
        upper_addr = (current_addr >> 16) & 0xFFFF
        if upper_addr != last_upper_addr:
            # Extended Linear Address Record (Type 04)
            ext_record = [2, 0, 0, 0x04, (upper_addr >> 8) & 0xFF, upper_addr & 0xFF]
            checksum = (~sum(ext_record) + 1) & 0xFF
            lines.append(":%02X%04X%02X%04X%02X" % (2, 0, 0x04, upper_addr, checksum))
            last_upper_addr = upper_addr
        
        # Use lower 16 bits for the data record
        record_addr = current_addr & 0xFFFF
        
        # Data Record (Type 00)
        record = [chunk_size, (record_addr >> 8) & 0xFF, record_addr & 0xFF, 0x00]
        record.extend(chunk)
        checksum = (~sum(record) + 1) & 0xFF
        
        hex_data = ''.join(['%02X' % b for b in chunk])
        lines.append(":%02X%04X%02X%s%02X" % (chunk_size, record_addr, 0x00, hex_data, checksum))
        
        offset += chunk_size
    
    return lines


def merge_bins_to_hex(build_dir, env_name, chip_type="esp32"):
    """
    Merge multiple binary files into a single Intel HEX file.
    
    Args:
        build_dir: Path to the build directory containing the bin files
        env_name: PlatformIO environment name for filename generation
        chip_type: ESP32 chip type (esp32, esp32s3, esp32c3)
    """
    
    # Define flash addresses based on chip type
    if chip_type in ["esp32s3", "esp32c3", "esp32-s3", "esp32-c3"]:
        bootloader_addr = 0x0000
    else:
        bootloader_addr = 0x1000
    
    partitions_addr = 0x8000
    boot_app0_addr = 0xe000
    firmware_addr = 0x10000
    
    # Define input files and their addresses
    files_config = [
        ("bootloader.bin", bootloader_addr),
        ("partitions.bin", partitions_addr),
        ("boot_app0.bin", boot_app0_addr),
        ("firmware.bin", firmware_addr),
    ]
    
    hex_lines = []
    
    output_filename = generate_output_filename(env_name, "hex")
    
    print("\n========== MERGING BIN TO HEX ==========")
    print(f"Build directory: {build_dir}")
    print(f"Environment: {env_name}")
    print(f"Chip type: {chip_type}")
    print(f"Output file: {output_filename}")
    print("-" * 40)
    
    for filename, address in files_config:
        filepath = os.path.join(build_dir, filename)
        
        if not os.path.exists(filepath):
            print(f"  [SKIP] {filename} not found")
            continue
        
        with open(filepath, 'rb') as f:
            data = f.read()
        
        file_size = len(data)
        print(f"  [OK] {filename}: 0x{address:05X} ({file_size} bytes)")
        
        # Convert to hex lines
        hex_lines.extend(bin_to_hex_line(address, list(data)))
    
    # Add End Of File record
    hex_lines.append(":00000001FF")
    
    # Write output hex file
    output_path = os.path.join(build_dir, output_filename)
    with open(output_path, 'w') as f:
        f.write('\n'.join(hex_lines))
    
    print("-" * 40)
    print(f"  Output: {output_path}")
    print(f"  Total lines: {len(hex_lines)}")
    print("========================================\n")
    
    return output_path


def merge_bins_to_single_bin(build_dir, env_name, chip_type="esp32"):
    """
    Merge multiple binary files into a single binary file.
    
    Args:
        build_dir: Path to the build directory containing the bin files
        env_name: PlatformIO environment name for filename generation
        chip_type: ESP32 chip type (esp32, esp32s3, esp32c3)
    """
    
    # Define flash addresses based on chip type
    if chip_type in ["esp32s3", "esp32c3", "esp32-s3", "esp32-c3"]:
        bootloader_addr = 0x0000
    else:
        bootloader_addr = 0x1000
    
    partitions_addr = 0x8000
    boot_app0_addr = 0xe000
    firmware_addr = 0x10000
    
    # Define input files and their addresses
    files_config = [
        ("bootloader.bin", bootloader_addr),
        ("partitions.bin", partitions_addr),
        ("boot_app0.bin", boot_app0_addr),
        ("firmware.bin", firmware_addr),
    ]
    
    output_filename = generate_output_filename(env_name, "bin")
    
    print("\n========== MERGING BIN FILES ==========")
    print(f"Build directory: {build_dir}")
    print(f"Environment: {env_name}")
    print(f"Chip type: {chip_type}")
    print(f"Output file: {output_filename}")
    print("-" * 40)
    
    # Find the end address to determine total size
    max_end = 0
    file_data = []
    
    for filename, address in files_config:
        filepath = os.path.join(build_dir, filename)
        
        if not os.path.exists(filepath):
            print(f"  [SKIP] {filename} not found")
            continue
        
        with open(filepath, 'rb') as f:
            data = f.read()
        
        file_size = len(data)
        end_addr = address + file_size
        max_end = max(max_end, end_addr)
        
        print(f"  [OK] {filename}: 0x{address:05X} - 0x{end_addr:05X} ({file_size} bytes)")
        file_data.append((address, data))
    
    # Create output buffer filled with 0xFF
    output_buffer = bytearray([0xFF] * max_end)
    
    # Copy each file to its position
    for address, data in file_data:
        output_buffer[address:address + len(data)] = data
    
    # Write output bin file
    output_path = os.path.join(build_dir, output_filename)
    with open(output_path, 'wb') as f:
        f.write(output_buffer)
    
    print("-" * 40)
    print(f"  Output: {output_path}")
    print(f"  Total size: {len(output_buffer)} bytes")
    print("========================================\n")
    
    return output_path


def post_build_merge(source, target, env):
    """PlatformIO post-build action to merge bin files."""
    build_dir = env['PROJECT_BUILD_DIR'] + '/' + env['PIOENV']
    env_name = env['PIOENV']
    target_name = env_name.upper()
    
    # Determine chip type from target name
    if "ESP32S3" in target_name:
        chip_type = "esp32s3"
    elif "ESP32C3" in target_name:
        chip_type = "esp32c3"
    else:
        chip_type = "esp32"
    
    # Generate both hex and merged bin
    merge_bins_to_hex(build_dir, env_name, chip_type)
    merge_bins_to_single_bin(build_dir, env_name, chip_type)


# For standalone usage
if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python merge_bin_to_hex.py <build_dir> <env_name> [chip_type]")
        print("  env_name: e.g., Unified_ESP32S3_2400_TX_via_UART")
        print("  chip_type: esp32 (default), esp32s3, esp32c3")
        sys.exit(1)
    
    build_dir = sys.argv[1]
    env_name = sys.argv[2]
    chip_type = sys.argv[3] if len(sys.argv) > 3 else "esp32"
    
    merge_bins_to_hex(build_dir, env_name, chip_type)
    merge_bins_to_single_bin(build_dir, env_name, chip_type)
