#!/usr/bin/env python3
import sys
import os

def convert_to_little_endian(hex_string):
    # Remove any whitespace or newlines
    hex_string = hex_string.strip()
    # Convert the hex string to bytes
    big_endian_bytes = bytes.fromhex(hex_string)
    # Reverse the bytes to get little endian format
    little_endian_bytes = big_endian_bytes[::-1]
    # Convert back to hex with space-separated format
    return ' '.join(f'{byte:02x}' for byte in little_endian_bytes)

def process_file(input_file):
    # Extract the base name without extension for output file naming
    base_name = os.path.splitext(input_file)[0]
    output_file = f"{base_name}"
    
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            # Convert each line to little endian
            little_endian_hex = convert_to_little_endian(line)
            # Write the result to the output file
            outfile.write(little_endian_hex + '\n')
    
    print(f"Conversion complete. Output saved to: {output_file}")

# Main function to check command-line arguments
if __name__ == "__main__":
    # Ensure there is exactly one argument
    if len(sys.argv) != 2:
        print("Usage: python <program_name> <file>.hex.dump")
        sys.exit(1)
    
    # Get input file name from command-line argument
    input_file = sys.argv[1]
    
    # Check if the input file has the expected extension
    if not input_file.endswith('.hex.dump'):
        print("Error: Input file should have a '.hex.dump' extension")
        sys.exit(1)
    
    # Process the file
    process_file(input_file)

