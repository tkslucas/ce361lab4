_start:
    # Initialize some values in memory for testing
    li x1, 0x100                 # Load base address 0x100 into x1
    li x2, 0x12345678            # Test value for word (32-bit)
    sw x2, 0(x1)                 # Store word x2 at address 0x100
    li x3, 0x89AB                # Test value for half-word (16-bit)
    sh x3, 4(x1)                 # Store half-word x3 at address 0x104
    li x4, 0xCD                  # Test value for byte (8-bit)
    sb x4, 6(x1)                 # Store byte x4 at address 0x106

    # Load the values back using different instructions
    lw x5, 0(x1)                 # Load word from address 0x100 into x5
    lh x6, 4(x1)                 # Load signed half-word from address 0x104 into x6
    lhu x7, 4(x1)                # Load unsigned half-word from address 0x104 into x7
    lb x8, 6(x1)                 # Load signed byte from address 0x106 into x8
    lbu x9, 6(x1)                # Load unsigned byte from address 0x106 into x9

end:
    j end                        # Infinite loop to halt the program

