    .text
    .global _start

_start:
    # lui: Load upper immediate into x10 with a 20-bit immediate value
    # This loads 0x12345 into the upper 20 bits of x10 (result: 0x12345000)
    lui x10, 0x12345       # Expected: x10 = 0x12345000

    # auipc: Add upper immediate to PC and store in x11
    # If the current PC at this instruction is 0x00000004, it will add 0x20000 to it
    # Result will depend on the exact PC address, but conceptually adds the 20-bit immediate to PC
    auipc x11, 0x20000     # Expected: x11 = current PC + 0x20000000

    # To check auipc behavior, use another auipc to add a small offset for verification
    auipc x12, 0x0         # This should set x12 to the current PC value (aligned to 4-byte)

end:
    j end                  # Infinite loop to end the program

