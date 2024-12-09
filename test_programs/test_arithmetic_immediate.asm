    .text
    .global _start

_start:
    # Initialize some registers for testing in the register file
    # x1 = 5 (used in several operations)
    # x2 = 10 (used in several operations)
    # x3 = 15 (used in several operations)
    # x4 = 20 (used in several operations)

    # ADDI: Add immediate value (x5 = x1 + 3)
    addi x5, x1, 3     # x5 = 5 + 3 = 8

    # XORI: Bitwise XOR immediate (x6 = x2 ^ 5)
    xori x6, x2, 5     # x6 = 10 ^ 5 = 15

    # ORI: Bitwise OR immediate (x7 = x3 | 7)
    ori x7, x3, 7      # x7 = 15 | 7 = 15

    # ANDI: Bitwise AND immediate (x8 = x4 & 5)
    andi x8, x4, 5     # x8 = 20 & 5 = 4

    # SLLI: Shift left logical immediate (x9 = x1 << 2)
    slli x9, x1, 2     # x9 = 5 << 2 = 20

    # SRLI: Shift right logical immediate (x10 = x2 >> 1)
    srli x10, x2, 1    # x10 = 10 >> 1 = 5

    # SRAI: Shift right arithmetic immediate (x11 = x3 >> 2)
    srai x11, x3, 2    # x11 = 15 >> 2 = 3 (arithmetic shift)

    # SLTI: Set if less than immediate (x12 = 1 if x1 < 7, otherwise 0)
    slti x12, x1, 7    # x12 = 1 (since 5 < 7)

    # SLTIU: Set if unsigned less than immediate (x13 = 1 if x2 < 15, otherwise 0)
    sltiu x13, x2, 15  # x13 = 1 (since 10 < 15)

end:
    j end              # Infinite loop to end the program

