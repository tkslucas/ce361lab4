    .text
    .global _start

_start:
	# In register file
    # Load 0xA (decimal 10) into x1
    # Load 0x5 (decimal 5) into x2

    add x3, x1, x2  # x3 = x1 + x2 = 0xA + 0x5 = 0xF
    sub x4, x1, x2  # x4 = x1 - x2 = 0xA - 0x5 = 0x5
    xor x5, x1, x2  # x5 = x1 ^ x2 = 0xA ^ 0x5 = 0xF
    or  x6, x1, x2  # x6 = x1 | x2 = 0xA | 0x5 = 0xF
    and x7, x1, x2  # x7 = x1 & x2 = 0xA & 0x5 = 0x0

    sll x8, x1, 2   # x8 = x1 << 2 = 0xA << 2 = 0x28
    srl x9, x1, 2   # x9 = x1 >> 2 = 0xA >> 2 = 0x2
    sra x10, x1, 2  # x10 = x1 >>> 2 = 0xA >>> 2 = 0x2 (arithmetic shift)
    
    slt x11, x1, x2 # x11 = (x1 < x2) ? 1 : 0 = (0xA < 0x5) ? 0 : 1 = 0x0
    sltu x12, x1, x2 # x12 = (unsigned)(x1) < (unsigned)(x2) ? 1 : 0 = (0xA < 0x5) ? 0 : 1 = 0x0

end:
    j end           # Infinite loop to end program

