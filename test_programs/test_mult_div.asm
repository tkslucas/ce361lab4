_start:
    li x5, 0x0000000A          # Load x5 with 10 (hexadecimal 0x0000000A)
    li x6, 0xFFFFFFF6          # Load x6 with -10 (hexadecimal 0xFFFFFFF6, two's complement)

    mul x7, x5, x6             # Signed multiplication: x7 = x5 * x6
    mulh x8, x5, x6            # Signed high multiplication: x8 = high part of x5 * x6
    mulhsu x9, x5, x6          # Mixed signed/unsigned high multiplication: x9 = high part of x5 * unsigned(x6)
    mulhu x10, x5, x6          # Unsigned high multiplication: x10 = high part of unsigned(x5) * unsigned(x6)

    div x11, x5, x6            # Signed division: x11 = x5 / x6
    divu x12, x5, x6           # Unsigned division: x12 = unsigned(x5) / unsigned(x6)
    rem x13, x5, x6            # Signed remainder: x13 = x5 % x6
    remu x14, x5, x6           # Unsigned remainder: x14 = unsigned(x5) % unsigned(x6)

end:
    j end                      # Infinite loop to halt the program

