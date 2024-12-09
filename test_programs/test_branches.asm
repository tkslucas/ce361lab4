    .text
    .global _start

_start:
    li x1, 0xA       # Load 0xA (decimal 10) into x1
    li x2, 0x5       # Load 0x5 (decimal 5) into x2
    li x3, 0xA       # Load 0xA into x3 (for equality checks)
    
    # beq: Branch if x1 == x3 (x1 and x3 are both 0xA)
    beq x1, x3, beq_true
    li x4, 0x0       # If branch not taken, x4 = 0x0
    j next
beq_true:
    li x4, 0x1       # If branch taken, x4 = 0x1

next:
    # bne: Branch if x1 != x2 (0xA != 0x5)
    bne x1, x2, bne_true
    li x5, 0x0       # If branch not taken, x5 = 0x0
    j next2
bne_true:
    li x5, 0x1       # If branch taken, x5 = 0x1

next2:
    # blt: Branch if x2 < x1 (0x5 < 0xA)
    blt x2, x1, blt_true
    li x6, 0x0       # If branch not taken, x6 = 0x0
    j next3
blt_true:
    li x6, 0x1       # If branch taken, x6 = 0x1

next3:
    # bge: Branch if x1 >= x2 (0xA >= 0x5)
    bge x1, x2, bge_true
    li x7, 0x0       # If branch not taken, x7 = 0x0
    j next4
bge_true:
    li x7, 0x1       # If branch taken, x7 = 0x1

next4:
    # bltu: Branch if x2 < x1 (unsigned, 0x5 < 0xA)
    bltu x2, x1, bltu_true
    li x8, 0x0       # If branch not taken, x8 = 0x0
    j next5
bltu_true:
    li x8, 0x1       # If branch taken, x8 = 0x1

next5:
    # bgeu: Branch if x1 >= x2 (unsigned, 0xA >= 0x5)
    bgeu x1, x2, bgeu_true
    li x9, 0x0       # If branch not taken, x9 = 0x0
    j end
bgeu_true:
    li x9, 0x1       # If branch taken, x9 = 0x1

end:
    j end            # Infinite loop to end program

