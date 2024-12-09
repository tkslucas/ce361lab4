.global _boot
.text

_boot:
    li x1, 0           # Initialize x1 to 0
    li x2, 0           # Initialize x2 to 0
    jal x1, target     # Jump to "target", saving return address in x1
    addi x2, x2, 1     # Should not execute if jump is correct

target:
    addi x2, x2, 2     # x2 = 2 if jump is correct

end:
    j end              # Infinite loop

# Expected result:
# - x1 should contain the address of the instruction right after jal
# - x2 should be 2 if the JAL worked correctly.

