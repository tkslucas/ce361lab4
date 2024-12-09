.global _boot
.text

_boot:
    li x1, 0           # Initialize x1 to 0
    li x2, 12          # Initialize x2 to 0
    jalr x1, x2, 4     # Jump to end, saving return address in x1
    li x3, 1           # Should not execute if jump is correct

end:
    j end              # Infinite loop

# Expected result:
# - x1 should contain the address of the instruction right after jal
# - x2 should be 2 if the JAL worked correctly.

