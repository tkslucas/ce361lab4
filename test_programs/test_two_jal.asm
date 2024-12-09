.global _boot
.text

_boot:
    li x1, 0           # Initialize x1 to 0
    li x2, 0           # Initialize x2 to 0
    jal x1, target     # Jump to "target", saving return address in x1
    addi x2, x2, 1     # Should not execute if jump is correct

back_from_target:
    addi x2, x2, 10    # If we get here, x2 should be set to 10
    j end              # Jump to end after test completes

target:
    addi x1, x1, 4     # Increment x1 to simulate processing at the target
    jal x0, back_from_target # Jump back without setting a return address

end:
    j end              # Infinite loop

# Expected result:
# - x1 should contain the address of "back_from_target"
# - x2 should be 10 if the JAL worked correctly.

