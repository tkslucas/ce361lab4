RISCtakers_lab4: RISCtakers_lab4.v lab4_tb.v
	/vol/eecs362/iverilog-new/bin/iverilog -o RISCtakers_lab4 RISCtakers_lab4.v lab4_tb.v

run: RISCtakers_lab4
	./RISCtakers_lab4 +MEM_IN=test_programs/test_two_adds.hex

clean:
	rm RISCtakers_lab4
