all:
	gcc wavecatcher.c -lpthread -lusb-1.0 -o wavecatcher
	gcc -g process.c -o process
	@$(MAKE)  -f Makefile.stm32

clean:
	rm -f *.elf
	rm -f wavecatcher
