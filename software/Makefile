all:
	gcc usb.c -lpthread -lusb-1.0 -o wavecatcher

	@$(MAKE)  -f Makefile.stm32

clean:
	rm -f *.elf
	rm -f wavecatcher
