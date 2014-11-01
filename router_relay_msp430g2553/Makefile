	# Makefile
PHONY: all
all:
	msp430-gcc -Os -mmcu=msp430g2553 -o main.elf main.c


	msp430-objcopy -O ihex main.elf main.hex

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~


flash: all
	MSP430Flasher.exe -n msp430g2xx3 -w "main.hex" -v -z [VCC] -m SBW2