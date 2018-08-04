sudo openocd -f openocd-jlink-swd.cfg \
	-c "program firmware.elf verify; reset run; exit"
