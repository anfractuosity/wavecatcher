sudo openocd -f openocd-jlink-swd.cfg \
	-c "program cdcacm.elf verify; reset run; exit"
