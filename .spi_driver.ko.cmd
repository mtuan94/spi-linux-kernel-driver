cmd_/home/pi/spi-linux-kernel-driver/spi_driver.ko := ld -EL -r  -T ./scripts/module-common.lds --build-id  -o /home/pi/spi-linux-kernel-driver/spi_driver.ko /home/pi/spi-linux-kernel-driver/spi_driver.o /home/pi/spi-linux-kernel-driver/spi_driver.mod.o