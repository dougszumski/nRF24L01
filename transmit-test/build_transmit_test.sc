#Crude build script
arm-angstrom-linux-gnueabi-gcc  -o transmit_test transmit_test.c gpio.c threaded_isr.c spi.c radio.c -I/home/doug/OE_Legacy/mini2440_kernel/include/ -lpthread
cp transmit_test /home/doug/mini2440/root_fs/

#-mabi=aapcs
