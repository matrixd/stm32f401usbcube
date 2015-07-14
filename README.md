# stm32f401usbcube
Simple vcp example for stm32f401 using stm32cube.

It works like a basic echo over usb. Device will be detected as ttyACM.

How to use:
 - compile a code from repo
 - configure your ttyACMX using stty `stty raw -echo -F /dev/ttyACM0`

To listen from ttyACM `cat /dev/ttyACM0`
To send information `echo "some messge" > /dev/ttyACM0`
