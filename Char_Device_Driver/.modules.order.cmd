cmd_/home/work/KERNEL_MODULE/Char_Device_Driver/modules.order := {   echo /home/work/KERNEL_MODULE/Char_Device_Driver/char_device_driver.ko; :; } | awk '!x[$$0]++' - > /home/work/KERNEL_MODULE/Char_Device_Driver/modules.order