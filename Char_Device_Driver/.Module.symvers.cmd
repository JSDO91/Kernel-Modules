cmd_/home/work/KERNEL_MODULE/Char_Device_Driver/Module.symvers := sed 's/\.ko$$/\.o/' /home/work/KERNEL_MODULE/Char_Device_Driver/modules.order | scripts/mod/modpost -m -a  -o /home/work/KERNEL_MODULE/Char_Device_Driver/Module.symvers -e -i Module.symvers   -T -