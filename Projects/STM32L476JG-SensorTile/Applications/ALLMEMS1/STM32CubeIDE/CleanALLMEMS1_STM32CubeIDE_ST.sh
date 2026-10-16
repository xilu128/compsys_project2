#!/bin/bash
echo "Disabled: legacy bootloader layout. Flash the rebuilt ELF; BIN base is 0x08000000. See README.md." >&2
exit 1





CubeProg="STM32_Programmer_CLI"



echo "/******************************************/"
echo "           Clean FP-SNS-ALLMEMS1"
echo "/******************************************/"
echo "              Full Chip Erase"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -e all
echo "/******************************************/"
echo "              Install BootLoader"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -d ../../../../../Utilities/BootLoader/STM32L476RG/BootLoaderL4.bin 0x08000000 -v
echo "/******************************************/"
echo "          Install FP-SNS-ALLMEMS1"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -d ./Release/STM32L476JG-SensorTile_ALLMEMS1.bin 0x08004000 -v
echo "/******************************************/"
echo "      Dump FP-SNS-ALLMEMS1 + BootLoader"
echo "/******************************************/"
SizeBinBL=`ls -l ./Release/STM32L476JG-SensorTile_ALLMEMS1.bin | awk '{print $5+0x4000};'`
"${CubeProg}" -c port=swd mode=UR reset=HWrst -u 0x08000000 ${SizeBinBL} ./Release/STM32L476JG-SensorTile_ALLMEMS1_BL.bin
echo "/******************************************/"
echo "                Reset STM32"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -rst
