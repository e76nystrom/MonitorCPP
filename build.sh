#!/bin/bash

cd Debug
/usr/bin/make all
arm-none-eabi-objdump -h -S  MonitorCPP.elf  > "MonitorCPP.list"
