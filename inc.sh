#!/bin/bash

python3 ../LatheCPP/inc.py src/lclcmd.cpp include/lclcmd.h
python3 ../LatheCPP/inc.py src/serialio.cpp include/serialio.h
python3 ../LatheCPP/inc.py src/current.cpp include/current.h
python3 ../LatheCPP/inc.py ../LatheCPP/lathe_src/stm32Info.cpp include/stm32Info.h
