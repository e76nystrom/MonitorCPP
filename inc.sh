#!/bin/bash

python3 ../LatheCPP/inc.py Src/lclcmd.cpp include/lclcmd.h
python3 ../LatheCPP/inc.py Src/serialio.cpp include/serialio.h
python3 ../LatheCPP/inc.py Src/current.cpp include/current.h
python3 ../LatheCPP/inc.py ../LatheCPP/lathe_src/stm32Info.cpp include/stm32Info.h
