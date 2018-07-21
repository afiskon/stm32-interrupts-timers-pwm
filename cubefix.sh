#!/bin/sh

# Fix project after updating it with STM32CubeMX

set -e

cp Makefile.backup Makefile
dos2unix Src/main.c
dos2unix Src/stm32f4xx_it.c
sed -i -e '1,39d' Src/main.c
