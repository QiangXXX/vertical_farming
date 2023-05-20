cmake_minimum_required(VERSION 3.20.0)

set(BOARD rpi_pico)

find_package(Zephyr)

project(sender)


set(OPENOCD "openocd")
set(OPENOCD_DEFAULT_PATH "/usr/local/share/openocd/scripts")

target_sources(app PRIVATE main.c)
