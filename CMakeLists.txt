cmake_minimum_required(VERSION 3.8.2)

set(BOARD nrf52_adafruit_feather)

# DTC overlays are per-board; each board must define a boards/$BOARD.overlay.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}.overlay)
  set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}.overlay)
endif()

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(bt_controller)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE 
  ${app_sources}
  $ENV{ZEPHYR_BASE}/samples/bluetooth/gatt/bas.c
  $ENV{ZEPHYR_BASE}/samples/bluetooth/gatt/dis.c
  $ENV{ZEPHYR_BASE}/samples/bluetooth/gatt/hog.c
)

zephyr_link_libraries_ifdef(CONFIG_FILE_SYSTEM_NFFS NFFS)

zephyr_library_include_directories($ENV{ZEPHYR_BASE}/samples/bluetooth)