#!/usr/bin/env -S just --justfile
# Cross platform shebang:

shebang := if os() == 'windows' { 'pwsh.exe' } else { '/usr/bin/env bash' }

#  '/usr/bin/env -S pwsh -noprofile -nologo'

set shell := ["/usr/bin/env", "-S", "pwsh", "-noprofile", "-nologo", "-c"]

# set shell := ["/usr/bin/env", "bash" ,"-c"]

set windows-shell := ["pwsh.exe", "-NoLogo", "-noprofile", "-c"]

build-all:
    /usr/local/bin/pio run --environment esp_controller
    /usr/local/bin/pio run --environment xdrive_foc_0
    /usr/local/bin/pio run --environment b_g431b_esc1_foc_1
    /usr/local/bin/pio run --environment b_g431b_esc1_foc_0
    /usr/local/bin/pio run --environment odesc_foc_0

build device="esp_controller":
    if ("{{ device }}" -eq "esp_controller") { /usr/local/bin/pio run --environment esp_controller }
    if ("{{ device }}" -eq "esp_foc_0") { /usr/local/bin/pio run --environment esp_foc_0 }
    if ("{{ device }}" -eq "bg431besc1") { /usr/local/bin/pio run --environment b_g431b_esc1_foc_0 }
    if ("{{ device }}" -eq "bg431besc2") { /usr/local/bin/pio run --environment b_g431b_esc1_foc_1 }
    if ("{{ device }}" -eq "xdrive") { /usr/local/bin/pio run --environment xdrive_foc_0 }
    if ("{{ device }}" -eq "odesc_0") { /usr/local/bin/pio run --environment odesc_foc_0 }

upload device:
    if ("{{ device }}" -eq "esp_controller") { /usr/local/bin/pio run -t upload --upload-port /dev/esp32/esp_controller -e esp_controller -d /home/mosthated/_dev/languages/esp32/xdrive_mini/foc/ }
    if ("{{ device }}" -eq "esp_foc_0") { /usr/local/bin/pio run -t upload --upload-port /dev/esp32/esp_foc_0 -e esp_foc_0 -d /home/mosthated/_dev/languages/esp32/xdrive_mini/foc/ }
    if ("{{ device }}" -eq "bg431besc1") { /usr/local/bin/pio run -t upload --upload-port /dev/esp32/bg431besc1 -e b_g431b_esc1_foc_0 -d /home/mosthated/_dev/languages/esp32/xdrive_mini/foc/ }
    if ("{{ device }}" -eq "bg431besc1_1") { /usr/local/bin/pio run -t upload --upload-port /dev/esp32/bg431besc1_1 -e b_g431b_esc1_foc_1 -d /home/mosthated/_dev/languages/esp32/xdrive_mini/foc/ }
    if ("{{ device }}" -eq "xdrive") { /usr/local/bin/pio run -t upload --upload-port /dev/esp32/xdrive -e xdrive_foc_0 -d /home/mosthated/_dev/languages/esp32/xdrive_mini/foc/ }
    if ("{{ device }}" -eq "odesc_0") { just set-home-position 1 && /usr/local/bin/pio run -t upload --upload-port /dev/esp32/odesc_0 -e odesc_foc_0 -d /home/mosthated/_dev/languages/esp32/xdrive_mini/foc/ }
    # if ("{{ device }}" -eq "xdrive") { just set-home-position 1 && /usr/local/bin/pio run -t upload --upload-port /dev/esp32/xdrive -e xdrive_foc_0 -d /home/mosthated/_dev/languages/esp32/xdrive_mini/foc/ }

# controller - /dev/esp32/esp_controller
# xdrive - /dev/esp32/xdrive

monitor device="xdrive":
    echo "Device: {{ device }}"

    if ("{{ device }}" -eq "esp_controller") { /usr/local/bin/pio device monitor -e esp_controller -p /dev/esp32/esp_controller }
    if ("{{ device }}" -eq "esp_foc_0") { /usr/local/bin/pio device monitor -e esp_foc_0 -p /dev/esp32/esp_foc_0 }
    if ("{{ device }}" -eq "xdrive") { /usr/local/bin/pio device monitor -e xdrive_foc_0 -p /dev/esp32/xdrive }
    if ("{{ device }}" -eq "odesc_0") { /usr/local/bin/pio device monitor -e odesc_foc_0 -p /dev/esp32/odesc_0 }
    if ("{{ device }}" -eq "bg431besc1") { /usr/local/bin/pio device monitor -e b_g431b_esc1_foc_0 -p /dev/esp32/bg431besc1 }
    if ("{{ device }}" -eq "bg431besc1_1") { /usr/local/bin/pio device monitor -e b_g431b_esc1_foc_1 -p /dev/esp32/bg431besc1_1 }

upload-monitor device:
    just upload {{device}} && sleep 2 && just monitor {{device}}    

set-home-position motor="1":
    /home/mosthated/_dev/languages/esp32/controller_ui/send.py set_target_to 0 {{motor}}
    sleep 1
