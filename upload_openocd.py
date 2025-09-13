Import("env")

firmware_path = env.get("TARGET")[0].get_abspath()

# Optional: print the actual path to confirm it's correct
print("Flashing firmware:", firmware_path)

openocd_cmd = [
    "openocd",
    "-f", "interface/cmsis-dap.cfg",
    "-f", "target/at91samdXX.cfg", #at91samd21g18
    "-c", f"program \"{firmware_path}\" verify reset exit"
]

# Run OpenOCD
env.Execute(" ".join(openocd_cmd))

#
# https://community.platformio.org/t/error-couldnt-open-pio-build-seeed-xiao-firmware-bin/36349/5
