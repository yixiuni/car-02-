"""Check the firmware against the independent C10B schematic pin contract."""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def read(name):
    return (ROOT / name).read_text(encoding="utf-8")

board = read("Core/Inc/board_c10b.h")
definitions = dict(re.findall(r"^#define\s+(BOARD_\w+)\s+(\w+)", board, re.M))
expected = {
    "MOTOR_A": ("GPIOA", {"IN1": 6, "IN2": 7}),
    "MOTOR_B": ("GPIOB", {"IN1": 0, "IN2": 1}),
    "BUTTON": ("GPIOA", {"": 0}),
    "ENABLE": ("GPIOC", {"": 13}),
    "LED": ("GPIOC", {"RUN": 2, "FAULT": 3}),
    "GRAY": ("GPIOC", {"TX": 10, "RX": 11}),
    "MPU": ("GPIOB", {"SCL": 14, "SDA": 15}),
}
owners = {}
for peripheral, (port, pins) in expected.items():
    assert definitions[f"BOARD_{peripheral}_PORT"] == port
    for signal, pin in pins.items():
        suffix = f"_{signal}" if signal else ""
        assert definitions[f"BOARD_{peripheral}{suffix}_PIN"] == f"GPIO_PIN_{pin}"
        assert (port, pin) not in owners, f"Pin conflict: {port}.{pin}"
        assert not (port == "GPIOA" and pin in (13, 14)), "SWD pin reassigned"
        owners[(port, pin)] = peripheral

ioc = dict(line.split("=", 1) for line in read("stm32-car.ioc").splitlines() if "=" in line)
for key, value in {
    "Mcu.CPN": "STM32F103RCT6", "Mcu.Package": "LQFP64",
    "PA0.Signal": "GPIO_Input", "PA0.GPIO_PuPd": "GPIO_PULLDOWN",
    "PC13-TAMPER-RTC.Signal": "GPIO_Input", "PC13-TAMPER-RTC.GPIO_PuPd": "GPIO_PULLUP",
    "PC2.Signal": "GPIO_Output", "PC3.Signal": "GPIO_Output",
    "PA6.Signal": "S_TIM3_CH1", "PA7.Signal": "S_TIM3_CH2",
    "PB0.Signal": "S_TIM3_CH3", "PB1.Signal": "S_TIM3_CH4",
    "PC10.Signal": "UART4_TX", "PC11.Signal": "UART4_RX",
    "UART4.BaudRate": "115200", "TIM3.Period": "3599",
    "PB14.GPIO_ModeDefaultOutputPP": "GPIO_MODE_OUTPUT_OD",
    "PB15.GPIO_ModeDefaultOutputPP": "GPIO_MODE_OUTPUT_OD",
    "PA13.Signal": "SYS_JTMS-SWDIO", "PA14.Signal": "SYS_JTCK-SWCLK",
}.items():
    assert ioc.get(key) == value, f"IOC mismatch: {key}"

cmake = read("cmake/stm32cubemx/CMakeLists.txt")
assert "STM32F103xE" in cmake and "STM32F103xB" not in cmake
assert "startup_stm32f103xe.s" in cmake
linker = read("STM32F103RCTx_FLASH.ld")
assert "LENGTH = 48K" in linker and "LENGTH = 256K" in linker
for path in ("cmake/gcc-arm-none-eabi.cmake", "cmake/starm-clang.cmake"):
    assert "STM32F103RCTx_FLASH.ld" in read(path)
for path, key in ((".vscode/launch.json", "deviceName"), (".settings/ide.store.json", "device")):
    data = json.loads(read(path))
    if key == "deviceName": data = data["configurations"][0]
    assert data[key] == "STM32F103RCT6"
startup = read("startup_stm32f103xe.s")
vectors = re.findall(r"^\s*\.word\s+(\w+)", startup.split("g_pfnVectors:", 1)[1], re.M)
device = read("Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h")
irq = int(re.search(r"UART4_IRQn\s*=\s*(\d+)", device)[1])
assert vectors[16 + irq] == "UART4_IRQHandler", "Wrong IRQ vector position"
assert "HiwonderGray_IRQHandler();" in read("Core/Src/stm32f1xx_it.c")
assert "GPIO_PIN_13, GPIO_PIN_RESET" not in read("Core/Src/main.c")
print("PASS: C10B pin ownership, IOC, F103RCT6 target, memory layout, UART4 vector, SWD")
