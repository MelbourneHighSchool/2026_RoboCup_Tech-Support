"""Firmware scan/publication and cross-language PCB layout regression tests."""

import subprocess
from pathlib import Path

from lib.pcb_layout import sensor_layout

ROOT = Path(__file__).resolve().parents[1]


def test_firmware_scan_and_python_layout(tmp_path):
    source = (ROOT / "STM32/Core/Src/main.c").read_text()

    def section(start, end):
        return source.split(start, 1)[1].split(end, 1)[0]

    # Compile the production state, selection helpers and callbacks with a tiny
    # host HAL. No duplicated implementation of scanning or publication.
    snippets = [
        "uint8_t TxData" + section("uint8_t TxData", "/* Shared between"),
        "static void select_multiplexer1" + section(
            "static void select_multiplexer1", "static void service_kick_cooldown"),
        "void HAL_I2C_AddrCallback" + section(
            "void HAL_I2C_AddrCallback", "void HAL_I2C_SlaveRxCpltCallback"),
        "void HAL_ADC_ConvCpltCallback" + section(
            "void HAL_ADC_ConvCpltCallback", "/* USER CODE END 0 */"),
    ]
    startup = section("/* USER CODE BEGIN 2 */", "/* USER CODE END 2 */")
    assert "select_multiplexer1();" in startup and "select_multiplexer2();" in startup
    (tmp_path / "stm32_scan_under_test.h").write_text("\n".join(snippets))
    binary = tmp_path / "stm32-scan"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
        "-I", str(tmp_path), "-I", str(ROOT / "STM32/Core/Inc"),
        str(ROOT / "tests/stm32_sensor_scan.c"), "-o", str(binary),
    ], check=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    native = []
    for line in result.stdout.splitlines():
        index, mux, pin, bearing = line.split()
        native.append({"index": int(index), "multiplexer": int(mux), "pin": int(pin),
                       "bearing_deg": float(bearing)})
    assert native == sensor_layout()["sensors"]
    assert len({(row["multiplexer"], row["pin"]) for row in native}) == 30
    assert {row["bearing_deg"] for row in native} == {
        position * 11.25 for position in range(32) if position not in (5, 6)
    }


def test_native_pcb_transport(tmp_path):
    binary = tmp_path / "pcb-native"
    subprocess.run([
        "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
        "-I", str(ROOT / "lib"), str(ROOT / "tests/pcb_native.cpp"),
        str(ROOT / "lib/linux_wire.cpp"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
