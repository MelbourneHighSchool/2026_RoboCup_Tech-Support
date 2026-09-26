# I2C Protocol
This protocol is used to efficiently send messages between our Raspberry Pi and our PCB.

The below descriptions of the Start and Stop Flags and Address Frame follow the I2C standard. More information can be found in Section 3 of [A Basic Guide to I2C](https://www.ti.com/lit/an/sbaa565/sbaa565.pdf?ts=1789115277946). If you are familiar with this, you can skip to the [Data Frame](#data-frame).

## Start and Stop Flags
The Pi will begin all communications with a START flag. This is done by pulling SDA down first, then pulling SCL down. After communication is completed, the Pi will send a STOP flag by releasing SCL first, then releasing SDA.

## Address Frame
At the start of each message, the Raspberry Pi will send an address frame to the PCB's I2C address, `0x37`. This begins with the START flag, by pulling SDA low first, then pulling SCL low, then it will send the address MSB first. After the address, it send the read-write bit. A read command means that the Pi wants to know the current state of every phototransistor. A write command means that the Pi wants the PCB execute some command. The PCB will then pull SDA low as an ACK. If the ACK bit is missing, the Pi will immediately halt communication and report an error.

## Data Frame
Each data frame consists of data bytes (MSB first), each followed by an ACK.
If the read-write bit was a 1, the Pi expects 30 phototransistor data bytes. The Pi ACKs the first 29 bytes, then NACKs the final byte and sends STOP. If the read-write bit was a 0, the Pi sends a command in 1 data byte with an ACK from the PCB after.

### Commands
If the data byte is `0xFF`, the Pi is sending a kick command. For any other byte the Pi is setting a target brightness for the LED ring, where `0x00` is 0% and `0xFE` is 100%.

### Read
The Pi performs a plain 30-byte read at `0x37`, without a register-selection write (a write would execute a command). Each byte represents one working phototransistor voltage: `0x00` is 0V and `0xFF` is 3.3V, so 1 LSB ≈ 0.013V. The firmware snapshots the last complete scan for the entire transaction; before the first complete scan the buffer contains zeros.

Multiplexers are numbered **1 and 2**; their pins and the transmitted byte indices are **zero-based**. The forward sensor is **multiplexer 2 pin 9**. Readings are packed clockwise, with no bytes for disconnected or broken inputs:

- Bytes **0–4**: multiplexer 2 pins **9–13**.
- Byte **5**: multiplexer 2 pin **15**.
- Bytes **6–20**: multiplexer 1 pins **1–15**.
- Bytes **21–29**: multiplexer 2 pins **0–8**; the next sensor wraps to byte 0.

**Do not sample multiplexer 1 pin 0 or multiplexer 2 pin 14.** There are 15 working inputs per multiplexer and 30 transmitted readings. The ring still has 32 physical positions, spaced 11.25° apart: the two dead positions are immediately clockwise of multiplexer 2 pin 13, at **56.25° and 67.5°**. They are not transmitted. Multiplexer 2 pin 15 is at **78.75°** and multiplexer 1 pin 1 follows immediately at **90°**, with no additional gap at the multiplexer boundary.

For transmitted byte index `i`, the clockwise bearing relative to robot-forward is:

```text
bearing_deg = (i < 5 ? i : i + 2) * 11.25
```

The firmware applies the wiring offset; the Pi must not rotate the readings again. Firmware, native transport and localisation share `STM32/Core/Inc/pcb_sensor_layout.h`. The dashboard's hardware-independent Python metadata in `lib/pcb_layout.py` is checked against it by the mapping tests.

This is a **30-byte protocol**, incompatible with the previous 32-byte stream. Flash the firmware and update/rebuild both Pi extensions together; there is no automatic format negotiation. Brightness and kick commands are unchanged.
