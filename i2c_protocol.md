# I2C Protocol
This protocol is used to efficiently send messages between our Raspberry Pi and our PCB.

The below descriptions of the Start and Stop Flags and Address Frame follow the I2C standard. More information can be found in Section 3 of [A Basic Guide to I2C](https://www.ti.com/lit/an/sbaa565/sbaa565.pdf?ts=1789115277946). If you are familiar with this, you can skip to the [Data Frame](#data-frame).

## Start and Stop Flags
The Pi will begin all communications with a START flag. This is done by pulling SDA down first, then pulling SCL down. After communication is completed, the Pi will send a STOP flag by releasing SCL first, then releasing SDA.

## Address Frame
At the start of each message, the Raspberry Pi will send an address frame to the PCB's I2C address, `0x37`. This begins with the START flag, by pulling SDA low first, then pulling SCL low, then it will send the address MSB first. After the address, it send the read-write bit. A read command means that the Pi wants to know the current state of every phototransistor. A write command means that the Pi wants the PCB execute some command. The PCB will then pull SDA low as an ACK. If the ACK bit is missing, the Pi will immediately halt communication and report an error.

## Data Frame
Each data frame consists of one data byte (MSB first) followed by an ACK.
If the read-write bit was a 1, the Pi will expect the PCB to send over the phototransistor information over 32 data bytes, with an ACK from the Pi after each one. If the read-write bit was a 0, the Pi will send a command in 1 data frame.

### Commands
If the data byte is `0xFF`, the Pi is sending a kick command. For any other byte the Pi is setting a target brightness for the LED ring, where `0x00` is 0% and `0xFE` is 100%.

### Read
Whenever the Pi requests a read, the PCB should send a full dump of all of the phototransistor voltages. This consists of 32 data bytes, corresponding to the 32 phototransistors, starting with the front and moving around clockwise. Each byte represents the voltage of that phototransistor, where `0x00` represents 0V and `0xFF` represents 3.3V. This means that 1LSB ≈ 0.013V.
