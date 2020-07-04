# STM32F7 External Memories Test

This is a simple application which maps two external memories and a display to an `STM32F7` chip's internal memory space. I wanted to learn how to use external memories with `STM32`s, and I was pleasantly surprised by how easy their memory-mapping peripherals are to use.

The target hardware is an `STM32F723IE` Discovery Kit:

https://www.st.com/en/evaluation-tools/32f723ediscovery.html

These boards have a traditional 8-pin Flash chip which the `QSPI` peripheral maps to `0x90000000`, and a parallel PSRAM chip which the `FMC` "Flexible Memory Controller" peripheral maps to `0x60000000`.

There's also a 240x240-pixel TFT display connected to "bank 2" of the `FMC` peripheral, but since its interface only supports "data" wires, its memory is accessed through two pointers. `0x64000000` is the "data" address, and `0x64000002` is the "command" address. In practice, it works about the same as a `SPI` display, only with 16 data lines instead of 1 and "read/write enable" wires instead of a synchronous clock signal.

This example uses `DMA` to send data from the external RAM to the external display, and it uses the TFT's "tearing effect" pin to minimize refresh artefacts by starting new transfers when the display controller is not refreshing its pixel data. It just pulses different solid colors for now.

Before the display is configured, a couple of memory writes and reads are performed on the `QSPI` Flash and external RAM. The results are printed over the debugger's `USART` serial connection.
