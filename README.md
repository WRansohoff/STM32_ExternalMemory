# STM32F7 External Memories Test

This is a simple application which maps two external memories and a display to an `STM32F7` chip's internal memory space. I wanted to learn how to use external memories with `STM32`s, and I was pleasantly surprised by how easy their memory-mapping peripherals are to use.

The target hardware is an `STM32F723IE` Discovery Kit:

https://www.st.com/en/evaluation-tools/32f723ediscovery.html

These boards have a traditional 8-pin Flash chip which the `QSPI` peripheral maps to `0x90000000`, and a parallel PSRAM chip which the `FMC` "Flexible Memory Controller" peripheral maps to `0x60000000`.

This example runs a couple of memory read/writes on the external RAM and Flash, and prints the results over the debugger's `USART` serial connection at 115200 baud.

There's also a 240x240-pixel TFT display connected to "bank 2" of the `FMC` peripheral, but since its interface only supports "data" wires, its registers and display memory are accessed through two pointers. `0x64000000` is the "data" address, and `0x64000002` is the "command" address. In practice, it works about the same as a `SPI` TFT display; the signals just use a parallel interface instead of a serial one.

This example uses "memory-to-memory" `DMA` to quickly send data from the external RAM to the external display. Each frame, it clears the framebuffer to a new color and prints a message including the current framerate.

It also enables an interrupt connected to the TFT's "tearing effect" pin and includes commented-out code which synchronizes data transfers to the display's "vblank" period. But the example runs ~8fps slower if it waits for the "vblank" interrupt to trigger new DMA transfers, and I didn't notice any tearing or artefacts when I ignored it.

I'm sure there are a few ways to speed things up, but even with the entire framebuffer and display getting refreshed every frame, I get a reasonable framerate of 27-30fps using this approach.
