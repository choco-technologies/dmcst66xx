# dmcst66xx

DMOD driver for touch panels with a **CST66xx** controller (Hynitron: CST3530, CST36xx, CST66xx, CST68xx) on I2C - e.g. the LILYGO T-Deck Pro (V1.1).
The CST328 is another protocol: [dmcst328](https://github.com/choco-technologies/dmcst328).

[![CI](https://github.com/choco-technologies/dmcst66xx/actions/workflows/ci.yml/badge.svg)](https://github.com/choco-technologies/dmcst66xx/actions/workflows/ci.yml)

The node follows the generic **input** interface of [dmdrvi](https://github.com/choco-technologies/dmdrvi) (`DMDRVI_IOCTL_INPUT_*`), so
generic code (a GUI library, a test tool) needs only the path: `/dev/dmcst66xx0`. It reports up to five contacts in screen coordinates
(the driver swaps, mirrors and scales the chip's axes as configured), `WAIT_EVENT` sleeps until they change.

It talks to the chip through the nodes of other drivers - [dmi2c](https://github.com/choco-technologies/dmi2c) for the bus and
[dmgpio](https://github.com/choco-technologies/dmgpio) for the reset line - so it has no hardware port and runs wherever those do.

## Use

One `friends_group` holds the touch panel and everything it is wired to; `friend_role` says who is who (`i2c`, `reset`).
[configs/board/t-deck-pro/touch.ini](configs/board/t-deck-pro/touch.ini) is the whole touch panel of the LILYGO T-Deck Pro V1.1 (the I2C
bus of the board, the reset line). In the flash modules of the board:

```text
dmgpio
dmi2c
dmcst66xx driver=board/t-deck-pro/touch.ini
```

```c
void *fp = Dmod_FileOpen("/dev/dmcst66xx0", "r");
uint32_t timeout = 1000;
if (Dmod_Ioctl(fp, DMDRVI_IOCTL_INPUT_WAIT_EVENT, &timeout) == 0) {
    dmdrvi_input_state_t state;
    Dmod_Ioctl(fp, DMDRVI_IOCTL_INPUT_GET_STATE, &state);    /* contacts[i].x, .y, .event */
}
```

See [docs/api-reference.md](docs/api-reference.md) and [docs/configuration.md](docs/configuration.md). `touchtest` of dmcst328 (any input node) prints the
contacts.

## Building

The code does not depend on the CPU, so there is no port: the module is built per architecture, as dmtty or dmspiflash are.

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build                       # the host: the tests
cmake -S . -B build-esp -DDMOD_TOOLS_NAME=arch/xtensa/esp32s3 && cmake --build build-esp   # an architecture
```

`DMOD_CPU_FAMILY` (stm32f7 by default) only picks the package of dmgpio and dmi2c whose headers are used (their packages are named
by family; the headers are the same for every architecture).
