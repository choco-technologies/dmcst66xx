# dmcst66xx API Reference

The node (`/dev/dmcst66xx<instance>`) follows the generic **input** interface of [dmdrvi](https://github.com/choco-technologies/dmdrvi)
(`DMDRVI_IOCTL_INPUT_*`): generic code needs only the path.

| | |
|---|---|
| `DMDRVI_IOCTL_INPUT_GET_INFO` | `dmdrvi_input_info_t`: "CST66xx", a touch screen, `width` x `height`, up to `max_contacts` contacts |
| `DMDRVI_IOCTL_INPUT_GET_STATE`, `read()` | `dmdrvi_input_state_t`: the contacts in screen coordinates; each has `event` DOWN, MOVE or UP (the last report of a lifted contact, once) |
| `DMDRVI_IOCTL_INPUT_WAIT_EVENT` | sleeps until the state differs from the one handed out last (`uint32_t*` timeout in ms, `-ETIMEDOUT`) |
| `dmcst66xx_ioctl_cmd_get_config` | `dmcst66xx_config_t`: the configuration, with the range of the chip as the driver uses it |

The chip is polled by `read()` / `GET_STATE` and, every `poll_ms`, by `WAIT_EVENT`; nothing runs in the background. A state that was
not handed out yet is kept as it is (no UP is lost between two calls).

```c
void *fp = Dmod_FileOpen("/dev/dmcst66xx0", "r");
uint32_t timeout = 1000;
if (Dmod_Ioctl(fp, DMDRVI_IOCTL_INPUT_WAIT_EVENT, &timeout) == 0) {
    dmdrvi_input_state_t state;
    Dmod_Ioctl(fp, DMDRVI_IOCTL_INPUT_GET_STATE, &state);
    for (int i = 0; i < state.contact_count; i++)
        Dmod_Printf("%u,%u\n", state.contacts[i].x, state.contacts[i].y);
}
```

## Functions

Pure functions, exported for the tests and for other code that reads the chip itself:

```c
bool    dmcst66xx_validate_config(const dmcst66xx_config_t *config);
uint8_t dmcst66xx_parse_report(const uint8_t *report, size_t size, dmcst66xx_raw_contact_t *contacts, uint8_t max_contacts);
void    dmcst66xx_map_point(const dmcst66xx_config_t *config, uint16_t raw_x, uint16_t raw_y, uint16_t *x, uint16_t *y);
```

## The chip

32-bit register addresses, high byte first (a write of the address, then a read with a repeated START; a command is the address alone).

- `0xD0070000`: the report. The first read is 9 bytes (a 16-bit checksum, the type, the numbers of fingers (low nibble) and keys (high
  nibble), the first entry), the other entries follow as a plain read, 5 bytes each: x low, y low, pressure, x high (low nibble) and y
  high (high nibble), the finger (low nibble) and the event (high nibble, 0 = lifted). The keys come first. The type is `0xFF` for
  positions; the chip reads zeros while it has nothing new. The checksum is the sum of the entries and `0x55`.
- `0xD00002AB`: written after every report, the chip then makes the next one.
- `0xD0030000`: what the chip says about itself (50 bytes; bytes 2 and 3 are `0xCA`, the chip type is in the bytes 0..3, the resolution
  in 28..31).
- `0xD0000400` (twice, 1 ms apart), `0xD0000000`, `0xD0000C00`, `0xD0000100`: the start - wake up and the normal mode. The chip goes
  to sleep when nothing touches it and does not answer then: the first access wakes it, the driver wakes it and reads again.
