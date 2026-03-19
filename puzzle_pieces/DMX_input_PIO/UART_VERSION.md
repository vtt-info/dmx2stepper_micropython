# UART Version Alternative

Located in `/home/pi/Documents/DMX_input/`

If you need to use hardware UART instead of PIO, an identical implementation is available:

```bash
cd /home/pi/Documents/DMX_input
./deploy.sh
```

## Differences

| Aspect | PIO Version | UART Version |
|--------|-------------|--------------|
| Location | `DMX_input_PIO/` | `DMX_input/` |
| Resources | 1 PIO SM + GPIO1 | UART0 + GPIO0 + GPIO1 |
| UARTs freed | Both (UART0, UART1) | Neither |
| Performance | ~30 fps | ~30 fps |
| Use case | Default choice | When PIO unavailable |

Both versions have identical APIs and performance. Use PIO version unless you need UART0 for other RP2040 PIO operations.
