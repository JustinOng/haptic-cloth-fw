# UART Config

8N1 1200baud to PB1

- `Cl<byte>\n`: set `min_ldr`
- `CL<byte>\n`: set `max_ldr`
- `Cm<byte>\n`: set `min_motor`
- `CM<byte>\n`: set `max_motor`

`motor_duty_cycle = map(ldr, min_ldr, max_ldr, min_motor, max_motor)` (similar in operation to the [Arduino `map` function](https://www.arduino.cc/reference/en/language/functions/math/map/), except that if `ldr` < `min_ldr`, output = `min_motor`, if `ldr` > `max_ldr`, output = `max_motor`)
