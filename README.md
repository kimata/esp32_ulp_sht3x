# ULP SHT3x sensor application

This software captures temerature and humidity and send those value to
Fluentd.

By using low power ULP, it can powered by AAA batteries.

# Pin assign
* GPIO26: SCL
* GPIO25: SDA
* GPIO14: TPS61291 Bypass (if battery voltage is high enough, output L)
* GPIO32: Battery voltage monitor (A/D)
