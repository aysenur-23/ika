# Arduino tarafi - ika_motor_controller

## Gereksinimler
- Arduino IDE 2.x veya `arduino-cli`
- ArduinoJson kutuphanesi (`Library Manager` uzerinden)

## Yukleme
```
arduino-cli compile --fqbn arduino:avr:uno ika_motor_controller
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ika_arduino ika_motor_controller
```

## Kalibrasyon
`ika_motor_controller.ino` icindeki sabitler gercek arac uzerinde olculecek:
- `MAX_SPEED_MPS` - motorlarin maksimum cizgisel hizi
- `MIN_PWM` - olu bant (motor donmeye baslayan en dusuk PWM)
- `MAX_PWM` - motor surucusunun guvenli ust siniri

## Protokol
Pi'den gelen tek satir JSON:
```
{"l":0.12,"r":-0.05}\n
```
l, r: m/s. TIMEOUT_MS (500ms) icinde komut gelmezse Arduino motorlari durdurur.
