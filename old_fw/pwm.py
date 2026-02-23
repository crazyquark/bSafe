from machine import Pin, PWM

# Create PWM on GPIO0
pwm = PWM(Pin(0))

# Set frequency (example: 1 kHz)
pwm.freq(10000)

# Set 10% duty cycle (range: 0–1023)
pwm.duty(310)  # ≈10% of 1023
