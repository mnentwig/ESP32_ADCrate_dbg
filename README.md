# ESP32 ADC sample rate (continuous mode) debug
Program shows that the actual data rate is off from the configured rate by a factor of 9/11 (0.818181...)

Run with ```idf.py build flash monitor``` (no HW connections required other than UART)