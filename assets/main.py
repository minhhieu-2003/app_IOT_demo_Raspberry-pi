import RPi.GPIO as GPIO
import time
import spidev

# SPI setup for MCP3008
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

# Define GPIO pin mappings
LCD_RS = 5
LCD_E  = 6
LCD_D4 = 12
LCD_D5 = 13
LCD_D6 = 16
LCD_D7 = 19

GAS_SENSOR = 18     # MQ-2 Gas Sensor pin
LM35_SENSOR = 0     # LM35 Temperature Sensor (MCP3008 CH0)
RELAY1 = 4         # Relay 1 (Gas Control)
RELAY2 = 27         # Relay 2 (Temperature Control)
LED = 17            # Indicator LED

# LCD Configuration
LCD_WIDTH = 16  # Max characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80  # LCD RAM address for 1st line
LCD_LINE_2 = 0xC0  # LCD RAM address for 2nd line

E_PULSE = 0.0005
E_DELAY = 0.0005

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([LCD_E, LCD_RS, LCD_D4, LCD_D5, LCD_D6, LCD_D7], GPIO.OUT)
    GPIO.setup(GAS_SENSOR, GPIO.IN)
    GPIO.setup([RELAY1, RELAY2, LED], GPIO.OUT, initial=GPIO.OUT)
    lcd_init()

def read_adc(channel):
    """Reads the ADC value from MCP3008"""
    if channel < 0 or channel > 7:
        return -1
    response = spi.xfer2([1, (8 + channel) << 4, 0])
    adc_value = ((response[1] & 3) << 8) + response[2]
    return adc_value

def get_temperature():
    """Reads temperature from LM35 sensor"""
    adc_val = read_adc(LM35_SENSOR)
    voltage = adc_val * 3.3 / 1024  # Convert ADC value to voltage
    return voltage / 0.01  # 10mV per °C

def gas_detected():
    """Checks if gas is detected by MQ-2 sensor"""
    return GPIO.input(GAS_SENSOR) == GPIO.HIGH

def relay_control(temp):
    """Controls relays based on gas detection and temperature"""
    if gas_detected():
        GPIO.output(RELAY1, GPIO.HIGH)
    else:
        GPIO.output(RELAY1, GPIO.LOW)
  
    if temp > 60:  # If temperature exceeds 60°C
        GPIO.output(RELAY2, GPIO.HIGH)
    else:
        GPIO.output(RELAY2, GPIO.LOW)

def lcd_init():
    """Initializes the LCD display"""
    lcd_byte(0x33, LCD_CMD)  # Initialize
    lcd_byte(0x32, LCD_CMD)  # Set to 4-bit mode
    lcd_byte(0x28, LCD_CMD)  # 2 line, 5x7 matrix
    lcd_byte(0x0C, LCD_CMD)  # Display ON, Cursor OFF
    lcd_byte(0x06, LCD_CMD)  # Entry mode set
    lcd_byte(0x01, LCD_CMD)  # Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    """Sends byte to LCD in 4-bit mode"""
    GPIO.output(LCD_RS, mode)  # Command or Data mode

    # Send high 4 bits
    GPIO.output(LCD_D4, bool(bits & 0x10))
    GPIO.output(LCD_D5, bool(bits & 0x20))
    GPIO.output(LCD_D6, bool(bits & 0x40))
    GPIO.output(LCD_D7, bool(bits & 0x80))
    lcd_toggle_enable()

    # Send low 4 bits
    GPIO.output(LCD_D4, bool(bits & 0x01))
    GPIO.output(LCD_D5, bool(bits & 0x02))
    GPIO.output(LCD_D6, bool(bits & 0x04))
    GPIO.output(LCD_D7, bool(bits & 0x08))
    lcd_toggle_enable()

def lcd_toggle_enable():
    """Toggles enable pin to send data"""
    time.sleep(E_DELAY)
    GPIO.output(LCD_E, GPIO.HIGH)
    time.sleep(E_PULSE)
    GPIO.output(LCD_E, GPIO.LOW)
    time.sleep(E_DELAY)

def lcd_string(message, line):
    """Displays a string on the LCD"""
    message = message.ljust(LCD_WIDTH, " ")  # Pad string to 16 chars
    lcd_byte(line, LCD_CMD)
    for char in message:
        lcd_byte(ord(char), LCD_CHR)

def main():
    setup()
    while True:
        temp = get_temperature()
        relay_control(temp)
        gas_status = "Yes" if gas_detected() else "No"

        lcd_string(f"Gas: {gas_status} ", LCD_LINE_1)
        lcd_string(f"Temp: {temp:.1f}C", LCD_LINE_2)
        
        print(f"Gas: {gas_status}, Temp: {temp:.1f}C, Relay1: {GPIO.input(RELAY1)}, Relay2: {GPIO.input(RELAY2)}")

        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
