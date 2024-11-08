from machine import I2C, Pin
from time import sleep
from micropython_rotary_encoder import RotaryEncoderRP2, RotaryEncoderEvent
import uasyncio as asyncio

# LED startup sequence
def startup():
    led = Pin(25, Pin.OUT)
    led.toggle()
    sleep(5)
    led.toggle()
    sleep(0.2)
    led.toggle()
    sleep(0.1)
    led.toggle()
    sleep(0.2)
    led.toggle()
    led.toggle() # disable led
    print("\n===============\n")
    print("Startup Complete")
    print("\n===============\n")

# Scan I2C
def scan_i2c(i2c):
    print("\n===============\n")
    print("Scanning i2c bus...")
    devices = i2c.scan()
    if len(devices) == 0:
        print("No i2c device found!")
    else:
        print("i2c devices found:", len(devices))
        for device in devices:
            print("Decimal address: ", device, " | Hexa address: ", hex(device))

    print(f"Address: {hex(device)}\n")
    print("\n===============\n")
    sleep(0.1)


# TPS55289 I2C address
TPS55289_ADDR = 0x74  # or 0x75 depending on MODE pin configuration

# Registers
VOUT_REG_LSB    = 0x00
VOUT_REG_MSB    = 0x01
IOUT_LIMIT_REG  = 0x02
VOUT_SR_REG     = 0x03
VOUT_FS_REG     = 0x04
ENABLE_REG      = 0x06
STATUS_REG      = 0x07

fb_voltage_settings = [0x00,0x01,0x02,0x03]
fb_voltages = [0.2256, 0.1128, 0.0752, 0.0564] # mV

class TPS55289:
    def __init__(self, i2c, i2c_address=0x74, enablePin=0):
        self.i2c = i2c
        self.i2c_address = i2c_address
        self.enablePin = enablePin
        self.fb = fb_voltages[3]
        self.vref = 0

        self.set_feedback_mode(True, 3)

    def write_register(self, register, value):
        self.i2c.writeto_mem(self.i2c_address, register, bytes([value]))

    def read_register(self, register):
        read_value = self.i2c.readfrom_mem(self.i2c_address, register, 1)
        return int.from_bytes(read_value, 'big')

    def set_feedback_mode(self, internal, feedback_voltage = 3):
        if internal:
            value = fb_voltage_settings[feedback_voltage]
            print(f"Feedback Mode: Internal ({fb_voltages[feedback_voltage]}mV)")
        else:
            value = 0x80  # 0b10000000
            print("Feedback Mode: External")

        self.write_register(VOUT_FS_REG, value)

    def set_vref_bytes(self, lsb, msb): # 
        self.write_register(VOUT_REG_LSB, lsb)
        self.write_register(VOUT_REG_MSB, msb)
    
    def set_voltage(self, vout): # in buck mode is scaled from 0 to 1023
        self.vref = vout * fb_voltages[3] + 45 # mV * mV + 45mV
        out = int(self.map_range(self.vref, 45, 1200, 0, 1023))
        lsb, msb = self.split_to_bytes(out)
        self.set_vref_bytes(lsb, msb)
        print("\n===============\n")
        print(f"LSB: {lsb}, MSB: {msb}, Vref: {self.vref:.0f}mV, Vout: {self.vref/self.fb/1000:.2f}V")
        print("\n===============\n")

    def set_voltage_percent(self, percent): # percent 0.0 - 1.0
        out = int(self.map_range(percent, 0, 1, 0, 1023))
        lsb, msb = self.split_to_bytes(out)
        self.set_vref_bytes(lsb, msb)
        # print("\n===============\n")
        # print(f"LSB: {lsb}, MSB: {msb}, Vout: {percent*100}%")
        # print("\n===============\n")

    def map_range(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def split_to_bytes(self, value):
        if not (0 <= value < 1024):  # Check if value is in the 10-bit range (0 to 1023)
            raise ValueError("Value out of range. Must be between 0 and 1023.")
        
        lower_byte = value & 0xFF  # Mask with 0xFF to keep only the lower 8 bits
        upper_byte = (value >> 8) & 0x03  # Shift right 8 bits and mask with 0x03
        
        return lower_byte, upper_byte

    def enable(self):
        value = 0x01
        value = value << 7
        self.write_register(ENABLE_REG, value)

    def disable(self):
        value = 0x00
        value = value << 7
        self.write_register(ENABLE_REG, value)

    def read_status(self):
        print("\n===============\n")
        print("READING STATUS\n")
        status_value = self.read_register(STATUS_REG)
        print(f"STATUS Register Value: {bin(status_value)}\n")

        # Interpret the status register
        scp = (status_value >> 7) & 0x01
        ocp = (status_value >> 6) & 0x01
        ovp = (status_value >> 5) & 0x01
        mode = status_value & 0x03  # Bits 1 and 0 indicate the operating mode

        print(f"Short Circuit Protection (SCP): {'Triggered' if scp else 'Normal'}")
        print(f"Overcurrent Protection (OCP): {'Triggered' if ocp else 'Normal'}")
        print(f"Overvoltage Protection (OVP): {'Triggered' if ovp else 'Normal'}")

        if mode == 0b00:
            print("\nOperating Mode: Boost")
        elif mode == 0b01:
            print("\nOperating Mode: Buck")
        elif mode == 0b10:
            print("\nOperating Mode: Buck-Boost")
        else:
            print("\nOperating Mode: Reserved")
        print("\n===============\n")
        
###########################






def turn_right_listener():
    global vout
    if vout <= 0:
        vout = 0
    else:
        vout -= 0.01
    print(f"Decreasing Vout: {vout:.2f}")



# def turn_right_fast_listener():
#     global vout
#     if vout <= 0:
#         vout = 0
#     else:
#         vout -= 0.05
#     print(f"Decreasing Vout Fast: {vout}")


def turn_left_listener():
    global vout
    if vout >= 1:
        vout = 1
    else:
        vout += 0.01
    print(f"Increasing Vout: {vout:.2f}")


# def turn_left_fast_listener():
#     global vout
#     if vout >= 1:
#         vout = 1
#     else:
#         vout += 0.05
#     print(f"Increasing Vout Fast: {vout}")

##Click 

def single_click_listener():
    global enabled
    if enabled:
        print("Output: Disabled")
        ic.disable()
        enabled = False
    else:
        print("Output: Enabled")
        ic.enable()
        enabled = True




# create pins for encoder and button
encoder_pin_clk = Pin(14, Pin.IN, Pin.PULL_UP)
encoder_pin_dt = Pin(15, Pin.IN, Pin.PULL_UP)
encoder_pin_sw = Pin(16, Pin.IN, Pin.PULL_UP)

# create an encoder object
# encoder = RotaryEncoderRP2(en_pin_clk, en_pin_dt) # disables fast turning
encoder = RotaryEncoderRP2(
    pin_clk=encoder_pin_clk,
    pin_dt=encoder_pin_dt,
    pin_sw=encoder_pin_sw,
    fast_ms=0,  # disable fast turn events
)

#Click events
encoder.on(RotaryEncoderEvent.CLICK, single_click_listener)

# Turn events
encoder.on(RotaryEncoderEvent.TURN_LEFT, turn_left_listener)
# encoder.on(RotaryEncoderEvent.TURN_LEFT_FAST, turn_left_fast_listener)
encoder.on(RotaryEncoderEvent.TURN_RIGHT, turn_right_listener)
# encoder.on(RotaryEncoderEvent.TURN_RIGHT_FAST, turn_right_fast_listener)

async def update_vout():
    while True:
        ic.set_voltage_percent(vout)
        await asyncio.sleep(0)


async def main():
    await asyncio.gather(
        encoder.async_tick(1), # run encoder event handling every 1ms
        update_vout()
    )

    

##############################


startup()


i2c = I2C(0, scl=Pin(5), sda=Pin(4))

scan_i2c(i2c)

ic = TPS55289(i2c)
vout = 0 # percentage
enabled = False

print(f"Starting Values: \nVout: {vout}\nEnabled: {enabled}")

ic.disable()
ic.set_voltage_percent(vout) # set initial voltage to minimum


try:
    asyncio.run(main())
except Exception as e:
    ic.disable()
    print(f"ERROR: SHUTING DOWN ({e})")



##############################



