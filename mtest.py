""" main.py
    Main entry point for Raspberry Pi roaster control software.
    
    1 second timer drives the system.
    SSRs are on a 1 second loop, based on a one-shot timer. First part of the loop is ON, second part is OFF.
    The thermocouple temperatures take some time to convert...need to check how long.
    
    SSR1 and SSR2 are created in an off condition.
    Command processor attempts to read a command from serial input. If there is a command, process it.
    Update display and return.

    Hardware assumptions include:
    1. Up to four thermocouples, on I2C interface.
    2. One 4x20 display, on I2C interface.
    3. Up to 2 SSRs.
    4. Up to 2 DC motors.
    5. Up to 3 potentiometers.
    6. USB serial communication (stdin, stdout).
"""
from machine import Pin, I2C
from command import Interpreter                # command processing
from control import SSR, Motor, Potentiometer
from i2c_lcd import I2cLcd
from utime import sleep_ms

# Global variables for temperatures and system state
# Values are stored in a dictionary, with pin and current state as a tuple
temps = {}
ssrs = {}
motors = {}
pots = {}

"""
Configuration area: set up required hardware specifications.
============================================================
"""
"""
Using stdin instead...

# Serial port setup for communications with Artisan
serial = {"tx":0,     # 
          "rx":11,     # 
        "baud":115200 ,    # communications baud rate
          }
"""

# Solid state relays using 1Hz switching
ssrs = {"SSR1":{"pin": 25}}   # Not using the second SSR


# Motors are digital pwm pins. Set to None if not required.
motors = {"M1":{"PWM":None, "pin":10, "freq":5000, "duty":0},
          "M2":{"PWM":None, "pin":11, "freq":5000, "duty":0},
          }# Pin number and initial value

# I2C channel setup.

I2c = {"SDA":14,
       "SCL":15,
       }

# I2C display setup. If no display, set values to None
display = {"address":0xf8, # check and match display
           "rows":4, "cols":20
           }


# Thermocouple setup
tc_address = 0xff      # check and match hardware
tc_count = 2           # number of thermocouples: 0 to 4

# Potentiometer setup

pots = {"P1":{"pin":26, "val":0},
        "P2":{"pin":27, "val":0},
        "P3":{"pin":28, "val":0},
        }

"""
End of user configuration area
==============================
"""
# Initialize global data structures

for p in pots:
    pots[p]=Potentiometer(pots[p]["pin"])

    
for s in ssrs:
    ssrs[s] = SSR(ssrs[s]["pin"])

    
for m in motors:
    motors[m] = Motor(motors[m]["pin"], motors[m]["freq"], motors[m]["duty"])


I2c["I2C"] = I2C(1, scl=Pin(I2c["SCL"]), sda=Pin(I2c["SDA"]))

print(pots)
print(ssrs)
print(motors)

# can't set up display until one is attached.
# display["display"]=I2cLcd(I2c["I2C"], display["address"], display["rows"], display["cols"])
# print(display)

# set up the interpreter

cmnds = Interpreter(1, " ;,:")

# define readers

# Define commands and callbacks
def cmnd_read(arg, **args):
    """
    READ
    request for operating values. No arguments.
    """
    temps = [0, 1, 2, 3]  # temperatures returned as a list of values
    pots = [3, 4, 5]    # pots returned as a list of values
    ssrs = [6]     # up to two ssr values
    motors = [7, 8] # up to two motor values
    # Artisan wants 2 or 4 thermocouple values
    # and four other values, but it ignores the last value
    cmnds.send("All the results")

cmnds.add("READ", 0, cmnd_read)       # to send data back to Artisan

# Digital write
def cmnd_dwrite(args):
    """
    Set a digital out pin
    """
    pass

cmnds.add("DWRITE", 2, cmnd_dwrite)

# Analog write (pwm)
def cmnd_awrite(args):
    """
    Set an analog out pin
    """
    pass

cmnds.add("AWRITE", 2, cmnd_awrite)

# Temperature celcius or fahrenheit
def cmnd_units(args):
    """
    Set temperature units
    """
    print("UNITS callback")
    if args[0][0] == "F":
        temp_units = "Fahrenheit"
    if args[0][0] == "C":
        temp_units = "Celcius"
        
cmnds.add("UNITS", 1, cmnd_units)

# SSR 1, typically main heat
if len(ssrs) >= 1:
    def cmnd_ssr1(args):
        """
        Set an SSR
        """
        print("Setting SSR1: ", args)
        ssr = ssrs["SSR1"]
        d = int(args[0])
        ssr.duty(d)

    cmnds.add("SSR1", 1, cmnd_ssr1)       # to control the first SSR

# SSR2
if len(ssrs) >= 2:
    def cmnd_ssr2(args):
        """
        Set an SSR
        """
        print("SSR2 callback")

    cmnds.add("SSR2", 1, cmnd_ssr2)       # to control the second SSR

# Motor 1: typically main fan
if len(motors) >= 1:
    def cmnd_m1(args):
        """
        Set a motor
        """
        print("M1 callback")
    cmnds.add("M1", 1, cmnd_m1)       #

# Motor 2: extraction, rotation etc.
if len(motors) >= 2:
    def cmnd_m2(args):
        """
        Set a motor
        """
        print("M2 callback")

    cmnds.add("M2", 1, cmnd_m2)

# Main loop. Reads values from sensors, updates the display, and processes commands.
while True:
    cmnds.do()
#    display.update()
    sleep_ms(20)

    