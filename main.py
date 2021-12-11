""" main.py
    Main entry point for Raspberry Pi roaster control software.
    
    1 second timer drives the system.
    SSRs are on a 1 second loop, based on a one-shot timer. First part of the loop is ON, second part is OFF.
    The thermocouple temperatures take some time to convert...need to check how long.
    
    SSRs are created in an off condition.
    Command processor attempts to read a command from serial input (stdin). If there is a command, process it.
    Update display and return.

    Hardware assumptions include:
    1. Up to four thermocouples, on I2C interface.
    2. One 4x20 display, on I2C interface.
    3. A number of SSRs.
    4. A number of DC motors.
    5. Up to 3 potentiometers (Pico only has 3 analog channels).
    6. USB serial communication (stdin, stdout).
"""
from machine import Pin, PWM, I2C, Timer
from command import Interpreter                         # command processing
from control import SSR, Motor, Potentiometer, Elapsed  # control elements
from pico_i2c_lcd import I2cLcd                         # I2C display
from utime import sleep_ms
from tc import TC
from mcp9800 import MCP9800


"""
Configuration area: set up required hardware specifications.
============================================================
"""
splash = "MicroRoast v0.1"

temp_units = "C"  # C for Celsius, F for Fahrenheit.

# Thermocouples: support for 2 or 4.
tc = {"count":2, "address": 0x068}  # Implements the first two only.

ambient = {"address": 0x4d}

# Motors are digital pwm pins. Set to None if not required.
motors = [{"PWM":None, "pin":10, "freq":5000, "duty":0},
          {"PWM":None, "pin":11, "freq":5000, "duty":0},
       ]

# Solid state relays using 1Hz switching
ssrs = [{"pin": 25, "constraints":[75, # M0 must be above 75
                                   50] # M1 must be above 50
                    }
            ]# Not using the second SSR

# I2C channel setup.
i2c = [{"SDA":14, "SCL":15}]

# I2C display setup. If no display, set values to None
display = [{"address":0xf8, # check and match display
            "rows":4, "cols":20}
        ]

# Potentiometer setup. Client is the object the pot controls
pots = [
    {"client": ssrs[0],   "pin":26, "val":0},
    {"client": motors[0], "pin":27, "val":0},
    {"client": motors[1], "pin":28, "val":0},
    ]


"""
End of user configuration area
==============================
"""
# Initialize global data structures

# Create I2C channel
i2c[0] = I2C(1, scl=Pin(i2c[0]["SCL"]), sda=Pin(i2c[0]["SDA"]))
    
# Thermocouples
tc = [TC(i2c, tc["address"], i) for i in range(tc["count"])]

ambient = MCP9800(i2c, ambient["address"])

# Motors are accessible from this dictionary  
for m in range(len(motors)):
    motors[m] = Motor(motors[m]["pin"], motors[m]["freq"], motors[m]["duty"])
print(motors)
# SSRs are accessible from this dictionary
# populate ssrs with motors / constraint tuples
for s in range(len(ssrs)):
    ssrs[s]["constraints"] = list(zip(motors, ssrs[s]["constraints"]))
    print(ssrs[s])
# now create the SSRs themselves
for s in range(len(ssrs)):
    ssrs[s] = SSR(ssrs[s]["pin"], ssrs[s]["constraints"])

# Pots are accessible from this dictionary. 
for p in range(len(pots)):
    pots[p]=Potentiometer(pots[p]["pin"], pots[p]["client"])

# Create the display
"""
display[0] = I2cLcd(I2c[0], display[0]["address"],
                    display[0]["rows"], display[0]["cols"])
"""
display = None
print("Display:", display)

# set up the interpreter

cmnds = Interpreter(1, " ;,:")

# Create elapsed time object

elapsed = Elapsed() # an object that reports seconds since reset

# define readers

# Define commands and callbacks

def cmnd_reset(args):
    """
    RESET
    Snapshot elapsed time
    Reset motors and SSRs to zero
    Print splash screen
    """
    elapsed.reset()
    for s in ssrs:
        s.value(0)
    for m in motors:
        m.value(0)
    updater()
    
cmnds.add("RESET", 0, cmnd_reset)
    
    
def cmnd_read(arg, **args):
    """
    READ
    request for operating values. No arguments.
    """
    p_values = [int(p.value() - 0.5) for p in pots]
    cmnds.send(p_values)
    s_values = [s.duty() for s in ssrs]     # up to two ssr values
    cmnds.send(s_values)
    m_values = [m.duty() for m in motors]   # up to two motor values
    cmnds.send(m_values)
    # Artisan wants 2 or 4 thermocouple values
    # and four other values, but it ignores the last value

cmnds.add("READ", 0, cmnd_read)       # to send data back to Artisan

# Digital write
def cmnd_dwrite(args):
    """
    Set a digital out pin
    """
    assert len(args) == 2, "DWRITE: pin and 0/1 value required"
    p = int(args[0])
    v = int(args[1])
    p = Pin(p, mode=Pin.OUT)
    if v == 0:
        p.off()
    if v == 1:
        p.on()

cmnds.add("DWRITE", 2, cmnd_dwrite)

# Analog write (pwm)
def cmnd_awrite(args):
    """
    Set an analog out pin
    """
    assert len(args) == 2, "AWRITE: pin and 0-100 value required"
    p = int(args[0])
    d = int(int(args[1]) * 655.35)
    d = max(0, min(65535, d))
    p = PWM(Pin(p))
    p.freq(5000)
    p.duty_u16(d)

cmnds.add("AWRITE", 2, cmnd_awrite)

# Temperature celsius or fahrenheit
def cmnd_units(args):
    """
    Set temperature units
    Look at the first letter of the argument
    Ignore any values other than "C" and "F"
    Arg is case-sensitive.
    """
    if args[0][0] == "F":
        temp_units = "F"
    if args[0][0] == "C":
        temp_units = "C"
    print(temp_units)
        
cmnds.add("UNITS", 1, cmnd_units)

# SSR 0, typically main heat
if ssrs:
    def cmnd_ssrs(n, ssr):
        """
        SSR generator
        SSR is special in that it will be reset to zero unless
        all motors are above a threshold value. This is managed by the SSR object.
        """
        def ssr_cb(args):
            d = args[0]
            if d[0] == '-':
                d = max(ssr.duty() - int(d[1:]), 0)
            elif d[0] == '+':
                d = min(ssr.duty() + int(d[1:]), 100)
            else:
                d = int(args[0])
            ssr.duty(d)
        return ssr_cb

    n = 0
    for s in ssrs:
        print(n, s)
        cmnds.add("SSR"+str(n), 1, cmnd_ssrs(n, s))
        n = n + 1

# Motor command creation
if motors:
    #define a generator that creates a closure with a specific motor
    def cmnd_motors(n, motor, ssrs):
        """
        Motor generator.
        Successive motors are numbered from zero.
        SSRS are passed in for use in constraints.
        """
        def motor_cb(args):
            """
            Generate a motor callback
            """
            d = args[0]
            if d[0] == '-':
                d = max(motor.duty() - int(d[1:]), 0)
            elif d[0] == '+':
                d = min(mmotor.duty() + int(d[1:]), 100)
            else:
                d = int(args[0])
            motor.duty(d)
            for s in ssrs:     # check constraints on SSRs
                s.constrain()
        return motor_cb

    n = 0
    for m in motors:
        print(m, n)
        cmnds.add("M" + str(n), 1, cmnd_motors(n, m, ssrs))
        n = n + 1

def updater(args):
    """
    Update information from motors, SSRs and temperatures.
    Update the display.
    Save results for use in future READ command.
    Args is the timer that calls this function
    """
    current_amb = ambient.read_temperature()
    current_temps = temps.value()
    current_ssrs = [s.value() for s in ssrs]
    current_motors = [m.value() for m in motors]
    current_elapsed = elapsed.value()
    if display:
        # we have a real one
        print("Sending to display")
        display.clear()
    else:
        # get temperatures
        p_values = [int(p.value() - 0.5) for p in pots]
        cmnds.send(p_values)
        s_values = [s.duty() for s in ssrs]     # up to two ssr values
        cmnds.send(s_values)
        m_values = [m.duty() for m in motors]   # up to two motor values
        cmnds.send(m_values)


# set a timer to update measurements and the display twice a second
loop_timer = Timer()
loop_timer.init(mode=Timer.PERIODIC, period=500, callback=updater)

# Main loop. Processes commands.
while True:
    cmnds.do()                # process the input stream
    sleep_ms(200)             # take a nap. Not sure how long this could be...


    