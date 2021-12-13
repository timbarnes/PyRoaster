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
from sys import exit
from command import Interpreter                         # command processing
from control import SSR, Motor, Potentiometer, Elapsed  # control elements
from pico_i2c_lcd import I2cLcd                         # I2C display
from utime import sleep_ms
from tc import TC
from lm75a import LM75A
from time import ticks_ms, ticks_diff


"""
Configuration area: set up required hardware specifications.
============================================================
"""
splash = "   MicroRoast v0.1   "

temp_units = "C"  # C for Celsius, F for Fahrenheit.

# Thermocouples: support for 2 or 4.
tc = {"count":2, "address": 0x68}  # Implements the first two only.

ambient = {"address": 0x48}

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
display = [{"address":0x27, # PCF8574
            "rows":4, "cols":20}
        ]

# Potentiometer setup. Client is the object the pot controls
# Specify client type and index
pots = [
    {"client": ("SSR", 0),   "pin":26, "val":0},
    {"client": ("MOTOR", 0), "pin":27, "val":0},
    {"client": ("MOTOR", 1), "pin":28, "val":0},
    ]


"""
End of user configuration area
==============================
"""
# Initialize global data structures

# Create I2C channel
i2c = I2C(1, scl=Pin(i2c[0]["SCL"]), sda=Pin(i2c[0]["SDA"]))
    
# Thermocouples
tc = [TC(i2c, tc["address"], i) for i in range(tc["count"])]

ambient = LM75A(i2c, ambient["address"])

# Motors are accessible from this dictionary  
for m in range(len(motors)):
    motors[m] = Motor(motors[m]["pin"], motors[m]["freq"], motors[m]["duty"])

# SSRs are accessible from this dictionary
# populate ssrs with motors / constraint tuples
for s in range(len(ssrs)):
    ssrs[s]["constraints"] = list(zip(motors, ssrs[s]["constraints"]))

# now create the SSRs themselves
for s in range(len(ssrs)):
    ssrs[s] = SSR(ssrs[s]["pin"], ssrs[s]["constraints"])

# Pots are accessible from this dictionary. 
for p in range(len(pots)):
    client = pots[p]["client"]
    if client[0] == "SSR":
        client = ssrs[client[1]]
    elif client[0] == "MOTOR":
        client = motors[client[1]]
    pots[p]=Potentiometer(pots[p]["pin"], client)

# Create the display and print the splash screen

display = I2cLcd(i2c, display[0]["address"],
                 display[0]["rows"], display[0]["cols"])

display.clear()
display.move_to(0, 1)
display.putstr(splash)
sleep_ms(1000)


# set up the interpreter

cmnds = Interpreter(1, " ;,:")

# Create elapsed time object

elapsed = Elapsed() # an object that reports seconds since reset

# define readers

# Define commands and callbacks

def cmnd_shutdown(args):
    """
    SHUTDOWN
    Exit the application.
    Power cycling or Run from IDE is required to restart
    """
    display.clear()
    display.move_to(4, 1)
    display.putstr("Shutting down")
    for s in ssrs:
        s.duty(0)
    for m in motors:
        m.duty(0)
    sleep_ms(1000)
    display.move_to(5, 2)
    display.putstr("...done...")
    display.move_to(0, 3)
    display.putstr(splash)
    display.backlight_off()
    exit()
    
cmnds.add("SHUTDOWN", 0, cmnd_shutdown)

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
    m_values = [m.value() for m in motors]   # up to two motor values
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
        cmnds.add("M" + str(n), 1, cmnd_motors(n, m, ssrs))
        n = n + 1

def eval_pots():
    """
    Check the potentiometers
    If their values have changed, update their clients
    """
    for p in pots:
        if p.client:
            p_old, p_new = p.value()
            if p_old:                # there's been a change
                p.client.duty(p_new) # so update the client
    for s in ssrs:
        s.constrain()

def update_display():
    """
    Update information from motors, SSRs and temperatures.
    Update the display.
    Save results for use in future READ command.
    """
    amb_val = ambient.read_temperature()
#    current_temps = temps.value()
    temp_vals = [195.55, 187.5434]
    ssr0_val = ssrs[0].value()
    ssr0_duty = ssrs[0].duty()
    ssr0_constrained = ssrs[0].constrained
    m0_val = motors[0].value()
    m1_val = motors[1].value()
    elapsed_val = elapsed.value()
    if display:
        def show_at(x, y, msg):
            """
            print a message to the display at coords x and y
            """
            display.move_to(x, y)
            display.putstr(msg)
        
        # we have a real one
        # elapsed time
        min = int(elapsed_val / 60)
        show_at(0, 0, f'{int(elapsed_val / 60):2}:')
        if min < 10:
            show_at(0, 0, '0')
        show_at(2, 0, ':')
        show_at(3, 0, f'{int(elapsed_val % 60):2}')
        # ambient temperature
        show_at(6, 0, f'AT: {int(amb_val)}')
        # ET and BT
        show_at(13, 0, f'ET: {int(temp_vals[0]):3}')
        show_at(13, 1, f'BT: {int(temp_vals[1]):3}')
        # Heater
        show_at(0, 2, f'HTR:  {ssr0_val:3}%')
        if ssr0_duty == 0: # heat is set to off
            if ssr0_constrained:
                show_at(11, 2, '         ')
            else:
                show_at(11, 2, 'READY    ')
        else: # heat is non-zero
            if ssr0_constrained:
               show_at(11, 2,  'CONSTRAIN ')
            else:
                show_at(11, 2, '*HEATING*')
        show_at(0, 3, f'FAN:  {m0_val:3}%')
        show_at(11, 3, f'ROT: {m1_val:3}%')


display.clear()


# Main loop. Processes commands.
while True:
    ts = ticks_ms()
    cmnds.do()                    # process the input stream
    eval_pots()                   # check to see if any pots have moved, and update their clients
    update_display()          # update the display
    tc = ticks_diff(ticks_ms(), ts)
    sleep_ms(1000-tc)             # take a nap for the rest of a second.



    