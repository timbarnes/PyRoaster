""" control.py

    Manage inputs and outputs:
    - potentiometers on analog lines
    - SSRs on digital lines (1Hz manipulated pwm)
    - motors on digital / pwm lines
"""

from machine import Pin, Timer, PWM, ADC
from time import ticks_ms, ticks_diff
from MCP342x import MCP342x


class SSR(object):
    """
    Manage a slow-pulsing solid state relay - one per instance.
    Duty ranges from 0 (off) to 100 (fully on).
    
    At initialization, the SSR is off.
    It is the caller's responsibility to know if fans etc. are on before activating the SSR.
    If duty is zero, the SSR is turned off.
    If duty is 100, the SSR is turned on.
    For all other values,
      the SSR is turned off
      a timer is set to repeatedly turn the SSR off at one second intervals.
      a one-shot timer is run to set another repeated timer to turn the SSR on at one second intervals.
      the one-shot timer establishes a delay that skews on and off times to set the duty cycle.
    """
    pin = None
    _pinNum = 0
    _onTimer = None
    _offTimer = None
    _deltaTimer = None
    _duty = 0  # current duty value is available to callers
       
    def __init__(self, pin, constraints=None):
        """
        Create an instance of an SSR controller using the specified pin.
        Constraints is a list of motors and values below which the SSR will be disabled.
        A value of zero is ignored and means the motor does not implement a constraint.
        This is to ensure that a heater doesn't run without air, for example.
        """
        self.pin = Pin(pin, Pin.OUT)
        self._pinNum = pin
        self.pin.off()  # start with everything turned off
        self._onTimer = Timer()
        self._offTimer = Timer()
        self._deltaTimer = Timer()
        self._constraints = constraints
        self.constrained = True  # check constraints first time through
        
    def __str__(self):
        return "SSR pin:"+str(self.pin) + "duty:" + str(self._duty)
            
    def off(self):
        self.pin.off(0)
        
    def on(self):
        self.pin.on(1)
        
    def duty(self, duty=None):
        """
        The SSR is off at the beginning of the cycle.
        duty values are clamped to a range of 0-100.
        """
        def ssrOff(timer):
            self.pin.off()
        def ssrOn(timer):
            self.pin.on()
        def setOffTimer(timer):
            """
            Callback that sets a one second repeating timer to turn the SSR off
            """
            self._offTimer.init(freq=1, callback=ssrOff)
            self.pin.off() # for the first cycle

        if duty == None:
            return self._duty
        # Action required, so deinit existing timers
        self._offTimer.deinit()
        self._onTimer.deinit()
        if duty <= 0:
            self._duty = 0
            self.pin.off()
        elif duty >= 100:
            self._duty = 100
            self.pin.on()
        else:
            self._duty = duty
            # check constraints and douse the SSR if motors are too low
            if self.constrained:
                print("duty: constrained")
                return self._duty
            # set timers for the requested duty cycle.
            self.constrained = False
            ms = duty * 10  # the number of milliseconds of on time in a one second cycle
            self._deltaTimer.init(mode=Timer.ONE_SHOT, period=ms, callback=setOffTimer)
            self.pin.on()  # on time comes first
            self._onTimer.init(mode=Timer.PERIODIC, freq=1, callback=ssrOn)  
        return self._duty

    def constrain(self):
        """
        Check constraints, and snub SSR if required
        0 means no constraint; otherwise integer value between 1 and 100.
        """
        for c in self._constraints:
            m, d = c
            if d > 0 and m.duty() < d:
                # Turn off the SSR, but don't change the stored duty value
                self._offTimer.deinit()
                self._onTimer.deinit()
                self.pin.off()
                self.constrained = True
                return self.constrained
        if self.constrained == True:
            # restart the SSR using the stored duty value
            self.constrained = False
            self.duty(self._duty)
            return self.constrained

class Motor(object):
    """
    A PWM pin used to drive a fan or other DC motor.
    Duty ranges from 0 to provided value <= 65535. Default is 100
    """
    pin = None
    _pwm = None
    _freq = 1000
    _duty = 0
    maxDuty = 100
    
    def __init__(self, pin, freq=1000, duty=0, maxDuty=100):
        """
        Create a Motor object, setting up the output pin with initial value.
        maxDuty is the highest allowable value, so motor drive can be scaled.
        Minimum is always zero.
        """
        # set up class variables
        self.pin = pin
        self._duty = duty
        self._freq = freq
        self.maxDuty = maxDuty
        # create the PWM and set it's frequency and duty
        self._pwm = PWM(Pin(pin))
        self._pwm.freq(int(freq*65535.0/maxDuty))
        self._pwm.duty_u16(duty)
        
    def deinit(self):
        """
        Turn off PWM on the pin
        """
        if self._pwm:
            self._pwm.deinit()
            return True

    def duty(self, duty=None):
        """
        Set the duty value for the PWM if provided.
        Otherwise return the current value
        """
        if duty == None:
            return self._duty
        if duty != self._duty:
            self._duty = duty
            self._pwm.duty_u16(duty)
            return duty

    def freq(self, freq=None):
        """
        Set the PWM frequency if arg. provided
        Return PWM frequency
        """
        if freq == None:
            return self._freq
        if freq != self._freq:
            self._freq = freq
            self._pwm.freq(freq)
            return freq
        
class Potentiometer(object):
    """
    Manage analog inputs.
    Voltage range is from 0 to 3.3V
    Raw range of the ADC is from 0 to 65535
    Potentiometers are rescaled to a range of 0-100
    """
    _adc = None
    _lastVal = 0
    _value = 0
    pin = None
    id = None
    
    def __init__(self, pin, client = None):
        """
        Initialize a potentiometer interface.
        Pin is the analog input pin for the pot.
        client is the object that the pot controls / serves
        """
        self.pin = pin
        self._adc = ADC(pin)
        self._value = self._adc.read_u16() * 100. / 65535
        self.client = client

    def value(self):
        """
        Read the potentiometer and return the current value
        after saving the previous value
        """
        new = self._adc.read_u16() * 100. / 65535
        if new != self._value:
            self._lastVal = self._value
            self._value = new
        return self._value
    

class Elapsed(object):
    """
    Keeps track of time since the last RESET or from the start of execution.
    """
    start = ticks_ms()
    
    def reset(self):
        """
        Reset time stamp to begin a new count.
        """
        self.start = ticks_ms()
    
    def now(self):
        """
        Time in seconds since timer start or reset.
        """
        return (ticks_diff(ticks_ms(), self.start)) / 1000

    