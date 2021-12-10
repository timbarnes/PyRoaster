""" control.py

    Manage inputs and outputs:
    - potentiometers on analog lines
    - SSRs on digital lines (1Hz manipulated pwm)
    - motors on digital / pwm lines
"""

from machine import Pin, Timer, PWM

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
    _pin = None
    _pinNum = 0
    _onTimer = None
    _offTimer = None
    duty = 0  # current duty value is available to callers
    
    def __init__(self, pn):
        """
        Create an instance of an SSR controller using the specified pin.
        """
        self._pin = Pin(pn, Pin.OUT)
        self._pinNum = pn
        self._pin.value(0)  # start with everything turned off
        
    def off(self):
        self._pin.value(0)
        
    def on(self):
        self._pin.value(1)
        
    def set(self, duty):
        """
        The SSR is off at the beginning of the cycle.
        duty values are clamped to a range of 0-100.
        """
        def setOffTimer(timer):
            """
            Callback that sets a one second repeating timer to turn the SSR off
            """
            def off():
                self._pin.value(0)
                
            self._offTimer = Timer(mode=Timer.PERIODIC, period=1000, callback=self.off)
            
        if duty == self.duty:
            # there's no change, so return
            return self.duty
        # There's a change, so deinit existing timers if there are any
        if self._offTimer:
            self._offTimer.deinit()
            self._offTimer = None
        if self._onTimer:
            self._onTimer.deinit()
            self._onTimer = None
        if duty <= 0:
            self.duty = 0
            self._pin.value(0)
        elif duty >= 100:
            self.duty = 100
            self._pin.value(1)
        else:
            self.duty = duty
            # set timers for the requested duty cycle.
            ms = duty * 10  # the number of milliseconds of on time in a one second cycle
            deltaTimer = Timer(mode=Timer.ONE_SHOT, period = ms, callback = self.setOffTimer)
            self._offTimer = Timer.init(mode=Timer.PERIODIC, period=1000, callback=off)  
        return self.duty
    
class Motor(object):
    """
    A PWM pin used to drive a fan or other DC motor.
    Duty ranges from 0 to provided value <= 65535. Default is 100
    """
    pin = None
    _mPWM = None
    _freq = 1000
    _duty = 0
    max = 100
    
    def __init__(self, pin, duty=0, freq=1000, max=100):
        """
        Create a Motor object, setting up the output pin with initial value.
        """
        # set up class variables
        self.pin = pin
        self._duty = duty
        self._freq = freq
        self.max = max
        # create the PWM and set it's frequency and duty
        self._mPWM = PWM(Pin(pin))
        self.PWM.freq(int(freq*65535.0/max))
        self.PWM.duty_u16(duty)
        
    def deinit(self):
        """
        Turn off PWM on the pin
        """
        if _mPWM:
            _mPWM.deinit()
            return True

    def duty(self, duty=-1):
        """
        Set the duty value for the PWM if provided.
        Otherwise return the current value
        """
        if duty < 0:
            return self._duty
        if duty != self._duty:
            self._duty = duty
            self._PWM.duty(duty)
            return duty

    def freq(self, f=-1):
        """
        Set the PWM frequency if arg. provided
        Return PWM frequency
        """
        if freq <0:
            return self._freq
        if freq != self._freq:
            self._freq = freq
            self.PWM.freq(freq)
            return freq
        