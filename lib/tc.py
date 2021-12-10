"""
Thermocouple library.

Configure ADC, read ambient, read thermocouples, and calculate temperatures.
"""

from mcp9800 import MCP9800
from bisect import bisect


class OutOfRangeError(Exception):
    pass


class TC(object):
    """
    Thermocouple interface using MCP3428
    """
    
    _temps = [0, 0, 0, 0]
    _count = 0  # initially no thermocouples
    _i2c = None
    _address = None
    _types = None
    # type K decades from https://srdata.nist.gov/its90/download/type_k.tab
    DEGREES_C = list(range(-270, 1380, 10))
    MILLIVOLTS = [-6.458, -6.441, -6.404, -6.344, -6.262, -6.158, -6.035, -5.891,
    -5.730, -5.550, -5.354, -5.141, -4.913, -4.669, -4.411, -4.138, -3.852, -3.554,
    -3.243, -2.920, -2.587, -2.243, -1.889, -1.527, -1.156, -0.778, -0.392, 0.000,
    0.397, 0.798, 1.203, 1.612, 2.023, 2.436, 2.851, 3.267, 3.682, 4.096, 4.509,
    4.920, 5.328, 5.735, 6.138, 6.540, 6.941, 7.340, 7.739, 8.138, 8.539, 8.940,
    9.343, 9.747, 10.153, 10.561, 10.971, 11.382, 11.795, 12.209, 12.624, 13.040,
    13.457, 13.874, 14.293, 14.713, 15.133, 15.554, 15.975, 16.397, 16.820, 17.243,
    17.667, 18.091, 18.516, 18.941, 19.366, 19.792, 20.218, 20.644, 21.071, 21.497,
    21.924, 22.350, 22.776, 23.203, 23.629, 24.055, 24.480, 24.905, 25.330, 25.755,
    26.179, 26.602, 27.025, 27.447, 27.869, 28.289, 28.710, 29.129, 29.548, 29.965,
    30.382, 30.798, 31.213, 31.628, 32.041, 32.453, 32.865, 33.275, 33.685, 34.093,
    34.501, 34.908, 35.313, 35.718, 36.121, 36.524, 36.925, 37.326, 37.725, 38.124,
    38.522, 38.918, 39.314, 39.708, 40.101, 40.494, 40.885, 41.276, 41.665, 42.053,
    42.440, 42.826, 43.211, 43.595, 43.978, 44.359, 44.740, 45.119, 45.497, 45.873,
    46.249, 46.623, 46.995, 47.367, 47.737, 48.105, 48.473, 48.838, 49.202, 49.565,
    49.926, 50.286, 50.644, 51.000, 51.355, 51.708, 52.060, 52.410, 52.759, 53.106,
    53.451, 53.795, 54.138, 54.479, 54.819]
    #~ MILLIVOLTS_SET = set(MILLIVOLTS)

    
    def __init__(self, count, i2c, address, types):
        """
        Create count thermocouples and set up I2C communication.
        count:   number to be set up (1 to 4)
        i2c:     I2C object for communications
        address: address of the MCP3428 chip
        types:   thermocouple types (list)
        """
        assert count >= 1 and count <= 4, "TC: count must be between 1 and 4"
        self._count = count
        self._address = address
        self._itc = i2c
        self._types = types   # first implementation assumes type "K"
        # Connect to the ADC and enable it with 9 bit accuracy
    
    def read(self):
        """
        Read temperatures and return as a list.
        - Read temp from ambient
        - Convert to mV
        - Read mV values from ADC
        - Convert to temperatures
        - Return
        """
        pass
    
        def bisect(a, x, lo=0, hi=None):
        """
        Return the index where to insert item x in list a, assuming a is sorted.
        The return value i is such that all e in a[:i] have e <= x, and all e in
        a[i:] have e > x.  So if x already appears in the list, a.insert(x) will
        insert just after the rightmost x already there.
        Optional args lo (default 0) and hi (default len(a)) bound the
        slice of a to be searched.
        """
        if lo < 0:
            raise ValueError('lo must be non-negative')
        if hi is None:
            hi = len(a)
        while lo < hi:
            mid = (lo+hi)//2
            if x < a[mid]: hi = mid
            else: lo = mid+1
        return lo

    
       def C_to_mV(C):
        """
        convert degrees C to millivolts
        """
        if not -270 <= C <= 1370:
            raise OutOfRangeError('C out of range. Expected value between -270 and 1370, got {}'.format(C))
        idx = C//10+28 # faster than bisect
        x = C % 10
        y1 = MILLIVOLTS[idx-1]
        #~ if x == 0: # this check adds about 10 ns on average to each calculation. For non-random data this check would save some time.
            #~ return y1
        y2 = MILLIVOLTS[idx]
        # (rise over run) * x + b; run is always 10
        mV = ((y2 - y1) / 10.) * x + y1
        return mV
    
    def mV_to_C(mV):
        """
        convert millivolts to degrees C
        """
        if not -6.458 <= mV < 54.819:
            raise OutOfRangeError('mV out of range. Expected value between -6.458 and 54.819, got {}'.format(mV))
        idx = bisect(MILLIVOLTS, mV)
        y1 = DEGREES_C[idx-1]
        #~ if mV in MILLIVOLTS_SET: # this check adds about 100 ns on average to each calculation. For non-random data this check would save some time.
            #~ return y1
        x1 = MILLIVOLTS[idx-1]
        x2 = MILLIVOLTS[idx]
        x = mV - x1
        # (rise over run) * x + b; rise is always 10
        C = (10. / (x2 - x1)) * x + y1
        return C
    
    def get_temp(cold_junction, tc_voltage):
        """
        uses the cold junction temperature and the voltage reading from a type K
        thermocouple to calculate the true temperature of the thermocouple
    
        :cold_junction:
          temperature of the cold junction in degrees C
        :tc_voltage:
          voltage from the thermocouple in millivolts
        """
        return mV_to_C(C_to_mV(cold_junction) + tc_voltage)
 
    