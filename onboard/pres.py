from machine import Pin

class Pressure:
    def __init__(self, clk_pin, data_pin):
        self._clk = Pin(clk_pin, Pin.OUT)
        self._data = Pin(data_pin, Pin.IN, Pin.PULL_DOWN)
        
        self._pulsesNo = 1
    
    def read(self):
        while(not self._readyToSend()):
            pass
        
        # Read serial data bit by bit every clk pulse
        value = [0, 0, 0]
        for x in range(3):
            for i in range(8):
                self._clk.value(True)
                value[-x-1] |= self._data.value() << (7 - i)
                self._clk.value(False)
                
        # Why do that
#         for x in range(self._pulsesNo):
#             self._clk.value(True)
#             self._clk.value(False)
        
        return ((value[2] ^ 0x80) << 16) | (value[1] << 8) | value[0]
    
    def xor(self, a, b):
        return (a and not b) or (not a and b)
    
    def _readyToSend(self):
        return not self._data.value()
    
    @property
    def pressure(self):
        return self.read()