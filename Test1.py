
from select import poll, POLLIN
from sys import stdin, stdout
from utime import sleep_ms
from machine import Pin

pl = poll()
pl.register(stdin, POLLIN)

# we want a function that collects characters if they exist in the stdin stream.
# if the current character is a newline, echo the accumulated data back, and clear the result.

cmdBuf = ""
led = Pin(2, Pin.OUT)

def toggle(l):
    l.value(not l.value())

def reader():
    global cmdBuf
    while pl.poll(5): # there's something to read
        c = stdin.read(1)
        if c == "\n":
            res = cmdBuf
            cmdBuf = ""
            return res
        else:
            cmdBuf = cmdBuf + c
            print(cmdBuf)
    
while True:
    cmd = reader()
    if cmd:
        print("Command is", cmd)
    sleep_ms(200)
    toggle(led)