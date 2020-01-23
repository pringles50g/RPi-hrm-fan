# ANT - Heart Rate Monitor Example
#
# Copyright (c) 2012, Gustav Tiger <gustav@tiger.name>
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

from __future__ import absolute_import, print_function

from ant.easy.node import Node
from ant.easy.channel import Channel
from ant.base.message import Message

import logging
import struct
import threading
import sys
import RPi.GPIO as GPIO
import time

import time
from neopixel import *
import argparse

# LED strip configuration:
LED_COUNT      = 120      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53




GPIO.setmode(GPIO.BCM)

# init list with pin numbers

pinList = [2, 3, 4, 17]

# loop through pins and set mode and state to 'high'

for i in pinList:
    GPIO.setup(i, GPIO.OUT)
    GPIO.output(i, GPIO.HIGH)

# time to sleep between operations in the main loop

SleepTimeL = 2

RANGE_MIN=0
RANGE_MAX=1

# one range for each gear
FAN_SPEED_RANGES = (
                    (0,100),    # 0 - off
                    (100,140),  # 1 - low
                    (140,160),  # 2 - medium
                    (160,300),  # 3 - high
                   )

MIN_GEAR=0
MAX_GEAR=len(FAN_SPEED_RANGES) - 1

HEART_RATE_SWING = 3



# Define functions which animate LEDs in various ways.
def colorWipe(strip, color, wait_ms=0):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
#        time.sleep(wait_ms/1000.0)


class Fan:
    def __init__(self):
        self.current_gear = 0

    def select_gear(self, gear):
        assert gear >= 0 and gear <= 3
        if gear == 0:
            GPIO.output(4, GPIO.HIGH)
            GPIO.output(3, GPIO.HIGH)
            GPIO.output(2, GPIO.HIGH)
            colorWipe(strip, Color(0, 0, 255))  # Blue wipe dobrze
        elif gear == 1:
            GPIO.output(4, GPIO.HIGH)
            GPIO.output(3, GPIO.HIGH)
            GPIO.output(2, GPIO.LOW)
            colorWipe(strip, Color(255, 0, 0))  # Green wipe
        elif gear == 2:
            GPIO.output(4, GPIO.HIGH)
            GPIO.output(3, GPIO.LOW)
            GPIO.output(2, GPIO.HIGH)
            colorWipe(strip, Color(255, 255, 0))  # Yellow wipe
        elif gear == 3:
            GPIO.output(4, GPIO.LOW)
            GPIO.output(3, GPIO.HIGH)
            GPIO.output(2, GPIO.HIGH)
            colorWipe(strip, Color(0, 255, 0))  # Red wipe 
        self.current_gear = gear
    def max_gear(self):
        return 3

def set_fan_speed_from_hr(fan, bpm):
    if bpm > FAN_SPEED_RANGES[-1][RANGE_MAX]:
        fan.select_gear(MAX_GEAR)
        return
    if bpm < FAN_SPEED_RANGES[0][RANGE_MIN]:
        fan.select_gear(MIN_GEAR)
        return
    gear = fan.current_gear
    while True:
        if bpm < FAN_SPEED_RANGES[gear][RANGE_MIN] - HEART_RATE_SWING:
            gear -= 1
        elif bpm > FAN_SPEED_RANGES[gear][RANGE_MAX] + HEART_RATE_SWING:
            gear += 1
        else:
            break
    assert gear >= MIN_GEAR and gear <= MAX_GEAR
    fan.select_gear(gear)


NETWORK_KEY= [0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45]

fan = Fan()
# Make sure the number of heart rate ranges matches the number of gears
assert fan.max_gear() == MAX_GEAR

def on_data(data):
    heartrate = data[7]
    string = "Heartrate: " + str(heartrate) + " [BPM]"
    sys.stdout.write(string)
    sys.stdout.flush()
    sys.stdout.write("\b" * len(string))

    set_fan_speed_from_hr(fan, heartrate)

def main():
    node = Node()
    node.set_network_key(0x00, NETWORK_KEY)

    channel = node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE)

    channel.on_broadcast_data = on_data
    channel.on_burst_data = on_data

    channel.set_period(8070)
    channel.set_search_timeout(12)
    channel.set_rf_freq(57)
    channel.set_id(0, 120, 0)

    try:
        channel.open()
        node.start()

    finally:
        node.stop()
        GPIO.cleanup()
        colorWipe(strip, Color(0,0,0), 10)

if __name__ == "__main__":
    

    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()

    main()
