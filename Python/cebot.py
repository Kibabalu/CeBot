# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""Project 'CeBot': Cerasus bot	controlled by an Raspberry Pi and
                    Pololu Micro Maestro servo controller

Hardware:   Cerasus Bot with Raspberry Pi and 6-channel Pololu Micro Maestro
            servo driver

Implementation of different timed tasks by using the module asyncio. This can
be considered as a kind of 'real time light'. In fact, all the tasks can
block each other, thus it's not really real time! Beside the timed tasks is one
state machine task. This state machine can implement different operating
modes. The joystick from the senseHAT board is used to switch between the
modes (at the moment 'wait' and 'operation').
The serial communication between the raspberry pi and the Maestro board
is implemented by using the maestro module from
https://github.com/FRC4564/Maestro. The configuration of the serial interface
on the GPIO of the Raspberry Pi 3 was done according to
'Configuring The GPIO Serial Port On Raspbian Jessie Including Pi 3' on
https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian
-jessie-including-pi-3/.

2018 frank.kirschbaum@me.com, see also https://cerasusblog.wordpress.com
"""

import asyncio
from evdev import InputDevice, list_devices
from fysom import Fysom
import maestro
import math
import random
import RPi.GPIO as GPIO
from sense_hat import SenseHat
import sys

nom_pos = [992, 1500, 1500, 1500, 1500, 1500]    # nominal servo positions
min_pos = [4*432, 4*912, 4*944, 4*816, 4*352, 4*800]         # minimum servo positions
max_pos = [4*2400, 4*2400, 4*2032, 4*2192, 4*2400, 4*2224]   # maximum servo positions
max_vel = [0, 0, 0, 0, 0, 0]                     # maximum servo speeds
max_acc = [6, 7, 7, 7, 0, 0]                     # maximum servo accelerations


def set_min_max_pos(min_position, max_position):
    """Set the minimum and maximum possible servo positions for maestro board.
    """

    for ii in range(len(min_position)):
        controller.setRange(ii, min_position[ii], max_position[ii])


def get_act_pos(number_channels):
    """Get current servo positions.
    """

    act_cur_pos = [controller.getPosition(ii) for ii in range(number_channels)]
    return act_cur_pos


def set_limits(max_velocity, max_acceleration):
    """Set movement limits (max velocity and acceleration).
    """

    for ii in range(len(max_velocity)):
        controller.setSpeed(ii, max_velocity[ii])

    for ii in range(len(max_acceleration)):
        controller.setAccel(ii, max_acceleration[ii])


def set_des_pos(desired):
    """Set servo desired servo positions.
    """

    for ii in range(len(desired)):
        controller.setTarget(ii, desired[ii])


def pos_lim(value):
    """Return value for positive values and 0 for negative.
    """

    return value if value >= 0 else 0


def waste_some_time(number_cycles):
    """Waste some computing time. Just for testing purpose.
    """

    for ii in range(number_cycles):
        tst = math.sqrt(random.randrange(0, 100))
        tst *= tst


async def task_state_machine(cycle_time):
    """State machine task for the operating modes.
    """

    next_call = loop.time()

    while True:
        if (next_call - loop.time()) < -cycle_time:
            next_call = loop.time()

        # print('state-machine-task start: ', loop.time())

        for event in sense.stick.get_events():
            if event.action == 'pressed' and event.direction == 'up':
                state_machine.up()
            if event.action == 'pressed' and event.direction == 'down':
                state_machine.down()

        next_call += cycle_time
        await asyncio.sleep(next_call - loop.time())


async def task_10ms(cycle_time):
    """task @ cycle time approx. 10ms.
    """

    global des_pos, nom_pos, min_pos, max_pos, max_vel, max_acc

    next_call = loop.time()

    pin16_state = False

    while True:
        if (next_call - loop.time()) < -cycle_time:
            next_call = loop.time()
            print('    10ms-task start corrected ****************************')

        if state_machine.current == 'operation':
            print('    10ms-task start: ', loop.time(), ' ', pin16_state)

        pin16_state = False if pin16_state else True
        GPIO.output(16, pin16_state)

        # waste_some_time(200)

        next_call += cycle_time
        await asyncio.sleep(next_call - loop.time())


async def task_100ms(cycle_time):
    """task @ cycle time approx. 100ms.
    """

    global des_pos, nom_pos, min_pos, max_pos, max_vel, max_acc

    next_call = loop.time()

    pin18_state = False

    while True:
        if (next_call - loop.time()) < -cycle_time:
            next_call = loop.time()
            print('    100ms-task start corrected ***************************')

        if state_machine.current == 'operation':

            print('  100ms-task start: ', loop.time())

            pin18_state = False if pin18_state else True
            GPIO.output(18, pin18_state)

            # waste_some_time(200)

        next_call += cycle_time
        await asyncio.sleep(next_call - loop.time())


async def task_1000ms(cycle_time):
    """task @ cycle time approx. 1000ms.
    """

    global des_pos, nom_pos, min_pos, max_pos, max_vel, max_acc

    next_call = loop.time()

    while True:
        if (next_call - loop.time()) < -cycle_time:
            next_call = loop.time()
            print('    1000ms-task start corrected **************************')

        if state_machine.current == 'operation':
            print('1000ms-task start: ', loop.time())

            # waste_some_time(200)

            des_pos = [random.randint(min_pos[ii], max_pos[ii])
                       for ii in range(len(nom_pos))]
            print('des_pos:', des_pos)
            set_des_pos(des_pos)

        next_call += cycle_time
        await asyncio.sleep(next_call - loop.time())


if __name__ == '__main__':

    print('starting initialisation')
    # GPIO stuff
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([16, 18], GPIO.OUT, initial=GPIO.LOW)
    print('GPIO settings set')

    # servo controller, sense HAT, and state machine stuff
    controller = maestro.Controller('/dev/ttyACM0')     # maestro controller
    [print('actuator %s: %s' %(ii, controller.getPosition(ii))) for ii in range(6)]

    sense = SenseHat()                                  # sense HAT
    state_machine = Fysom(  # state machine
        {'initial': 'wait', 'events':
            [{'name': 'up', 'src': 'wait', 'dst': 'operation'},
             {'name': 'down', 'src': 'wait', 'dst': 'wait'},
             {'name': 'down', 'src': 'operation', 'dst': 'wait'},
             {'name': 'up', 'src': 'operation', 'dst': 'operation'}]})

    # joystick init stuff:
    joystick_found = False
    devices = [InputDevice(fn) for fn in list_devices()]
    for dev in devices:
        if dev.name == 'Raspberry Pi Sense HAT Joystick':
            joystick_found = True
            print('Raspberry Pi Sense HAT Joystick found')
            break
    if not joystick_found:
        print('Raspberry Pi Sense HAT Joystick not found. Aborting ...')
        sys.exit()

    # robot init stuff:
    set_min_max_pos(min_pos, max_pos)
    set_limits(max_vel, max_acc)
    des_pos = nom_pos

    # scheduler init stuff:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(asyncio.gather(
        task_state_machine(250e-3),
        task_10ms(10e-3),
        task_100ms(100e-3),
        task_1000ms(1000e-3)))
    loop.close()
