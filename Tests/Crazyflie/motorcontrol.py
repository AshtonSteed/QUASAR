# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""
import logging
import time

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)


class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Arm the Crazyflie
        self._cf.platform.send_arming_request(True)
        print('Connection to %s Completed' % (link_uri))
        time.sleep(1.0)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def ramp_motors(self):
        # Demonstrate individual motor control using motorPowerSet
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Set motorPowerSet.enable to 1 and control individual motors
        # Motor 1
        self._cf.param.set_value('motorPowerSet.enable', 1)
        self._cf.param.set_value('motorPowerSet.m1', 10000)
        time.sleep(2.0)
        self._cf.param.set_value('motorPowerSet.m1', 0)
        # Motor 2
        self._cf.param.set_value('motorPowerSet.m2', 10000)
        time.sleep(2.0)
        self._cf.param.set_value('motorPowerSet.m2', 0)
        # Motor 3
        self._cf.param.set_value('motorPowerSet.m3', 10000)
        time.sleep(2.0)
        self._cf.param.set_value('motorPowerSet.m3', 0)
        # Motor 4
        self._cf.param.set_value('motorPowerSet.m4', 10000)
        time.sleep(2.0)
        self._cf.param.set_value('motorPowerSet.m4', 0)
        
        #self._cf.platform.send_motor_power_set(0, 0, 0, 0, 0)

        for _ in range(30):
            # Continuously send the zero setpoint until the drone is recognized as landed
            # to prevent the supervisor from intervening due to missing regular setpoints
            self._cf.commander.send_setpoint(0, 0, 0, 0)
            # Sleeping before closing the link makes sure the last
            # packet leaves before the link is closed, since the
            # message queue is not flushed before closing
            time.sleep(0.1)

    def disconnect(self):
        self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    me = MotorRampExample(uri)

    me.ramp_motors()

    me.disconnect()