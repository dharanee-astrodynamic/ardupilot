#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Commands for the no-GPS single fan thrust vectoring test (Pixhawk 2.4.8, AltHold with the lidar as the height source).

  edf_tvc_commands.py [--connect CONN] arm
  edf_tvc_commands.py [--connect CONN] takeoff [HEIGHT_M]     take off to HEIGHT_M above the start (default 0.5) and hold
                                                              (arms first, if it is not armed: the vehicle disarms itself
                                                              10 s after arming if it has not taken off, DISARM_DELAY)
  edf_tvc_commands.py [--connect CONN] land                   land (Land mode), and wait for the disarm
  edf_tvc_commands.py [--connect CONN] status

CONN is any pymavlink connection string, for example /dev/ttyACM0, udp:127.0.0.1:14551 or tcp:127.0.0.1:5760.
Only one program can open the USB port: run MAVProxy as the router (mavproxy.py --master=/dev/ttyACM0
--out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551) and point the ground station and this script at its outputs.

The ground station's own Takeoff button switches to Guided mode, which needs a GPS. This sends the take-off command in
AltHold instead, with the flag that says no horizontal position is required.
"""
import argparse
import sys
import time

from pymavlink import mavutil

ALT_HOLD = 'ALT_HOLD'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--connect', default='tcp:127.0.0.1:5760')
    ap.add_argument('command', choices=['arm', 'takeoff', 'land', 'status'])
    ap.add_argument('height', nargs='?', type=float, default=0.5)
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.connect)
    if m.wait_heartbeat(timeout=15) is None:
        sys.exit("no heartbeat from the vehicle on %s" % args.connect)
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    state = {'armed': False, 'mode': None, 'range': None}

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            x = m.recv_match(blocking=True, timeout=0.2)
            if x is None:
                continue
            if x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == m.target_system:
                state['armed'] = bool(x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                state['mode'] = mavutil.mode_string_v10(x)
            elif x.get_type() == 'DISTANCE_SENSOR':
                state['range'] = x.current_distance * 0.01
            elif x.get_type() == 'STATUSTEXT':
                print("  vehicle: %s" % x.text)

    def command(command, *params):
        params = list(params) + [0] * (7 - len(params))
        m.mav.command_long_send(m.target_system, m.target_component, command, 0, *params)
        end = time.time() + 5
        while time.time() < end:
            x = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if x is not None and x.command == command:
                return x.result
        return None

    def arm():
        m.set_mode_apm(ALT_HOLD)
        pump(1)
        # a pre-arm check that is still settling after boot clears by itself: retry for up to a minute, showing the reason
        result = None
        for _ in range(30):
            result = command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            pump(2)
            if state['armed']:
                break
        show("arm (last result %s, 0 = accepted)" % result)
        if not state['armed']:
            sys.exit("not armed: read the 'Arm:' / 'PreArm:' messages above")

    def show(label):
        print("%s: mode %s, %s, lidar %s" % (label, state['mode'], 'ARMED' if state['armed'] else 'disarmed',
                                             'none' if state['range'] is None else '%.2f m' % state['range']))

    pump(2)
    if args.command == 'status':
        show("status")
    elif args.command == 'arm':
        arm()
    elif args.command == 'takeoff':
        m.set_mode_apm(ALT_HOLD)
        pump(1)
        if not state['armed']:
            arm()
        # param3 = 1: horizontal position not required (there is no GPS). param7 = height above the start
        result = command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 1, 0, 0, 0, args.height)
        print("take-off to %.2f m: result %s (0 = accepted)" % (args.height, result))
        for _ in range(12):
            pump(1)
            show("  climbing")
    elif args.command == 'land':
        m.set_mode_apm('LAND')
        for _ in range(40):
            pump(1)
            show("  landing")
            if not state['armed']:
                break
        print("landed and disarmed" if not state['armed'] else "still armed after 40 s: check the vehicle")


if __name__ == '__main__':
    main()
