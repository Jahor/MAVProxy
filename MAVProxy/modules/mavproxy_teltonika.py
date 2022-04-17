"""
Teltonika Router/Gateway integration
"""
import base64
import datetime
import json
import sys
import time
import typing
from math import sqrt, radians, cos, sin

import requests
from flask import request
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class JsonRPCError(BaseException):
    def __init__(self, code, message):
        super(JsonRPCError, self).__init__()
        self.code = code
        self.message = message


class TeltonikaModule(mp_module.MPModule):
    """
    Monitors Teltonika router/gateway mobile connection and reports it over mavlink as RADIO_STATUS or CELLULAR_STATUS packet

    Sets up incoming SMS handling via HTTP request and allows control over SMS messages.
    Re-configures sms commands of the gateway to include location/status to be available even in case of companion computer disconnection
    """
    def __init__(self, mpstate):
        super(TeltonikaModule, self).__init__(mpstate, "teltonika", "Teltonika status module", public=False)
        self.teltonika_settings = mp_settings.MPSettings(
            [('router', str, None),
             ('port', int, 80),
             ('username', str, None),
             ('password', str, None),
             ('path', str, 'ubus'),
             ('freq', float, 1.0),
             ('logfile', str, None),
             ('sendalllinks', bool, False),
             ('sendalloutputs', bool, False),
             ('sms_enabled', bool, False),
             ])
        self.add_command('teltonika', self.cmd_teltonika, 'Teltonika Status control',
                         ["<status>",
                          "<start>",
                          "<stop>",
                          "set (TELSETTING)"])
        self.add_completion_function('(TELSETTING)',
                                     self.teltonika_settings.completion)
        self.started = False
        self.session = None
        self.logfile = None
        self.last_status = None
        self.last_update = 0
        self.last_login = 0
        self.request_num = 1
        self.last_status_message_update = 0
        self.modem = None

        restserver = self.mpstate.module('restserver')
        if restserver is not None:
            restserver.add_endpoint('/sms', 'sms', self.incoming_sms, methods=('GET',))

        self.cmdlong = self.mpstate.module('cmdlong')

    def incoming_sms(self):
        if not self.teltonika_settings.sms_enabled:
            return "DISABLED"
        sender = request.args['sender']
        message = base64.decodebytes(request.args['message'].encode('ascii')).decode('utf-8').strip()
        print(f"SMS from {sender}: ", message)
        if message.startswith("!!"):
            args = [a.strip() for a in message[2:].split(' ') if a.strip()]
            cmd = args[0].lower()
            if cmd == 'arm':
                self.master.mav.command_long_send(
                    self.target_system,  # target_system
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                    0,  # confirmation
                    1,  # param1 (1 to indicate arm)
                    0,  # param2  (all other params meaningless)
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    0)  # param7
            elif cmd == 'status':
                status = self.master.messages.get('SYS_STATUS')
                position = self.master.messages.get('GLOBAL_POSITION_INT')
                wind = self.master.messages.get('WIND')

                self.send_sms(sender,
                              f"{'ARMED' if self.master.motors_armed() else 'DISARMED'} {self.master.flightmode} Batt {status.battery_remaining}% ({status.voltage_battery / 1000:.2f}V)\n"
                              f"@ {position.lat / 1e7:.7f}, {position.lon / 1e7:.7f} {sqrt(position.vx ** 2 + position.vy ** 2) / 100:.1f}m/s @ {position.hdg / 100:.0f}deg\n"
                              f"Wind: {wind.speed:.1f}m/s @ {wind.direction:.0f}deg"
                              )

            elif cmd in ('rtl', 'auto', 'guided', 'hold'):
                self.master.set_mode(cmd.upper())
            elif cmd in ('loit', 'loiter'):
                self.master.set_mode('LOITER')
            elif cmd == 'move':
                if len(args) not in (3, 5):
                    self.send_sms(sender, "Usage: move X fwd|aft|port|stb|HDG [Y fwd|aft|port|stb]")
                    return "OK"
                if self.master.flightmode != 'GUIDED':
                    self.send_sms(sender, f"Not in Guided mode {self.master.flightmode}")
                    return "OK"
                distance1 = float(args[1])
                direction1 = args[2].lower()
                distance2 = float(args[3]) if len(args) == 5 else 0.0
                direction2 = args[4].lower() if len(args) == 5 else ''

                if direction2 == '':
                    try:
                        bearing = int(direction1)
                    except ValueError:
                        bearing = None
                else:
                    bearing = None
                if bearing is not None:
                    frame = mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED
                    distance_x = distance1 * cos(radians(bearing))
                    distance_y = distance1 * sin(radians(bearing))
                else:
                    if direction1 in ('s', 'e', 'w', 'n', 'north', 'south', 'east', 'west') and direction2 in (
                    '', 's', 'e', 'w', 'n', 'north', 'south', 'east', 'west'):
                        frame = mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED
                        pos_x = ('n', 'north')
                        neg_x = ('s', 'south')
                        pos_y = ('e', 'east')
                        neg_y = ('w', 'west')
                    else:
                        frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_FRD
                        pos_x = ('fwd',)
                        neg_x = ('aft',)
                        pos_y = ('stb',)
                        neg_y = ('port',)
                    distance_x = (distance1 if direction1 in pos_x else -distance1 if direction1 in neg_x else 0) + (
                        distance2 if direction2 in pos_x else -distance2 if direction2 in neg_x else 0)
                    distance_y = (distance1 if direction1 in pos_y else -distance1 if direction1 in neg_y else 0) + (
                        distance2 if direction2 in pos_y else -distance2 if direction2 in neg_y else 0)
                print(f"Moving {distance_x:.1f}m {distance_y:.1f}m")
                if abs(distance_x) > 0.5 or abs(distance_y) > 0.5:
                    self.master.mav.set_position_target_local_ned_send(
                        0,  # system time in milliseconds
                        self.settings.target_system,  # target system
                        0,  # target component
                        frame,  # coordinate frame MAV_FRAME_BODY_NED | MAV_FRAME_BODY_OFFSET_NED
                        3580,  # type mask (pos only)
                        distance_x,  # position x
                        distance_y,  # position y
                        0,  # position z
                        0, 0, 0,  # velocity x,y,z
                        0, 0, 0,  # accel x,y,z
                        0, 0)  # yaw, yaw rate
            elif cmd in ("goto", "go"):
                if len(args) not in (3, 5):
                    self.send_sms(sender, f"Usage: {cmd} [-]LAT [-]LON or {cmd} LAT° (S|N), LON° (E|W)")
                    return "OK"
                if self.master.flightmode != 'GUIDED':
                    self.send_sms(sender, f"Not in Guided mode {self.master.flightmode}")
                    return "OK"
                if len(args) == 3:
                    lat = float(args[1])
                    lon = float(args[2])
                elif len(args) == 5:
                    lat = float(args[1].rstrip('°')) * (1 if args[2][0] == 'N' else -1)
                    lon = float(args[3].rstrip('°')) * (1 if args[4][0] == 'E' else -1)
                else:
                    lat = None
                    lon = None
                self.send_sms(sender, f"Going to {lat:.7f} {lon:.7f}")
                if lat is not None and lon is not None:
                    self.master.mav.set_position_target_global_int_send(
                        0,  # system time in milliseconds
                        self.settings.target_system,  # target system
                        0,  # target component
                        0,  # coordinate frame MAV_FRAME_BODY_NED
                        3580,  # type mask (pos only)
                        int(lat * 1e7), int(lon * 1e7), 0,  # position x,y,z
                        0, 0, 0,  # velocity x,y,z
                        0, 0, 0,  # accel x,y,z
                        0, 0)  # yaw, yaw rate
            elif cmd == 'pause':
                self.master.set_mode('LOITER')
            elif cmd == 'speed':
                if len(args) != 2:
                    self.send_sms(sender, "Usage: speed X (m/s)")
                    return "OK"
                self.cmdlong.cmd_do_change_speed(args[1:])
            elif cmd in ('start', 'resume'):
                if self.master.flightmode == 'AUTO':
                    self.master.set_mode('LOITER')
                self.master.set_mode('AUTO')
            elif cmd == 'wp':
                if len(args) != 2:
                    self.send_sms(sender, "Usage: wp N")
                    return "OK"
                self.master.waypoint_set_current_send(int(args[1]))
            else:
                self.send_sms(sender, f"Unknown command {cmd}")
        # TODO process apple/google share position

        # self.send_sms(sender, "Thanks!")
        return "Thanks"

    def send_sms(self, to, message):
        requests.get(f"http://{self.teltonika_settings.router}:{self.teltonika_settings.port}/cgi-bin/sms_send",
                     params=dict(
                         username=self.teltonika_settings.username,
                         password=self.teltonika_settings.password,
                         number=to,
                         text=message
                     ))

    def log(self, msg, level=0):
        if self.mpstate.settings.moddebug < level:
            return

        print('{}: {}'.format(__name__, msg))

    def log_mobile(self, data):
        """optionally log rtcm data"""
        if self.teltonika_settings.logfile is None:
            return
        if self.logfile is None:
            self.logfile = open(self.teltonika_settings.logfile, 'wt')
        if self.logfile is not None:
            self.logfile.write(json.dumps(data) + "\n")

    def _json_rpc(self, method, obj, action, **parameters):
        result = requests.post(
            f'http://{self.teltonika_settings.router}:{self.teltonika_settings.port}/{self.teltonika_settings.path}',
            json={"jsonrpc": "2.0", "id": self.request_num, "method": method,
                  "params": [self.session or "00000000000000000000000000000000", obj, action, parameters]}).json()
        self.request_num += 1
        self.log(f"Response to {method} {obj} {action}: {result}", 3)
        if 'error' in result:
            raise JsonRPCError(result['error']['code'], result['error']['message'])
        return result['result'][0], result['result'][1] if len(result['result']) > 1 else None

    def idle_task(self):
        """called on idle"""
        if not self.started:
            return
        now = time.time()
        if self.session is None:
            if now - self.last_login > 5:
                self.last_login = now
                login_result, login_response = self._json_rpc("call", "session", "login",
                                                              username=self.teltonika_settings.username,
                                                              password=self.teltonika_settings.password)
                if login_result == 0:
                    self.session = login_response["ubus_rpc_session"]
                    print("Teltonika: session started")
                else:
                    print("Teltonika: Failed to login")
                    return
        if now - self.last_update > 1 / self.teltonika_settings.freq:
            try:
                if self.modem is None:
                    # result, info = self._json_rpc("call", "vuci.network.mobile", "get_all_modems")
                    # for modem in info['modems']:
                    #     self.modem = modem['id']
                    #     print(f"Teltonika: modem {self.modem} selected")
                    #     break
                    result, info = self._json_rpc("call", "uci", "get", config='network')
                    if result == 0:
                        networks = info['values']
                        for network in networks.values():
                            if 'modem' in network:
                                self.modem = network['modem']
                                print(f"Teltonika: modem {self.modem} selected")
                                break
                if self.modem is None:
                    return
                result, info = self._json_rpc("call", "vuci.network.mobile", "mobile_info", modem=self.modem)
                if result == 0:
                    if self.teltonika_settings.sendalllinks:
                        links = self.mpstate.mav_master
                    else:
                        links = [self.master]
                    if self.teltonika_settings.sendalloutputs:
                        links += self.mpstate.mav_outputs
                    self.log(json.dumps(info["mobile"]), 2)
                    rssi = -(_any_int(info['mobile'], 'signal', 'rsrp', 'rscp', 'rsrq') or -255)
                    snr = abs(_any_int(info['mobile'], 'sinr') or -255)
                    imsi = info['mobile']['imsi'].strip()
                    mcc = imsi[:3]
                    mnc = imsi[3:5]
                    self.last_status = info['mobile']
                    self.last_update = now
                    for link in links:
                        if hasattr(link.mav, "cellular_status_send"):
                            link.mav.cellular_status_send(12, 0, 4, rssi, int(mcc), int(mnc),
                                                          0)  # TODO get status, mode
                        else:
                            link.mav.radio_status_send(rssi, 255, 100, snr, 255, 0, 0)
                # self._json_rpc("call", "file", "exec", command="zerotier-cli", params=["-j", "listnetworks"])
            except JsonRPCError as e:
                if e.code == '-32002':
                    print("Teltonika: Invalid Session")
                    self.session = None
        if self.session is not None:
            if now - self.last_status_message_update > 30:
                position = self.master.messages.get('GLOBAL_POSITION_INT')
                if position is not None:
                    try:
                        sr, _ = self._json_rpc("call", "uci", "set", config="sms_utils", section="cfg0392bd", values=dict(
                            message=f"%rn\n%cs over %ct at %ss\nCELL %ci\n{position.lat / 1e7:.7f} {position.lon / 1e7:.7f} @ {datetime.datetime.now().isoformat(timespec='seconds')}"))
                        sr2, _ = self._json_rpc("call", "uci", "set", config="events_reporting", section="cfg0292bd", values=dict(
                            message=f"%ts %rn %ex %ss\nCELL %ci\n{position.lat / 1e7:.7f} {position.lon / 1e7:.7f} @ {datetime.datetime.now().isoformat(timespec='seconds')}"))
                        ar, _ = self._json_rpc("call", "uci", "apply", timeout=10)
                        if sr != 0 or sr2 != 0 or ar != 0:
                            print(f"Status set: {sr}|{sr2}, applied: {ar}")
                        self.last_status_message_update = now
                    except JsonRPCError as e:
                        if e.code == '-32002':
                            print("Teltonika: Invalid Session")
                            self.session = None
                else:
                    print("Position unknown")

    def cmd_teltonika(self, args):
        """teltonika command handling"""
        if len(args) <= 0:
            print("Usage: teltonika <start|stop|status|set>")
            return
        if args[0] == "start":
            self.cmd_start()
        if args[0] == "stop":
            self.started = False
            self.session = None
            self.last_status = None
        elif args[0] == "status":
            self.teltonika_status()
        elif args[0] == "set":
            self.teltonika_settings.command(args[1:])

    def teltonika_status(self):
        """show teltonika status"""
        now = time.time()
        if not self.started:
            print("Teltonika: Not started")
            return
        elif self.last_status is None:
            print("Teltonika: no data")
            return
        key_length = max([len(k) for k in self.last_status.keys()])
        for key, value in self.last_status.items():
            print(f"{key:{key_length}s}: {value.strip()}")

    def cmd_start(self):
        """start teltonika link"""
        if self.teltonika_settings.router is None:
            print("Require router")
            return
        self.started = True


def init(mpstate):
    """initialise module"""
    return TeltonikaModule(mpstate)


def _any_int(info, *keys) -> typing.Optional[int]:
    for key in keys:
        value = info.get(key)
        if value is not None:
            try:
                return int(float(value.strip()))
            except ValueError:
                pass
    return None
