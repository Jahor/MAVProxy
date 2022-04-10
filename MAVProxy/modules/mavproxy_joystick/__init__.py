from __future__ import print_function

import os
import pygame
import pkg_resources
import yaml
import fnmatch

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from MAVProxy.modules.mavproxy_joystick import controls


class Joystick(mp_module.MPModule):
    '''
    joystick set verbose
    joystick set debug
    joystick status
    joystick probe
    '''

    def __init__(self, mpstate):
        """Initialise module"""
        super(Joystick, self).__init__(mpstate, 'joystick',
                                       'A flexible joystick driver')

        self.joystick = None
        self.joystick_settings = None

        self.init_pygame()
        self.init_settings()
        self.init_commands()
        self.load_definitions()
        self.probe()

    def log(self, msg, level=0):
        if self.mpstate.settings.moddebug < level:
            return

        print('{}: {}'.format(__name__, msg))

    def init_pygame(self):
        self.log('Initializing pygame', 2)
        pygame.init()

    def init_settings(self):
        self.joystick_settings = mp_settings.MPSettings(
                    [('autoprobe', bool, False)])
        self.add_completion_function('(JOYSTICKSETTING)',
                    self.joystick_settings.completion)

    def init_commands(self):
        self.log('Initializing commands', 2)
        self.add_command('joystick', self.cmd_joystick,
                         "A flexible joystick drvier",
                         ['status',  'probe', 'set (JOYSTICKSETTING)'])

    def load_definitions(self):
        self.log('Loading joystick definitions', 1)

        self.joydefs = []
        search = []

        userjoysticks = os.environ.get(
            'MAVPROXY_JOYSTICK_DIR',
            mp_util.dot_mavproxy('joysticks'))
        if userjoysticks is not None and os.path.isdir(userjoysticks):
            search.append(userjoysticks)

        search.append(pkg_resources.resource_filename(__name__, 'joysticks'))

        for path in search:
            self.log('Looking for joystick definitions in {}'.format(path),
                     2)
            path = os.path.expanduser(path)
            for dirpath, dirnames, filenames in os.walk(path):
                for joyfile in filenames:
                    root, ext = os.path.splitext(joyfile)
                    if ext[1:] not in ['yml', 'yaml', 'json']:
                        continue

                    joypath = os.path.join(dirpath, joyfile)
                    self.log('Loading definition from {}'.format(joypath), 2)
                    with open(joypath, 'r') as fd:
                        joydef = yaml.safe_load(fd)
                        joydef['path'] = joypath
                        self.joydefs.append(joydef)

    def probe(self, quiet=False):
        pygame.joystick.quit()
        pygame.joystick.init()
        for jid in range(pygame.joystick.get_count()):
            joy = pygame.joystick.Joystick(jid)
            self.log("Found joystick (%s)" % (joy.get_name(),))
            for joydef in self.joydefs:
                if 'match' not in joydef:
                    self.log('{} has no match patterns, ignoring.'.format(
                        joydef['path']), 0)
                    continue
                for pattern in joydef['match']:
                    if fnmatch.fnmatch(joy.get_name().lower(),
                                       pattern.lower()):
                        self.log('Using {} ("{}" matches pattern "{}")'.format(
                            joydef['path'], joy.get_name(), pattern))
                        self.joystick = controls.Joystick(joy, joydef)
                        return
        if not quiet:
            print('{}: Failed to find matching joystick.'.format(__name__))

    def usage(self):
        '''show help on command line options'''
        return "Usage: joystick <status|set>"

    def cmd_joystick(self, args):
        if not len(args):
            self.log('No subcommand specified.')
        elif args[0] == 'status':
            self.cmd_status()
        elif args[0] == 'probe':
            self.cmd_probe()
        elif args[0] == 'help':
            self.cmd_help()
        elif args[0] == 'set':
            self.joystick_settings.command(args[1:])

    def cmd_help(self):
        print('joystick probe -- reload and match joystick definitions')
        print('joystick status -- show currently loaded definition, if any')

    def cmd_probe(self):
        self.log('Re-detecting available joysticks', 0)
        self.load_definitions()
        self.probe()

    def cmd_status(self):
        if self.joystick is None:
            print('No active joystick')
        else:
            print('Active joystick:')
            print('Path: {path}'.format(**self.joystick.controls))
            print('Description: {description}'.format(
                **self.joystick.controls))
            print('instance id: %s; power: %s; name: %s' % (self.joystick.joystick.get_instance_id(), self.joystick.joystick.get_power_level(), self.joystick.joystick.get_name()))

    def detect_disconnected(self):
        if self.joystick is not None:
            found = False
            for jid in range(pygame.joystick.get_count()):
                joy = pygame.joystick.Joystick(jid)
                if joy.get_guid() == self.joystick.joystick.get_guid():
                    if joy.get_instance_id() == self.joystick.joystick.get_instance_id():
                        found = True
                        break
                    else:
                        self.joystick = controls.Jostick(joy, self.joystick.controls)
                        print("Joystick %s has been reconnected", self.joystick.joystick.get_name())
                        break
            if not found:
                print("Joystick %s has been disconnected" % self.joystick.joystick.get_name())
                self.joystick = None


    def idle_task(self):
        self.detect_disconnected()
        if self.joystick is None:
            if self.joystick_settings.autoprobe:
                self.probe(quiet=True)
            if self.joystick is None:
                return

        for e in pygame.event.get():
            override = self.module('rc').override[:]
            values = self.joystick.read()
            override = values + override[len(values):]

            # self.log('channels: {}'.format(override), level=3)

            if override != self.module('rc').override:
                self.module('rc').override = override
                self.module('rc').override_period.force()

    def unload(self):
        super(Joystick, self).unload()
        self.log('Uninitializing pygame', 2)
        pygame.joystick.quit()
        pygame.quit()

def init(mpstate):
    '''initialise module'''
    return Joystick(mpstate)
