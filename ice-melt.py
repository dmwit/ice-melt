from configparser import ConfigParser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Lock, Thread, Timer

import html
import urllib.parse as url

SENSOR_PREFIX = 'sensor'
CONTROL_PREFIX = 'control'
STATE_PREFIX = 'state'
PID_PREFIX = 'pid controller'
ACTION_PREFIX = 'user clicks'

class SensorConfig:
    def __init__(self, name, section):
        self.name = name
        self.raw_lo = section.getint('raw low', 0)
        self.raw_hi = section.getint('raw high')
        self.sho_lo = section.getint('display low', 0)
        self.sho_hi = section.getint('display high', 100)
        default_format = '{:.0f}%' if self.sho_lo == 0 and self.sho_hi == 100 else '{}'
        self.format = section.get('format', default_format, raw=True)

    def __lerp__(self, raw):
        return (raw - self.raw_lo) / (self.raw_hi - self.raw_lo) * (self.sho_hi - self.sho_lo) + self.sho_lo
    def lerp(self, raw):
        return min(self.sho_hi, max(self.sho_lo, self.__lerp__(raw)))

    def html(self, raw):
        disp = self.__lerp__(raw)
        s = self.format.format(disp)
        if disp < self.sho_lo:
            s = '<' + self.format.format(self.sho_lo)
        if disp > self.sho_hi:
            s = '>' + self.format.format(self.sho_hi)
        return s

class ControlConfig:
    def __init__(self, name, section):
        self.name = name
        self.raw_lo = section.getint('raw low', 0)
        self.raw_hi = section.getint('raw high')
        self.sho_lo = section.getint('display low', 0)
        self.sho_hi = section.getint('display high', 100)
        default_format = '{:.0f}%' if self.sho_lo == 0 and self.sho_hi == 100 else '{}'
        self.format = section.get('format', default_format, raw=True)

    def __lerp__(self, sho):
        return (sho - self.sho_lo) / (self.sho_hi - self.sho_lo) * (self.raw_hi - self.raw_lo) + self.raw_lo
    def lerp(self, sho):
        return min(self.raw_hi, max(self.raw_lo, self.__lerp__(sho)))

    def html(self, sho):
        s = self.format.format(sho)
        clipped_format = ('{} (clipped from {})').format(self.format, s)
        if sho < self.sho_lo:
            s = clipped_format.format(self.sho_lo)
        if sho > self.sho_hi:
            s = clipped_format.format(self.sho_hi)
        return s

class PIDConfig:
    def __init__(self, name, section):
        self.__name__ = name
        self.__cfg__ = section

    def name(self): return self.__name__
    def p(self): return self.__cfg__.getfloat('proportional gain', 1)
    def i(self): return self.__cfg__.getfloat('integral gain', 0.1)
    def d(self): return self.__cfg__.getfloat('derivative gain', 0.05)
    def pom(self): return self.__cfg__.getbool('proportional on measurement', false)
    def hz(self): return self.__cfg__.getfloat('control calculations per second', 0.1)
    def sensor(self): return self.__cfg__[SENSOR_PREFIX]

class SensorTrigger:
    def __init__(self, sensor, direction, threshold):
        self.sensor = sensor
        self.direction = direction
        self.threshold = threshold

    def triggers(self, val):
        return self.direction * val > self.direction * self.threshold

def parse_sensor_trigger(description):
    if not description.startswith(SENSOR_PREFIX): return None
    noitpircsed = description[len(description)::-1]
    evoba_index = noitpircsed.find('evoba')
    woleb_index = noitpircsed.find('woleb')
    if evoba_index < 0 and woleb_index < 0: return None
    elif woleb_index < 0 or (evoba_index >= 0 and evoba_index < woleb_index):
        direction = 1
        direction_index = evoba_index + len('above')
    else:
        direction = -1
        direction_index = woleb_index + len('below')
    direction_index = len(description) - direction_index - 1
    sensor_name = description[len(SENSOR_PREFIX):direction_index].strip()
    try: threshold = float(description.split()[-1])
    except ValueError: return None
    return SensorTrigger(sensor_name, direction, threshold)

class StateConfig:
    def __init__(self, name, section):
        self.__name__ = name
        self.__cfg__ = section

        self.__timed_transition__ = None
        if 'time passes' in self.__cfg__ and 'duration' in self.__cfg__:
            try: duration = float(self.__cfg__['duration'])
            except ValueError: pass
            else:
                self.__timed_transition__ = (self.__cfg__['time passes'], duration)

        self.__triggers__ = [(trigger, tgt)
            for desc, tgt in self.__cfg__.items()
            if (trigger := parse_sensor_trigger(desc)) is not None
            ]

    def name(self): return self.__name__

    def control_settings(self):
        return [(nm[len(CONTROL_PREFIX):].strip(), int(val))
            for nm, val in self.__cfg__.items()
            if nm.startswith(CONTROL_PREFIX)
            ]

    def actions(self):
        return [(nm[len(ACTION_PREFIX):].strip(), val)
            for nm, val in self.__cfg__.items()
            if nm.startswith(ACTION_PREFIX)
            ]

    def timer(self, state):
        if self.__timed_transition__ is None: return None
        tgt, dur = self.__timed_transition__
        t = Timer(dur, IceMeltState.transition, [state, self.__name__, tgt])
        t.start()
        return t

    def actions_html(self):
        return ''.join('<li><button data-hx-trigger="click" data-hx-get="/transition/{}/{}" data-hx-target="body" data-hx-swap="innerHTML">{}</button></li>'.format(
                url.quote(self.__name__),
                url.quote(tgt),
                html.escape(description))
            for description, tgt in
            self.actions()
            )

    def trigger(self, sensor, val):
        for trigger, tgt in self.__triggers__:
            if sensor == trigger.sensor and trigger.triggers(val):
                return tgt
        return None

    # list() is defensive programming, so that modifications to the returned
    # thing are less likely to mess with our internal state
    def triggers(self): return list(self.__triggers__)

class IceMeltConfig:
    def __init__(self, filename):
        self.__cfg__ = ConfigParser()
        self.__cfg__.optionxform = lambda s: s
        self.__cfg__.read(filename)

    def __prefixed_sections__(self, prefix, factory):
        return [
            ( nm := sec_nm[len(prefix):].strip()
            , factory(nm, self.__cfg__[sec_nm])
            )
            for sec_nm in self.__cfg__.sections()
            if sec_nm.startswith(prefix)
            ]

    def sensors(self): return self.__prefixed_sections__(SENSOR_PREFIX, SensorConfig)
    def controls(self): return self.__prefixed_sections__(CONTROL_PREFIX, ControlConfig)
    def states(self): return self.__prefixed_sections__(STATE_PREFIX, StateConfig)
    def pid_controllers(self): return self.__prefixed_sections__(PID_PREFIX, PIDConfig)

    def server_address(self):
        try: host = self.__cfg__['server']['host']
        except KeyError: host = 'localhost'

        try: port = self.__cfg__['server'].getint('port')
        except KeyError: port = 80
        except ValueError: port = 80

        return (host, port)

    def initial_state(self):
        return self.__cfg__['server']['initial state']

class SensorState:
    def __init__(self, cfg):
        self.__cfg__ = cfg
        self.__as_of__ = None
        self.__val__ = 0

    def as_of_html(self):
        return 'no reading yet' if self.__as_of__ is None else str(self.__as_of__)

    def val_float(self):
        if self.__as_of__ is None: raise ValueError
        return self.__cfg__.lerp(self.__val__)

    def val_html(self):
        return '?' if self.__as_of__ is None else self.__cfg__.html(self.__val__)

    def html(self):
        return '<tr><td>{}</td><td>{}</td><td>{}</td></tr>' \
            .format(html.escape(self.__cfg__.name), self.val_html(), self.as_of_html())

    def set_raw(self, val):
        self.__val__ = val
        self.__as_of__ = datetime.now()

    def name(self): return self.__cfg__.name

class ControlState:
    def __init__(self, cfg):
        self.__cfg__ = cfg
        self.__as_of__ = None
        self.__val__ = 0

    def as_of_html(self):
        return 'no control signal sent yet' if self.__as_of__ is None else str(self.__as_of__)

    def val_html(self):
        return '?' if self.__as_of__ is None else self.__cfg__.html(self.__val__)

    def html(self):
        return '<tr><td>{}</td><td>{}</td><td>{}</td></tr>' \
            .format(html.escape(self.__cfg__.name), self.val_html(), self.as_of_html())

    def set_value(self, val):
        self.__val__ = val
        self.__as_of__ = datetime.now()
        # TODO: send to logger
        print('set {} to {}'.format(self.__cfg__.name, self.__cfg__.lerp(val)))

class PIDState:
    def __init__(self, cfg, lock):
        self.__lock__ = lock
        # TODO

class IceMeltState:
    def __init__(self, cfg):
        self.__current_state__ = None
        self.__timer__ = None
        self.__lock__ = Lock()
        self.__cfg__ = cfg
        self.__sensors__ = {nm: SensorState(scfg) for nm, scfg in cfg.sensors()}
        self.__controls__ = {nm: ControlState(scfg) for nm, scfg in cfg.controls()}
        self.__states__ = dict(cfg.states())
        # TODO: self.__pids__ = {nm: PIDState(scfg, self.__lock__) for nm, scfg in cfg.pid_controllers()}
        self.transition(None, cfg.initial_state())

    def set_sensor(self, sensor, value):
        with self.__lock__:
            src = self.__current_state__
            try: ss = self.__sensors__[sensor]
            except KeyError: tgt = None
            else:
                ss.set_raw(value)
                tgt = self.current_state().trigger(sensor, ss.val_float())

        # done outside the lock, because transition takes the lock
        if tgt is not None:
            self.transition(src, tgt)

    def current_state(self): return self.__states__[self.__current_state__]

    def transition(self, src, tgt):
        new_tgt = None

        with self.__lock__:
            if self.__current_state__ == src and tgt in self.__states__:
                self.__current_state__ = tgt
                scfg = self.__states__[tgt]
                for nm, val in scfg.control_settings():
                    self.__controls__[nm].set_value(val)
                if self.__timer__ is not None: self.__timer__.cancel()
                self.__timer__ = scfg.timer(self)
                self.__last_transition__ = datetime.now()

                # check if any of the current sensor values means we should
                # immediately transition again
                for trigger, new_tgt in scfg.triggers():
                    try:
                        if trigger.triggers(self.__sensors__[trigger.sensor].val_float()):
                            break
                    except ValueError: pass
                    except KeyError: pass
                else: new_tgt = None

        if new_tgt is not None: self.transition(tgt, new_tgt)

    def content_html(self):
        with open('content-only.html', 'r', encoding='utf-8') as f:
            template = f.read()
        with self.__lock__:
            state_html = '{} (since {})'.format(
                html.escape(self.__current_state__),
                self.__last_transition__
                )
            sensors_html = ''.join(ss.html() for ss in self.__sensors__.values())
            controls_html = ''.join(cs.html() for cs in self.__controls__.values())
            actions_html = self.current_state().actions_html()
        return template.format(state=state_html, sensors=sensors_html, controls=controls_html, actions=actions_html)

    def doc_html(self):
        with open('index.html', 'r', encoding='utf-8') as f:
            return f.read().format(content=self.content_html())

class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global state

        if self.path == '/':
            self.send_response(200)
            self.send_header('content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(state.doc_html().encode())

        elif self.path == '/content-only':
            self.send_response(200)
            self.send_header('content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(state.content_html().encode())

        elif self.path.startswith('/transition/') and len(self.path.split('/')) == 4:
            self.send_response(200)
            self.send_header('content-type', 'text/html; charset=utf-8')
            self.end_headers()
            _, _, src, tgt = self.path.split('/')
            state.transition(src, tgt)
            self.wfile.write(state.content_html().encode())

        elif self.path == '/htmx.js':
            try:
                f = open('htmx.js', 'rb')
                js = f.read()
            except:
                self.send_response(500)
                self.end_headers()
            else:
                self.send_response(200)
                self.send_header('content-type', 'application/javascript')
                self.end_headers()
                self.wfile.write(js)
                f.close()

        else:
            self.send_response(404)
            self.end_headers()

# TODO: receive from logger
def read_sensors_forever():
    global state
    while True:
        line = input()
        ix = [i for i, c in enumerate(line) if c == ' '][-1]
        sensor = line[:ix]
        value = line[ix+1:]
        try: value = int(value)
        except ValueError: pass
        else: state.set_sensor(sensor, value)

SAFE_ID_CHARS = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_')
def escape_id(s):
    return 'a' + ''.join(
        c if c in SAFE_ID_CHARS else '-{}-'.format(ord(c))
        for c in s
        )

cfg = IceMeltConfig('ice-melt.ini')
state = IceMeltState(cfg)
Thread(target = read_sensors_forever, daemon=True).start()
HTTPServer(cfg.server_address(), RequestHandler).serve_forever()
