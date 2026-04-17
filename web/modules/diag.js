// Diagnostics panel — only active when URL has ?diag=1
// Provides developer controls for buzzer, LED, GPIO, I2C, and aux servos.

import { send } from './ws.js';

var _enabled = new URLSearchParams(window.location.search).get('diag') === '1';

// Circular buffer for last 10 telem lines
var _telemLines = [];
var _telemEl = null;

function pushTelem(msg) {
    _telemLines.push(JSON.stringify(msg));
    if (_telemLines.length > 10) _telemLines.shift();
    if (_telemEl) _telemEl.textContent = _telemLines.join('\n');
}

// --- Builders ---

function el(tag, attrs, children) {
    var e = document.createElement(tag);
    if (attrs) {
        Object.keys(attrs).forEach(function(k) {
            if (k === 'class') e.className = attrs[k];
            else if (k === 'style') e.style.cssText = attrs[k];
            else e.setAttribute(k, attrs[k]);
        });
    }
    if (children) {
        children.forEach(function(c) {
            if (typeof c === 'string') e.appendChild(document.createTextNode(c));
            else if (c) e.appendChild(c);
        });
    }
    return e;
}

function input(attrs) { return el('input', attrs); }
function label(text, forId) {
    var l = document.createElement('label');
    l.textContent = text;
    if (forId) l.setAttribute('for', forId);
    return l;
}
function btn(text, onClick) {
    var b = el('button', {'class': 'diag-btn'}, [text]);
    b.addEventListener('click', onClick);
    return b;
}
function row(children) { return el('div', {'class': 'diag-row'}, children); }
function section(title, children) {
    return el('div', {'class': 'diag-section'}, [
        el('h3', {'class': 'diag-section-title'}, [title])
    ].concat(children));
}

// --- Buzzer section ---

function buildBuzzer() {
    var freqIn = input({'type': 'number', 'id': 'diag-buzz-freq', 'value': '2400', 'min': '31', 'max': '20000', 'class': 'diag-input'});
    var durIn  = input({'type': 'number', 'id': 'diag-buzz-dur',  'value': '200',  'min': '1',  'max': '5000',  'class': 'diag-input'});

    return section('Buzzer', [
        row([label('Freq Hz', 'diag-buzz-freq'), freqIn,
             label('Duration ms', 'diag-buzz-dur'), durIn,
             btn('Beep', function() {
                 send({ type: 'cmd_buzzer', freq_hz: parseInt(freqIn.value, 10), duration_ms: parseInt(durIn.value, 10) });
             }),
             btn('Stop', function() {
                 send({ type: 'cmd_buzzer', freq_hz: 0, duration_ms: 0 });
             })
        ])
    ]);
}

// --- LED section ---

function buildLED() {
    var rIn  = input({'type': 'number', 'id': 'diag-led-r', 'value': '0', 'min': '0', 'max': '255', 'class': 'diag-input diag-input-sm'});
    var gIn  = input({'type': 'number', 'id': 'diag-led-g', 'value': '0', 'min': '0', 'max': '255', 'class': 'diag-input diag-input-sm'});
    var bIn  = input({'type': 'number', 'id': 'diag-led-b', 'value': '0', 'min': '0', 'max': '255', 'class': 'diag-input diag-input-sm'});
    var idxSel = el('select', {'class': 'diag-select'}, [
        el('option', {'value': '0'}, ['0 — onboard blue']),
        el('option', {'value': '1'}, ['1 — external'])
    ]);

    return section('LED', [
        row([
            label('R'), rIn,
            label('G'), gIn,
            label('B'), bIn,
            label('LED'), idxSel,
            btn('Set LED', function() {
                send({
                    type: 'cmd_led',
                    led: parseInt(idxSel.value, 10),
                    r: parseInt(rIn.value, 10),
                    g: parseInt(gIn.value, 10),
                    b: parseInt(bIn.value, 10)
                });
            })
        ])
    ]);
}

// --- GPIO section ---

var _gpioReadout = null;

function buildGPIO() {
    var pinSel = el('select', {'class': 'diag-select'}, [
        el('option', {'value': '32'}, ['32']),
        el('option', {'value': '33'}, ['33']),
        el('option', {'value': '1'},  ['1']),
        el('option', {'value': '3'},  ['3'])
    ]);
    var modeSel = el('select', {'class': 'diag-select'}, [
        el('option', {'value': 'input_floating'},  ['input_floating']),
        el('option', {'value': 'input_pullup'},    ['input_pullup']),
        el('option', {'value': 'input_pulldown'},  ['input_pulldown']),
        el('option', {'value': 'output'},          ['output'])
    ]);
    var valIn = input({'type': 'number', 'value': '0', 'min': '0', 'max': '1', 'class': 'diag-input diag-input-sm'});
    var readout = el('pre', {'class': 'diag-readout'}, ['—']);
    _gpioReadout = readout;

    function pin() { return parseInt(pinSel.value, 10); }

    return section('GPIO', [
        row([
            label('Pin'), pinSel,
            label('Mode'), modeSel,
            btn('Set Mode', function() {
                send({ type: 'cmd_gpio', op: 'mode', pin: pin(), mode: modeSel.value });
            })
        ]),
        row([
            label('Value'), valIn,
            btn('Write', function() {
                send({ type: 'cmd_gpio', op: 'write', pin: pin(), value: parseInt(valIn.value, 10) });
            }),
            btn('Read', function() {
                send({ type: 'cmd_gpio', op: 'read', pin: pin() });
            }),
            btn('Analog', function() {
                send({ type: 'cmd_gpio', op: 'analog', pin: pin() });
            }),
            btn('Subscribe', function() {
                send({ type: 'cmd_gpio', op: 'subscribe', pin: pin() });
            }),
            btn('Unsubscribe', function() {
                send({ type: 'cmd_gpio', op: 'unsubscribe', pin: pin() });
            })
        ]),
        readout
    ]);
}

// --- I2C section ---

var _i2cReadout = null;

function buildI2C() {
    var busSel = el('select', {'class': 'diag-select'}, [
        el('option', {'value': '1'}, ['Bus 1']),
        el('option', {'value': '2'}, ['Bus 2'])
    ]);
    var addrIn = input({'type': 'number', 'value': '0', 'min': '0', 'max': '127', 'class': 'diag-input diag-input-sm', 'placeholder': 'addr'});
    var regIn  = input({'type': 'number', 'value': '0', 'min': '0', 'max': '255', 'class': 'diag-input diag-input-sm', 'placeholder': 'reg'});
    var valIn  = input({'type': 'number', 'value': '0', 'min': '0', 'max': '255', 'class': 'diag-input diag-input-sm', 'placeholder': 'val'});
    var readout = el('pre', {'class': 'diag-readout'}, ['—']);
    _i2cReadout = readout;

    function bus() { return parseInt(busSel.value, 10); }

    return section('I2C', [
        row([
            label('Bus'), busSel,
            btn('Scan', function() {
                send({ type: 'cmd_i2c', op: 'scan', bus: bus() });
            }),
            label('Addr'), addrIn,
            label('Reg'), regIn,
            label('Val'), valIn,
            btn('Read', function() {
                send({ type: 'cmd_i2c', op: 'read', bus: bus(), addr: parseInt(addrIn.value, 10), reg: parseInt(regIn.value, 10) });
            }),
            btn('Write', function() {
                send({ type: 'cmd_i2c', op: 'write', bus: bus(), addr: parseInt(addrIn.value, 10), reg: parseInt(regIn.value, 10), val: parseInt(valIn.value, 10) });
            })
        ]),
        readout
    ]);
}

// --- Aux servo section ---

function buildAuxServo() {
    var idxSel = el('select', {'class': 'diag-select'}, [
        el('option', {'value': '8'},  ['8']),
        el('option', {'value': '9'},  ['9']),
        el('option', {'value': '10'}, ['10'])
    ]);
    var pulseIn = input({'type': 'number', 'value': '1500', 'min': '500', 'max': '2500', 'class': 'diag-input'});

    return section('Aux Servo (idx 8–10)', [
        row([
            label('Index'), idxSel,
            label('Pulse µs'), pulseIn,
            btn('Write', function() {
                send({ type: 'cmd_servo', index: parseInt(idxSel.value, 10), pulse_us: parseInt(pulseIn.value, 10) });
            })
        ])
    ]);
}

// --- Telem log ---

function buildTelemLog() {
    var pre = el('pre', {'id': 'diag-telem'}, ['(waiting for telem_gpio / telem_i2c / telem_button)']);
    _telemEl = pre;
    return el('div', {'class': 'diag-section'}, [
        el('h3', {'class': 'diag-section-title'}, ['Incoming Telemetry']),
        pre
    ]);
}

// --- Panel assembly ---

export function diagInit() {
    if (!_enabled) return;

    var panel = el('div', {'id': 'diag-panel', 'class': 'diag-visible'}, [
        el('div', {'class': 'diag-header'}, [
            el('span', {'class': 'diag-title'}, ['DEV DIAGNOSTICS']),
            el('span', {'class': 'diag-hint'}, ['?diag=1'])
        ]),
        buildBuzzer(),
        buildLED(),
        buildGPIO(),
        buildI2C(),
        buildAuxServo(),
        buildTelemLog()
    ]);

    document.body.appendChild(panel);
}

export function diagHandleTelem(msg) {
    if (!_enabled) return;
    var t = msg.type;
    if (t === 'telem_gpio' || t === 'telem_i2c' || t === 'telem_button') {
        pushTelem(msg);
        if (t === 'telem_gpio' && _gpioReadout) {
            _gpioReadout.textContent = JSON.stringify(msg);
        }
        if (t === 'telem_i2c' && _i2cReadout) {
            _i2cReadout.textContent = JSON.stringify(msg);
        }
    }
}
