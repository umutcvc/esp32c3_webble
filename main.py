# === ESP32-C3 Bluetooth PWM Controller + IMU Pitch (MicroPython) ===
# Added commands (non-breaking):
#   CAL[,ms]            → calibrate zero by averaging for ms (default 3000)
#   STREAM,ON|OFF       → optional stream control (default ON like original)
# Existing:
#   PWM,<freq>,<duty>   → start/update PWM on PIN_NUM
#   STOP                → stop PWM (drive LOW)
# IMU: streams "PITCH,<deg>" at PRINT_HZ rate over BLE TX notify.

from machine import Pin, PWM, UART, ADC
from micropython import const
import ubluetooth as bt
import time, struct
from micropyGPS import MicropyGPS

# ---------- USER CONFIG ----------
NAME     = "DevOpBreadBoard"
PIN_NUM  = 3         # your working PWM pin
# BNO08x RVC (change RX/TX if your wiring is different)
IMU_UID  = 1         # UART id to use
IMU_RX   = 6         # IMU TX -> ESP RX
IMU_TX   = 6         # not really used by RVC, but UART needs a TX pin
BAUD     = 115200
PRINT_HZ = 100        # pitch send rate over BLE
EMA_A    = 0.5       # exponential smoothing factor 0..1

# --- GPS config (BE-180 or similar NMEA GPS) ---
GPS_UID    = 0         # Different UART from IMU
GPS_BAUD   = 38400     # Common GPS baud rate
GPS_RX_PIN = 20        # GPS TX -> ESP32-C3 GPIO20
GPS_TX_PIN = 21        # GPS RX <- ESP32-C3 GPIO21 (optional)
GPS_HZ     = 10        # internal poll cadence (not NMEA rate)
# ---------------------------------

# ==== Battery ADC config (you finished 100k/100k divider) ====
VBAT_PIN  = 2        # <— connect divider midpoint here (GPIO4 is ADC-capable on C3)
DIVIDER   = 2.0      # 100k / 100k => halves the battery voltage
ADC_VMAX  = 3.60     # ESP32-C3 ~3.6 V full-scale with ATTN_11DB (good default)
CAL_K     = 0.871    # set later = (DMM_VBAT / GUI_VBAT)
_ADC_MAX  = 65535.0  # read_u16() full-scale
BATT_DT   = 2000     # send every 2 s

# Hold PWM pin LOW immediately on boot to avoid chirp
Pin(PIN_NUM, Pin.OUT, value=0)

# Battery ADC setup
try:
    _adc = ADC(Pin(VBAT_PIN))
    try: _adc.atten(ADC.ATTN_11DB)
    except: pass
except Exception:
    _adc = None  # keep safe if ADC/Pin missing

_vbat_ema = None
_batt_last = time.ticks_ms()


# ---------- BLE setup ----------
ble = bt.BLE()
ble.active(True)

# Nordic UART 128-bit UUIDs
_UART_UUID = bt.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_TX_UUID   = bt.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  # Notify
_RX_UUID   = bt.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  # Write

_UART_TX  = (_TX_UUID, bt.FLAG_NOTIFY)
_UART_RX  = (_RX_UUID, bt.FLAG_WRITE | bt.FLAG_WRITE_NO_RESPONSE)
_UART_SVC = (_UART_UUID, (_UART_TX, _UART_RX))

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

services  = ble.gatts_register_services((_UART_SVC,))
svc       = services[0]
tx_raw    = svc[0]
rx_raw    = svc[1]
tx_handle = tx_raw[0] if isinstance(tx_raw, (tuple, list)) else tx_raw
rx_handle = rx_raw[0] if isinstance(rx_raw, (tuple, list)) else rx_raw

conn_handle = None
pwm = None

# --- rate meters ---
imu_frames = 0
ble_msgs   = 0
rate_last  = time.ticks_ms()


# streaming + calibration state
stream_enabled = True   # default ON to preserve original behavior
cal_active = False
cal_deadline = 0
cal_sum = 0.0
cal_n = 0
pitch_offset = 0.0

def _notify(msg: str):
    try:
        if conn_handle is not None:
            ble.gatts_notify(conn_handle, tx_handle, (msg + "\n").encode())
    except:
        pass

def _start_pwm(freq, duty):
    """Start/update PWM safely (no blip)."""
    global pwm
    f = int(freq)
    d = max(0, min(100, int(duty)))
    if pwm is None:
        pwm = PWM(Pin(PIN_NUM))
        pwm.freq(f)
        pwm.duty_u16(0)  # start silent
    pwm.freq(f)
    pwm.duty_u16(int(d * 65535 // 100))
    _notify(f"ACK PWM {f}Hz {d}%")

def _stop_pwm():
    """Stop PWM cleanly and force pin LOW."""
    global pwm
    try:
        if pwm:
            try:
                pwm.duty_u16(0)
                time.sleep_ms(5)
            except:
                pass
            pwm.deinit()
            pwm = None
    finally:
        Pin(PIN_NUM, Pin.OUT).value(0)
    _notify("ACK STOP")

def _start_calibration(duration_ms=3000):
    """Begin averaging raw pitch for duration_ms; set as zero on finish."""
    global cal_active, cal_deadline, cal_sum, cal_n
    cal_active = True
    cal_deadline = time.ticks_add(time.ticks_ms(), int(duration_ms))
    cal_sum = 0.0
    cal_n = 0
    _notify(f"ACK CAL START {int(duration_ms)}ms")

def _update_calibration(raw_pitch_deg):
    """Accumulate raw pitch; when time elapses, set offset."""
    global cal_active, pitch_offset, cal_sum, cal_n
    if not cal_active:
        return
    cal_sum += raw_pitch_deg
    cal_n += 1
    if time.ticks_diff(cal_deadline, time.ticks_ms()) <= 0:
        if cal_n > 0:
            pitch_offset = cal_sum / cal_n
            _notify(f"ACK CAL DONE offset={pitch_offset:.2f}")
        else:
            _notify("ERR CAL no-data")
        cal_active = False

def on_ble(event, data):
    global conn_handle, stream_enabled, cal_active
    if event == _IRQ_CENTRAL_CONNECT:
        conn_handle, _, _ = data
        _notify("CONNECTED")
        # stream stays ON by default
    elif event == _IRQ_CENTRAL_DISCONNECT:
        conn_handle = None
        cal_active = False
        _stop_pwm()
        ble.gap_advertise(100_000, adv_data=adv, resp_data=resp)
    elif event == _IRQ_GATTS_WRITE:
        handle = data[1]
        if handle == rx_handle:
            try:
                msg = ble.gatts_read(rx_handle).decode().strip()
                if not msg: return
                _notify("RX:" + msg)
                up = msg.upper()
                if up.startswith("PWM"):
                    parts = msg.split(",")
                    if len(parts) != 3:
                        raise ValueError("PWM expects PWM,<freq>,<duty>")
                    _, f, d = parts
                    _start_pwm(f.strip(), d.strip())
                elif up.startswith("STOP"):
                    _stop_pwm()
                elif up.startswith("CAL"):
                    dur_ms = 3000
                    parts = msg.split(',')
                    if len(parts) >= 2:
                        try:
                            dur_ms = max(500, int(parts[1]))
                        except:
                            pass
                    _start_calibration(dur_ms)
                elif up.startswith("STREAM"):
                    # Optional: STREAM,ON or STREAM,OFF (default ON)
                    parts = up.split(',')
                    if len(parts) >= 2 and parts[1].strip() in ("ON", "OFF"):
                        stream_enabled = (parts[1].strip() == "ON")
                        _notify("ACK STREAM " + ("ON" if stream_enabled else "OFF"))
                    else:
                        raise ValueError("STREAM expects STREAM,ON or STREAM,OFF")
                else:
                    raise ValueError("Unknown cmd")
            except Exception as e:
                _notify("ERR " + str(e))
                
def _adc_read_u16_avg(n=8, settle_us=250):
    if _adc is None: return 0
    _ = _adc.read_u16()  # throw away first
    s = 0
    for _i in range(n):
        time.sleep_us(settle_us)
        s += _adc.read_u16()
    return s // max(1, n)

def _read_vbat_volts():
    if _adc is None: return 0.0
    raw   = _adc_read_u16_avg()
    v_pin = (raw / _ADC_MAX) * ADC_VMAX       # volts at ADC pin
    return (v_pin * DIVIDER) * CAL_K          # back to battery volts

def _vbat_tick(alpha=0.25):
    global _vbat_ema
    v = _read_vbat_volts()
    _vbat_ema = v if _vbat_ema is None else (_vbat_ema + alpha * (v - _vbat_ema))
    return _vbat_ema

def _vbat_percent(v):
    # Simple 1-cell Li-ion mapping (tweak if you like)
    if v >= 4.20: return 100
    if v <= 3.00: return 0
    if v < 3.70:  # 3.00..3.70 -> 0..50
        return int((v - 3.00) * (50.0 / 0.70))
    else:         # 3.70..4.20 -> 50..100
        return int(50 + (v - 3.70) * (50.0 / 0.50))


def adv_payload(name=None):
    p = bytearray()
    p += bytes((2, 0x01, 0x06))  # Flags: LE General Discoverable + BR/EDR not supported
    if name:
        n = name.encode()
        p += bytes((len(n) + 1, 0x09)) + n  # Complete Local Name
    return bytes(p)

# NUS UUID little-endian bytes for AD type 0x07 (Complete List of 128-bit UUIDs)
NUS_UUID_LE = bytes((
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
))
adv  = adv_payload(NAME)
resp = bytes((len(NUS_UUID_LE) + 1, 0x07)) + NUS_UUID_LE

ble.irq(on_ble)
ble.gap_advertise(100_000, adv_data=adv, resp_data=resp)
print("Advertising as", NAME)

# ---------- IMU (UART-RVC) ----------
imu = UART(IMU_UID, BAUD, rx=Pin(IMU_RX), tx=Pin(IMU_TX), timeout=0, timeout_char=0)
rvc_buf = bytearray()

def _parse_rvc_latest():
    """Return latest (yaw, pitch, roll) in deg from RVC stream, or None."""
    global rvc_buf, imu_frames   # <-- add imu_frames here
    out = None
    while True:
        i = rvc_buf.find(b'\xAA\xAA')
        if i < 0:
            rvc_buf = rvc_buf[-1:]  # keep last byte for header split
            break
        if len(rvc_buf) - i < 19:
            if i > 0:
                rvc_buf = rvc_buf[i:]
            break
        frame = rvc_buf[i+2:i+19]
        rvc_buf = rvc_buf[i+19:]
        if (sum(frame[0:16]) & 0xFF) != frame[16]:
            continue
        y_i, p_i, r_i, _, _, _ = struct.unpack('<hhhhhh', frame[1:13])
        out = (y_i/100.0, p_i/100.0, r_i/100.0)
        imu_frames += 1           # <-- add this line
    return out


def _wrap180(x):
    if x <= -180 or x > 180:
        x = ((x + 180) % 360) - 180
    return x

# ---------- GPS (MicropyGPS) ----------
gps_uart = UART(
    GPS_UID,
    baudrate=GPS_BAUD,
    rx=Pin(GPS_RX_PIN),
    tx=Pin(GPS_TX_PIN),
    timeout=50,
    timeout_char=50
)
my_gps = MicropyGPS()

# Latest parsed GPS state (normalized/scalars where possible)
gps_fix     = False
gps_lat_deg = float('nan')
gps_lon_deg = float('nan')
gps_sats    = -1
gps_hdop    = -1.0
gps_alt_m   = float('nan')
gps_spd_kn  = float('nan')
gps_cog_deg = float('nan')
gps_last_ms = 0

def _micropy_to_deg(lat_tuple, lon_tuple):
    # MicropyGPS latitude/longitude are like (deg, minutes.decimal, 'N'/'S'), same for lon with 'E'/'W'
    def conv(tup):
        if not tup or len(tup) < 3:
            return float('nan')
        deg = float(tup[0]); minutes = float(tup[1]); hemi = tup[2]
        val = deg + minutes/60.0
        if hemi in ('S', 'W'):
            val = -val
        return val
    return conv(lat_tuple), conv(lon_tuple)

def gps_poll():
    """Feed MicropyGPS with whatever is in UART, then snapshot fields."""
    global gps_fix, gps_lat_deg, gps_lon_deg, gps_sats, gps_hdop, gps_alt_m, gps_spd_kn, gps_cog_deg

    n = gps_uart.any()
    if n and n > 0:
        data = gps_uart.read(n)
        if data:
            for b in data:
                # Feed only printable ASCII to parser
                if 10 <= b <= 126:
                    my_gps.update(chr(b))

    # Update snapshot after feeding
    gps_fix = (my_gps.fix_type or 0) > 0

    # lat/lon to signed decimal degrees
    lat_d, lon_d = _micropy_to_deg(my_gps.latitude, my_gps.longitude)
    gps_lat_deg = lat_d if lat_d == lat_d else float('nan')   # NaN check
    gps_lon_deg = lon_d if lon_d == lon_d else float('nan')

    # Satellites / HDOP
    try: gps_sats = int(my_gps.satellites_in_use or -1)
    except: gps_sats = -1
    try: gps_hdop = float(my_gps.hdop) if my_gps.hdop is not None else -1.0
    except: gps_hdop = -1.0

    # Altitude, speed (knots), course (deg)
    try: gps_alt_m = float(my_gps.altitude) if my_gps.altitude is not None else float('nan')
    except: gps_alt_m = float('nan')
    try: gps_spd_kn = float(my_gps.speed[0]) if isinstance(my_gps.speed, (list, tuple)) else float(my_gps.speed)
    except: 
        try: gps_spd_kn = float(my_gps.speed) if my_gps.speed is not None else float('nan')
        except: gps_spd_kn = float('nan')
    try: gps_cog_deg = float(my_gps.course) if my_gps.course is not None else float('nan')
    except: gps_cog_deg = float('nan')

pitch_f = None
last_tx = time.ticks_ms()
TX_DT   = max(1, int(1000/PRINT_HZ))

# ---------- Main loop ----------
while True:
    b = imu.read()
    if b: rvc_buf.extend(b)
    v = _parse_rvc_latest()
    if v:
        p_raw = _wrap180(v[1])  # raw pitch (pre-offset)
        _update_calibration(p_raw)
        p = _wrap180(p_raw - pitch_offset)  # apply zero
        if pitch_f is None:
            pitch_f = p
        else:
            pitch_f += EMA_A * (p - pitch_f)
    now = time.ticks_ms()
    if (conn_handle is not None) and stream_enabled and (pitch_f is not None) and (time.ticks_diff(now, last_tx) >= TX_DT):
        _notify(f"PITCH,{pitch_f:.2f}")
        ble_msgs += 1                 # <-- add this line
        last_tx = now

    # ---- GPS poll (non-blocking) ----
    gps_poll()

    # ---- Send a GPS line ~1 Hz ----
    if time.ticks_diff(now, gps_last_ms) >= 900:
        fx   = 1 if gps_fix else 0
        lat  = gps_lat_deg
        lon  = gps_lon_deg
        alt  = gps_alt_m
        sats = gps_sats
        hd   = gps_hdop
        spd  = gps_spd_kn
        cog  = gps_cog_deg
        # Guard NaNs for format
        def fnan(v, fmt):
            try:
                if v != v:  # NaN
                    return "nan"
                return fmt % v
            except:
                return "nan"
        _notify(
            "GPS,%d,%s,%s,%s,%d,%s,%s,%s" % (
                fx,
                fnan(lat, "%.6f"),
                fnan(lon, "%.6f"),
                fnan(alt, "%.1f"),
                (sats if isinstance(sats, int) else -1),
                fnan(hd, "%.2f"),
                fnan(spd, "%.2f"),
                fnan(cog, "%.1f"),
            )
        )
        gps_last_ms = now

    # --- battery every BATT_DT ---
    if time.ticks_diff(now, _batt_last) >= BATT_DT:
        vb = _vbat_tick()
        pc = _vbat_percent(vb)
        _notify(f"BATT,{vb:.3f},{pc}")
        _batt_last = now

    # --- quick meter every 2s ---
    if time.ticks_diff(now, rate_last) >= 2000:
        dt_ms = max(1, time.ticks_diff(now, rate_last))
        imu_hz = (imu_frames * 1000.0) / dt_ms
        ble_hz = (ble_msgs * 1000.0) / dt_ms
        fix_flag = "FIX" if gps_fix else "NOFIX"
        _notify(f"RATE IMU~{imu_hz:.1f}Hz, BLE~{ble_hz:.1f}Hz, PRINT_HZ={PRINT_HZ} {fix_flag}")
        imu_frames = 0
        ble_msgs   = 0
        rate_last  = now



    time.sleep_ms(1)









