from __future__ import annotations

import struct
import time
from typing import Optional, Tuple


def read_exact(ser, size: int, attempts: int = 10, delay: float = 0.005) -> bytes:
    buffer = bytearray()
    for _ in range(attempts):
        chunk = ser.read(size - len(buffer))
        if chunk:
            buffer.extend(chunk)
            if len(buffer) >= size:
                return bytes(buffer)
        else:
            time.sleep(delay)
    raise TimeoutError(f"Lecture série incomplète ({len(buffer)}/{size} octets)")


def read_infrared_measurement(arduino, attempts: int = 3) -> Tuple[Optional[int], Optional[int]]:
    expected_payload = 4 + 2 + 2  # millis (int32) + valeur (int16) + padding (int16)
    for _ in range(attempts):
        try:
            arduino.reset_input_buffer()
            arduino.write(b'R')
            arduino.flush()
            payload = read_exact(arduino, expected_payload, attempts=20, delay=0.01)
            timestamp_ms = struct.unpack('<l', payload[0:4])[0]
            raw_value = struct.unpack('<h', payload[4:6])[0]
            return timestamp_ms, raw_value
        except TimeoutError:
            continue
    return None, None


def infrared_raw_to_distance_cm(raw_value: Optional[int]) -> Optional[float]:
    if raw_value is None or raw_value <= 20 or raw_value >= 1000:
        return None
    # Approximation based on datasheet: distance (cm) ≈ 4800 / (raw - 20)
    distance_cm = 4800.0 / (raw_value - 20.0)
    if distance_cm < 5 or distance_cm > 120:
        return None
    return distance_cm


def get_infrared_distance_cm(arduino, attempts: int = 3) -> Optional[float]:
    _, raw_value = read_infrared_measurement(arduino, attempts=attempts)
    return infrared_raw_to_distance_cm(raw_value)
