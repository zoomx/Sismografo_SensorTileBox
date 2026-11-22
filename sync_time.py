#!/usr/bin/env python3
"""Sincronizza l'RTC della SensorTile.box (UTC) durante la finestra di avvio.

Uso:  python sync_time.py COM5        (Windows)
      python sync_time.py /dev/ttyACM0 (Linux/macOS)

Procedura: collega la USB, premi il pulsante USER (o riaccendi la scheda) e lancia
subito lo script: la finestra resta aperta circa 15 s (il LED blu lampeggia).
Altri comandi utili sulla seriale: "SD" (autotest scheda SD) e "?" (stato RTC).
"""
import sys
import time

import serial  # pip install pyserial


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    with serial.Serial(sys.argv[1], 115200, timeout=1) as s:
        epoch = int(time.time())  # epoch UTC, indipendente dal fuso orario
        s.write(f"T{epoch}\n".encode())
        t0 = time.time()
        while time.time() - t0 < 5:
            line = s.readline().decode(errors="replace").strip()
            if line:
                print(line)
                if line.startswith(("OK", "ERR")):
                    return
        print("Nessuna risposta: la finestra di avvio e' gia' chiusa? Premi USER e riprova.")


if __name__ == "__main__":
    main()
