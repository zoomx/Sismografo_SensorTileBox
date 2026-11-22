#!/usr/bin/env python3
"""Aggiunge una colonna con il tempo (hh:mm:ss.fff, UTC) ai CSV del sismografo SensorTile.box.

Uso:
    python csv_con_tempo.py 000017.CSV                # crea 000017_tempo.csv
    python csv_con_tempo.py 000017.CSV --ods          # crea anche 000017_tempo.ods
    python csv_con_tempo.py 000017.CSV --mg           # aggiunge colonne x,y,z in mg
    python csv_con_tempo.py 000017.CSV --decimali 5   # piu' decimali nei secondi
    python csv_con_tempo.py 000017.CSV --local        # ora locale del PC invece di UTC
    python csv_con_tempo.py 000017.CSV --sep ";"      # separatore per Excel italiano

Come si calcola il tempo
    Ogni riga '# t=<epoch>.<ms> n=<N>' indica quando e' stato letto un blocco di N campioni.
    Per non fidarsi della frequenza nominale (104 Hz), il tempo dei campioni si ricava da una
    retta ai minimi quadrati fra "numero cumulativo di campioni" e "istante di lettura dei
    blocchi": la pendenza e' il passo di campionamento effettivo. Se l'evento ha un solo
    blocco si usa la frequenza nominale (--odr).

ODS richiede:  pip install odfpy
"""
import argparse
import os
import re
import sys
from datetime import datetime, timezone

RE_BLOCCO = re.compile(r"#\s*t=(\d+)\.(\d+)\s+n=(\d+)(\s+OVERRUN)?")
RE_RIGA = re.compile(r"^\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*$")
MG_PER_LSB = 0.061  # fondo scala +-2 g


def leggi_eventi(percorso):
    """Ritorna una lista di eventi; ogni evento e' una lista di blocchi (t, righe, overrun)."""
    with open(percorso, "rb") as f:
        testo = f.read().replace(b"\x00", b"").decode("latin1")

    eventi = []
    corrente = None
    blocco = None
    for riga in testo.splitlines():
        if "SensorTile.box seismograph" in riga:
            corrente = []
            eventi.append(corrente)
            blocco = None
            continue
        m = RE_BLOCCO.search(riga)
        if m:
            if corrente is None:
                corrente = []
                eventi.append(corrente)
            ms = m.group(2)
            t = int(m.group(1)) + float("0." + ms)
            blocco = {"t": t, "n": int(m.group(3)), "righe": [], "overrun": bool(m.group(4))}
            corrente.append(blocco)
            continue
        if riga.lstrip().startswith("#"):
            continue
        r = RE_RIGA.match(riga)
        if r and blocco is not None:
            blocco["righe"].append(tuple(int(v) for v in r.groups()))
    return [[b for b in ev if b["righe"]] for ev in eventi if any(b["righe"] for b in ev)]


def retta_minimi_quadrati(x, y):
    n = len(x)
    mx = sum(x) / n
    my = sum(y) / n
    sxx = sum((xi - mx) ** 2 for xi in x)
    if sxx == 0:
        return None
    b = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y)) / sxx
    return my - b * mx, b


def tempi_evento(blocchi, odr_nominale):
    """Calcola il tempo (epoch UTC, float) di ogni campione dell'evento."""
    cum, fine = 0, []
    for b in blocchi:
        cum += len(b["righe"])
        fine.append(cum)
    istanti = [b["t"] for b in blocchi]

    a = bpass = None
    if len(blocchi) >= 2:
        ris = retta_minimi_quadrati(fine, istanti)
        if ris and ris[1] > 0:
            a, bpass = ris
    if a is None:
        bpass = 1.0 / odr_nominale
        a = istanti[-1] - bpass * fine[-1]

    tempi = [a + bpass * (k + 1) for k in range(cum)]
    return tempi, 1.0 / bpass


def formatta_tempo(t, decimali, locale):
    unita = 10 ** decimali
    totale = int(round(t * unita))
    secondi, frazione = divmod(totale, unita)
    if locale:
        dt = datetime.fromtimestamp(secondi).astimezone()
        h, m, s = dt.hour, dt.minute, dt.second
    else:
        sod = secondi % 86400
        h, m, s = sod // 3600, (sod % 3600) // 60, sod % 60
    return f"{h:02d}:{m:02d}:{s:02d}.{frazione:0{decimali}d}"


def scrivi_csv(percorso, intestazione, righe, sep):
    with open(percorso, "w", encoding="utf-8", newline="") as f:
        f.write(sep.join(intestazione) + "\n")
        for r in righe:
            f.write(sep.join(str(v) for v in r) + "\n")


def scrivi_ods(percorso, intestazione, righe, decimali):
    try:
        from odf.number import Hours, Minutes, Seconds, TimeStyle
        from odf.number import Text as NumText
        from odf.opendocument import OpenDocumentSpreadsheet
        from odf.style import Style
        from odf.table import Table, TableCell, TableRow
        from odf.text import P
    except ImportError:
        print("  ODS non creato: installa la libreria con  pip install odfpy")
        return False

    doc = OpenDocumentSpreadsheet()

    # Formato numerico "hh:mm:ss.000" (tanti zeri quanti sono i decimali richiesti)
    formato = TimeStyle(name="NTempo")
    formato.addElement(Hours(style="long"))
    formato.addElement(NumText(text=":"))
    formato.addElement(Minutes(style="long"))
    formato.addElement(NumText(text=":"))
    formato.addElement(Seconds(style="long", decimalplaces=decimali))
    doc.automaticstyles.addElement(formato)
    stile_tempo = Style(name="CTempo", family="table-cell", datastylename="NTempo")
    doc.automaticstyles.addElement(stile_tempo)

    tabella = Table(name="Dati")
    doc.spreadsheet.addElement(tabella)

    def riga_ods(valori, con_tempo):
        tr = TableRow()
        for i, v in enumerate(valori):
            if con_tempo and i == 0:
                hh, mm, resto = v.split(":")
                tc = TableCell(valuetype="time", stylename="CTempo",
                               timevalue=f"PT{int(hh):02d}H{int(mm):02d}M{resto}S")
                tc.addElement(P(text=v))
            elif isinstance(v, (int, float)):
                tc = TableCell(valuetype="float", value=v)
                tc.addElement(P(text=str(v)))
            else:
                tc = TableCell(valuetype="string")
                tc.addElement(P(text=str(v)))
            tr.addElement(tc)
        tabella.addElement(tr)

    riga_ods(intestazione, False)
    for r in righe:
        riga_ods(r, True)
    doc.save(percorso)
    return True


def main():
    ap = argparse.ArgumentParser(description="Aggiunge la colonna del tempo ai CSV del sismografo.")
    ap.add_argument("file", help="file CSV scritto dalla SensorTile.box")
    ap.add_argument("--ods", action="store_true", help="crea anche un file .ods")
    ap.add_argument("--mg", action="store_true", help="aggiunge le colonne x,y,z in mg")
    ap.add_argument("--decimali", type=int, default=3, help="decimali dei secondi (default 3)")
    ap.add_argument("--local", action="store_true", help="ora locale del PC invece di UTC")
    ap.add_argument("--odr", type=float, default=104.0, help="frequenza nominale se c'e' un solo blocco")
    ap.add_argument("--sep", default=",", help="separatore del CSV in uscita (default ',')")
    args = ap.parse_args()

    if not os.path.isfile(args.file):
        sys.exit(f"File non trovato: {args.file}")

    eventi = leggi_eventi(args.file)
    if not eventi:
        sys.exit("Nessun dato trovato nel file.")

    intestazione = ["tempo_" + ("locale" if args.local else "UTC"), "x", "y", "z"]
    if args.mg:
        intestazione += ["x_mg", "y_mg", "z_mg"]

    righe = []
    for numero, blocchi in enumerate(eventi, 1):
        tempi, odr_eff = tempi_evento(blocchi, args.odr)
        campioni = [r for b in blocchi for r in b["righe"]]
        overrun = sum(1 for b in blocchi if b["overrun"])
        inizio = datetime.fromtimestamp(tempi[0], tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
        print(f"Evento {numero}: {len(campioni)} campioni, {len(blocchi)} blocchi, "
              f"inizio {inizio} UTC, frequenza effettiva stimata {odr_eff:.2f} Hz"
              + (f", blocchi con OVERRUN: {overrun}" if overrun else ""))
        for t, (x, y, z) in zip(tempi, campioni):
            riga = [formatta_tempo(t, args.decimali, args.local), x, y, z]
            if args.mg:
                riga += [round(x * MG_PER_LSB, 2), round(y * MG_PER_LSB, 2), round(z * MG_PER_LSB, 2)]
            righe.append(riga)

    base = os.path.splitext(args.file)[0] + "_tempo"
    scrivi_csv(base + ".csv", intestazione, righe, args.sep)
    print(f"  creato {base}.csv")
    if args.ods and scrivi_ods(base + ".ods", intestazione, righe, args.decimali):
        print(f"  creato {base}.ods")


if __name__ == "__main__":
    main()
