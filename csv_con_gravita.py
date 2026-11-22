#!/usr/bin/env python3
"""CSV del sismografo -> CSV con tempo (come csv_con_tempo.py) + ODS depurato dalla gravita'.

Uso:
    python csv_con_gravita.py 000017.CSV
    python csv_con_gravita.py 000017.CSV --riferimento post
    python csv_con_gravita.py 000017.CSV --finestra 150 --soglia 100

Crea:
    NOME_tempo.csv        identico a quello di csv_con_tempo.py (valori grezzi, colonna del tempo)
    NOME_tempo_netto.ods  due fogli:
        "Dati"          tempo, x/y/z in LSB meno il valore medio a riposo, x/y/z in mg, modulo in mg
        "Orientamento"  media a riposo, rumore, modulo, versore verso l'alto, inclinazione, roll, pitch

Come si trova il "riposo"
    Il file viene diviso in finestre di --finestra campioni (default 100, circa 1 s). Una finestra e'
    a riposo se in ogni asse la differenza picco-picco e' <= --soglia LSB (default 120, circa 7 mg).
    Si cercano finestre a riposo
      - "pre":  nella parte precedente al trigger (primo blocco, senza gli ultimi 20 campioni);
      - "post": nel resto dell'evento (di solito gli ultimi secondi, prima della chiusura).
    --riferimento auto (default) usa "pre" se ce n'e', altrimenti "post"; "pre", "post" e "tutto"
    forzano la scelta. Se non c'e' nessuna finestra a riposo si usa quella con meno variazione.

Convenzioni
    A riposo l'accelerometro misura +1 g nel verso OPPOSTO alla gravita', cioe' verso l'alto: il
    "versore" e' quel vettore normalizzato nel sistema di assi della scheda (x, y, z).
    inclinazione = angolo fra l'asse Z della scheda e la verticale (0 gradi = scheda piatta, faccia
    in su; 180 gradi = capovolta); roll = atan2(uy, uz); pitch = atan2(-ux, sqrt(uy^2 + uz^2)).

ODS richiede:  pip install odfpy
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import csv_con_tempo as base  # noqa: E402  (stesso cartella: parsing, tempi, scrittura CSV)

MARGINE_PRE = 20  # campioni finali del primo blocco esclusi (possibile inizio del segnale)


# ----------------------------------------------------------------------------
# Ricerca del riposo
# ----------------------------------------------------------------------------
def finestre_quiete(campioni, inizio, fine, w, soglia):
    """Finestre non sovrapposte di w campioni con picco-picco <= soglia su tutti gli assi."""
    trovate = []
    s = inizio
    while s + w <= fine:
        blocco = campioni[s:s + w]
        p2p = max(max(c[a] for c in blocco) - min(c[a] for c in blocco) for a in range(3))
        if p2p <= soglia:
            trovate.append((s, s + w))
            s += w
        else:
            s += max(1, w // 2)
    return trovate


def finestra_piu_calma(campioni, w):
    """Fallback: la finestra con la minore somma dei picco-picco."""
    migliore, punteggio = None, None
    for s in range(0, max(1, len(campioni) - w + 1), max(1, w // 2)):
        blocco = campioni[s:s + w]
        p = sum(max(c[a] for c in blocco) - min(c[a] for c in blocco) for a in range(3))
        if punteggio is None or p < punteggio:
            migliore, punteggio = (s, min(s + w, len(campioni))), p
    return [migliore]


def media_std(campioni, finestre):
    dati = [c for (a, b) in finestre for c in campioni[a:b]]
    n = len(dati)
    media = [sum(c[a] for c in dati) / n for a in range(3)]
    std = [math.sqrt(sum((c[a] - media[a]) ** 2 for c in dati) / n) for a in range(3)]
    return media, std, n


def orientamento(media_lsb):
    mg = [v * base.MG_PER_LSB for v in media_lsb]
    modulo = math.sqrt(sum(v * v for v in mg))
    u = [v / modulo for v in mg]
    inclinazione = math.degrees(math.acos(max(-1.0, min(1.0, u[2]))))
    roll = math.degrees(math.atan2(u[1], u[2]))
    pitch = math.degrees(math.atan2(-u[0], math.hypot(u[1], u[2])))
    return {"mg": mg, "modulo": modulo, "u": u, "incl": inclinazione, "roll": roll, "pitch": pitch}


def angolo_fra(u1, u2):
    d = sum(a * b for a, b in zip(u1, u2))
    return math.degrees(math.acos(max(-1.0, min(1.0, d))))


def analizza_riposo(campioni, n_pre, w, soglia, riferimento):
    pre = finestre_quiete(campioni, 0, max(0, n_pre - MARGINE_PRE), w, soglia)
    post = finestre_quiete(campioni, n_pre, len(campioni), w, soglia)

    if riferimento == "pre":
        scelte, nome = pre, "pre"
    elif riferimento == "post":
        scelte, nome = post, "post"
    elif riferimento == "tutto":
        scelte, nome = pre + post, "tutto"
    else:  # auto
        scelte, nome = (pre, "pre") if pre else (post, "post")

    avviso = None
    if not scelte:
        scelte, nome = finestra_piu_calma(campioni, w), "fallback"
        avviso = "nessuna finestra a riposo: usata la piu' calma (media poco affidabile)"

    media, std, n = media_std(campioni, scelte)
    ris = {"nome": nome, "media": media, "std": std, "n": n, "finestre": len(scelte),
           "avviso": avviso, "orient": orientamento(media)}
    if pre and post:
        mp, _, _ = media_std(campioni, pre)
        mq, _, _ = media_std(campioni, post)
        ris["scarto_pre_post"] = angolo_fra(orientamento(mp)["u"], orientamento(mq)["u"])
    return ris


# ----------------------------------------------------------------------------
# ODS
# ----------------------------------------------------------------------------
def scrivi_ods_netto(percorso, intestazione, righe_dati, righe_orient, decimali):
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
    formato = TimeStyle(name="NTempo")
    formato.addElement(Hours(style="long"))
    formato.addElement(NumText(text=":"))
    formato.addElement(Minutes(style="long"))
    formato.addElement(NumText(text=":"))
    formato.addElement(Seconds(style="long", decimalplaces=decimali))
    doc.automaticstyles.addElement(formato)
    doc.automaticstyles.addElement(Style(name="CTempo", family="table-cell", datastylename="NTempo"))

    def cella(v, tempo=False):
        if tempo:
            hh, mm, resto = v.split(":")
            tc = TableCell(valuetype="time", stylename="CTempo",
                           timevalue=f"PT{int(hh):02d}H{int(mm):02d}M{resto}S")
            tc.addElement(P(text=v))
        elif isinstance(v, (int, float)):
            tc = TableCell(valuetype="float", value=v)
            tc.addElement(P(text=str(v)))
        else:
            tc = TableCell(valuetype="string")
            tc.addElement(P(text="" if v is None else str(v)))
        return tc

    def foglio(nome, intest, righe, prima_colonna_tempo):
        t = Table(name=nome)
        if intest:
            tr = TableRow()
            for v in intest:
                tr.addElement(cella(v))
            t.addElement(tr)
        for r in righe:
            tr = TableRow()
            for i, v in enumerate(r):
                tr.addElement(cella(v, tempo=(prima_colonna_tempo and i == 0)))
            t.addElement(tr)
        doc.spreadsheet.addElement(t)

    foglio("Dati", intestazione, righe_dati, True)
    foglio("Orientamento", ["Grandezza", "Valore", "Unita'"], righe_orient, False)
    doc.save(percorso)
    return True


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="CSV del sismografo -> CSV con tempo + ODS depurato dalla gravita'.")
    ap.add_argument("file", help="file CSV scritto dalla SensorTile.box")
    ap.add_argument("--riferimento", choices=["auto", "pre", "post", "tutto"], default="auto",
                    help="da dove prendere il riposo (default auto)")
    ap.add_argument("--finestra", type=int, default=100, help="campioni per finestra (default 100)")
    ap.add_argument("--soglia", type=int, default=120, help="picco-picco massimo a riposo, in LSB (default 120)")
    ap.add_argument("--decimali", type=int, default=3, help="decimali dei secondi (default 3)")
    ap.add_argument("--local", action="store_true", help="ora locale del PC invece di UTC")
    ap.add_argument("--odr", type=float, default=104.0, help="frequenza nominale se c'e' un solo blocco")
    ap.add_argument("--sep", default=",", help="separatore del CSV in uscita (default ',')")
    args = ap.parse_args()

    if not os.path.isfile(args.file):
        sys.exit(f"File non trovato: {args.file}")
    eventi = base.leggi_eventi(args.file)
    if not eventi:
        sys.exit("Nessun dato trovato nel file.")

    mg = base.MG_PER_LSB
    colonna_tempo = "tempo_" + ("locale" if args.local else "UTC")
    intest_csv = [colonna_tempo, "x", "y", "z"]
    intest_ods = [colonna_tempo, "x_netto_LSB", "y_netto_LSB", "z_netto_LSB",
                  "x_netto_mg", "y_netto_mg", "z_netto_mg", "modulo_netto_mg"]

    righe_csv, righe_dati, righe_orient = [], [], []
    for numero, blocchi in enumerate(eventi, 1):
        tempi, odr_eff = base.tempi_evento(blocchi, args.odr)
        campioni = [r for b in blocchi for r in b["righe"]]
        n_pre = len(blocchi[0]["righe"])
        riposo = analizza_riposo(campioni, n_pre, args.finestra, args.soglia, args.riferimento)
        m = riposo["media"]
        o = riposo["orient"]

        print(f"Evento {numero}: {len(campioni)} campioni, frequenza effettiva {odr_eff:.2f} Hz")
        print(f"  riposo ({riposo['nome']}): {riposo['n']} campioni in {riposo['finestre']} finestre; "
              f"media LSB = {m[0]:.1f}, {m[1]:.1f}, {m[2]:.1f}")
        print(f"  |g| = {o['modulo']:.1f} mg; inclinazione = {o['incl']:.2f} gradi, "
              f"roll = {o['roll']:.2f}, pitch = {o['pitch']:.2f}")
        if riposo["avviso"]:
            print("  ATTENZIONE: " + riposo["avviso"])
        if "scarto_pre_post" in riposo:
            print(f"  scarto di orientamento fra riposo pre e post: {riposo['scarto_pre_post']:.2f} gradi")

        for t, (x, y, z) in zip(tempi, campioni):
            s = base.formatta_tempo(t, args.decimali, args.local)
            righe_csv.append([s, x, y, z])
            nx, ny, nz = x - m[0], y - m[1], z - m[2]
            mx, my, mz = nx * mg, ny * mg, nz * mg
            righe_dati.append([s, round(nx, 2), round(ny, 2), round(nz, 2),
                               round(mx, 3), round(my, 3), round(mz, 3),
                               round(math.sqrt(mx * mx + my * my + mz * mz), 3)])

        # foglio Orientamento
        if numero > 1:
            righe_orient.append(["", "", ""])
        righe_orient += [
            [f"EVENTO {numero}", "", ""],
            ["Riferimento a riposo", riposo["nome"], ""],
            ["Campioni a riposo", riposo["n"], "campioni"],
            ["Finestre a riposo", riposo["finestre"], ""],
            ["Media a riposo x", round(m[0], 2), "LSB"],
            ["Media a riposo y", round(m[1], 2), "LSB"],
            ["Media a riposo z", round(m[2], 2), "LSB"],
            ["Media a riposo x", round(o["mg"][0], 3), "mg"],
            ["Media a riposo y", round(o["mg"][1], 3), "mg"],
            ["Media a riposo z", round(o["mg"][2], 3), "mg"],
            ["Rumore (dev. standard) x", round(riposo["std"][0] * mg, 3), "mg"],
            ["Rumore (dev. standard) y", round(riposo["std"][1] * mg, 3), "mg"],
            ["Rumore (dev. standard) z", round(riposo["std"][2] * mg, 3), "mg"],
            ["Modulo del vettore a riposo", round(o["modulo"], 3), "mg"],
            ["Versore verso l'alto x", round(o["u"][0], 5), ""],
            ["Versore verso l'alto y", round(o["u"][1], 5), ""],
            ["Versore verso l'alto z", round(o["u"][2], 5), ""],
            ["Inclinazione asse Z dalla verticale", round(o["incl"], 3), "gradi"],
            ["Roll", round(o["roll"], 3), "gradi"],
            ["Pitch", round(o["pitch"], 3), "gradi"],
        ]
        if "scarto_pre_post" in riposo:
            righe_orient.append(["Scarto orientamento riposo pre/post", round(riposo["scarto_pre_post"], 3), "gradi"])
        if riposo["avviso"]:
            righe_orient.append(["ATTENZIONE", riposo["avviso"], ""])

    righe_orient += [
        ["", "", ""],
        ["Nota", "A riposo si misura +1 g verso l'alto, nel sistema di assi della scheda", ""],
    ]

    radice = os.path.splitext(args.file)[0] + "_tempo"
    base.scrivi_csv(radice + ".csv", intest_csv, righe_csv, args.sep)
    print(f"  creato {radice}.csv")
    if scrivi_ods_netto(radice + "_netto.ods", intest_ods, righe_dati, righe_orient, args.decimali):
        print(f"  creato {radice}_netto.ods")


if __name__ == "__main__":
    main()
