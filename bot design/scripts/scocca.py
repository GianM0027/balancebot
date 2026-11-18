# Scocca del robot self-balancing - script per Autodesk Fusion (API Python)
# Bozza 2
#
# Importa in un nuovo design i pezzi della scocca gia costruiti e verificati
# (file STEP nella cartella "bot design\modelli"):
#   SCHIENA, COVER, VISIERA, PARATIA, FERMO (batteria), RIPIANO, PAVIMENTO, VASSOIO, SLITTA (modello per i moduli)
# con ruote e ingombri dei componenti, montati oppure esplosi.
#
# I pezzi arrivano come solidi senza cronologia: sono leggeri, si spostano e si modificano
# con gli strumenti diretti (Premi/Trascina, Sposta, Combina). Ogni file diventa un componente.
# La geometria viene generata e controllata fuori da Fusion (collisioni, spessori, ingombri):
# per cambiare una quota chiedi a Claude di rigenerare i file STEP.

import adsk.core
import adsk.fusion
import traceback
import os

CARTELLA = r"C:\Users\Utente\Desktop\myProjects\self_balancing_robot\bot design\modelli"
ESPLOSO = True           # True: pezzi distanziati per guardare dentro; False: robot montato
CON_COMPONENTI = True    # False: solo i pezzi da stampare, senza ruote e ingombri

PEZZI = ["schiena.step", "cover.step", "visiera.step", "paratia.step", "fermo.step", "ripiano.step", "tappo.step", "pavimento.step", "vassoio.step", "slitta.step"]


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        root = design.rootComponent
        im = app.importManager

        if not CON_COMPONENTI:
            files = PEZZI
        elif ESPLOSO:
            files = ["robot_esploso.step"]
        else:
            files = ["robot_assemblato.step"]

        mancanti = [f for f in files if not os.path.isfile(os.path.join(CARTELLA, f))]
        if mancanti:
            ui.messageBox("File non trovati in\n{}:\n{}".format(CARTELLA, "\n".join(mancanti)))
            return
        for f in files:
            opts = im.createSTEPImportOptions(os.path.join(CARTELLA, f))
            im.importToTarget(opts, root)
        app.activeViewport.fit()
        ui.messageBox("Importati: {}.\n\nPer vedere dentro un pezzo senza spostarlo: "
                      "Ispeziona > Analisi di sezione.".format(", ".join(files)))
    except Exception:
        if ui:
            ui.messageBox("Errore:\n{}".format(traceback.format_exc()))
