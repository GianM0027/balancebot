# Staffa per motoriduttore JGA25-370 (con encoder) - script per Autodesk Fusion
#
# Riferimenti (asse motore = Z):
#   z = 0       faccia lato RUOTA della piastra (va sul piatto di stampa)
#   z = PLATE_T faccia del riduttore
#   y < 0       base che si avvita al telaio; y > 0 orecchie del morsetto
#
# Parti:
#  - piastra frontale da 2.5 mm (quanto il collarino Ø7 dell'albero):
#    foro per il collarino e 2 fori passanti da 5.8 mm a 17 mm di interasse
#    per viti M3x6 (o x8) a testa cilindrica: la testa si stringe contro la
#    faccia del riduttore e fa da perno contro la rotazione (lungo l'asse
#    tiene il morsetto). La testa (3 mm) sporge 0.5 mm verso la ruota.
#    Provato con i provini del 11/11 (versione a 2 tacche).
#  - culla attorno al riduttore Ø25, tagliata in alto e chiusa da 2 viti M3
#  - base lunga quanto riduttore + motore (l'encoder resta libero), a sella
#    sotto il corpo motore, con 4 fori M3 e DADI PRIGIONIERI: il dado entra
#    da una fessura laterale e la vite arriva da sotto il telaio
#
# Stampa: piastra sul piatto, senza supporti. La stessa staffa va bene per
# entrambi i lati del robot (e simmetrica).
# Le misure sono costanti qui sotto: cambiale e riesegui lo script.

import adsk.core
import adsk.fusion
import traceback

# ---------------- MISURE (mm) ----------------
GEARBOX_DIA = 25.0      # diametro riduttore
GEARBOX_LEN = 19.0      # lunghezza riduttore (282 RPM, rapporto 21.3) - misurala!
MOTOR_LEN = 31.0        # corpo motore, escluso l'encoder - misuralo!
BORE_CLEAR = 0.3        # gioco tra culla e riduttore
PLATE_T = 2.5           # spessore piastra = lunghezza collarino
BOSS_HOLE = 7.4         # foro per il collarino Ø7
FACE_HOLE_SPACING = 17.0
FACE_HOLE_DIA = 5.8     # foro passante per la testa M3 cilindrica (5.5 + gioco)
WALL = 3.0              # spessore parete culla
CRADLE_SHORT = 1.0      # la culla finisce 1 mm prima della fine del riduttore
BASE_SHORT = 1.0        # la base finisce 1 mm prima dell'encoder
SLIT = 2.0              # taglio del morsetto
EAR_T = 5.0             # spessore di ogni orecchia
EAR_TOP = 8.0           # quanto sporgono le orecchie oltre la culla
CLAMP_HOLE = 3.4        # foro vite M3 del morsetto
BASE_W = 48.0           # larghezza base
FLOOR_BELOW = 4.0       # materiale tra la culla e la faccia d'appoggio
# Fissaggio al telaio
BOLT_HOLE = 3.4         # foro passante M3
BOLT_X = 19.0           # fori a x = +-19
BOLT_Z_MARGIN = 6.0     # distanza dei fori dalle estremita della base
NUT_AF = 5.7            # larghezza fessura dado M3 (chiave 5.5 + gioco)
NUT_T = 2.7             # spessore fessura dado (2.4 + gioco)
NUT_FLOOR = 4.0         # materiale tra il dado e la faccia d'appoggio
BOLT_ABOVE_NUT = 3.0    # quanto prosegue il foro oltre il dado


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent
        extrudes = root.features.extrudeFeatures
        OP = adsk.fusion.FeatureOperations
        warnings = []

        cm = lambda mm: mm / 10.0
        VR = lambda mm: adsk.core.ValueInput.createByReal(cm(mm))

        def P(x, y, z=0.0):
            return adsk.core.Point3D.create(cm(x), cm(y), cm(z))

        def plane_z(z_mm, name):
            pin = root.constructionPlanes.createInput()
            pin.setByOffset(root.xYConstructionPlane, VR(z_mm))
            pl = root.constructionPlanes.add(pin)
            pl.name = name
            return pl

        def plane_y(y_mm, name):
            """Piano parallelo a XZ alla quota y (gestisce il verso della normale)."""
            for sign in (1, -1):
                pin = root.constructionPlanes.createInput()
                pin.setByOffset(root.xZConstructionPlane, VR(sign * y_mm))
                pl = root.constructionPlanes.add(pin)
                if abs(pl.geometry.origin.y - cm(y_mm)) < 1e-4:
                    pl.name = name
                    return pl
                pl.deleteMe()
            raise RuntimeError("piano y non creato")

        def all_profiles(sk):
            coll = adsk.core.ObjectCollection.create()
            for p in sk.profiles:
                coll.add(p)
            return coll

        r_bore = (GEARBOX_DIA + BORE_CLEAR) / 2
        r_out = r_bore + WALL
        cradle_len = PLATE_T + GEARBOX_LEN - CRADLE_SHORT
        base_len = PLATE_T + GEARBOX_LEN + MOTOR_LEN - BASE_SHORT
        y_floor = -(r_bore + FLOOR_BELOW)
        y_ear = r_out + EAR_TOP
        x_ear = SLIT / 2 + EAR_T
        bolt_z = (BOLT_Z_MARGIN, base_len - BOLT_Z_MARGIN)

        # --- 1. Volume pieno: anello della culla + base lunga + orecchie ---
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Outer"
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cm(r_out))
        body_ext = extrudes.addSimple(sk.profiles.item(0), VR(cradle_len),
                                      OP.NewBodyFeatureOperation)
        body_ext.bodies.item(0).name = "MotorMount"

        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Base"
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P(-BASE_W / 2, y_floor), P(BASE_W / 2, 0))
        extrudes.addSimple(sk.profiles.item(0), VR(base_len), OP.JoinFeatureOperation)

        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Ears"
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P(-x_ear, r_bore - 1), P(x_ear, y_ear))
        extrudes.addSimple(sk.profiles.item(0), VR(cradle_len), OP.JoinFeatureOperation)


        # --- 2. Culla per riduttore e sella per il motore (sopra la piastra) ---
        sk = root.sketches.add(plane_z(PLATE_T, "GearboxFace"))
        sk.name = "Bore"
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cm(r_bore))
        extrudes.addSimple(sk.profiles.item(0), VR(base_len), OP.CutFeatureOperation)

        # --- 3. Foro collarino + fori viti frontali ---
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "FaceHoles"
        circles = sk.sketchCurves.sketchCircles
        circles.addByCenterRadius(P(0, 0), cm(BOSS_HOLE / 2))
        for sx in (-1, 1):
            circles.addByCenterRadius(P(sx * FACE_HOLE_SPACING / 2, 0), cm(FACE_HOLE_DIA / 2))
        extrudes.addSimple(all_profiles(sk), VR(PLATE_T + 1), OP.CutFeatureOperation)

        # --- 4. Taglio del morsetto ---
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Slit"
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P(-SLIT / 2, 0), P(SLIT / 2, y_ear + 1))
        extrudes.addSimple(sk.profiles.item(0), VR(cradle_len), OP.CutFeatureOperation)

        # --- 5. Fori delle viti del morsetto (lungo X, attraverso le orecchie) ---
        sk = root.sketches.add(root.yZConstructionPlane)
        sk.name = "ClampHoles"
        y_bolt = (r_out + y_ear) / 2
        for z in (cradle_len / 3, 2 * cradle_len / 3):
            c = sk.modelToSketchSpace(P(0, y_bolt, z))
            sk.sketchCurves.sketchCircles.addByCenterRadius(c, cm(CLAMP_HOLE / 2))
        inp = extrudes.createInput(all_profiles(sk), OP.CutFeatureOperation)
        inp.setSymmetricExtent(VR(2 * x_ear + 2), True)
        extrudes.add(inp).name = "ClampHoles"

        # --- 6. Fori per il telaio: dalla faccia d'appoggio verso l'alto ---
        bolt_depth = NUT_FLOOR + NUT_T + BOLT_ABOVE_NUT
        sk = root.sketches.add(plane_y(y_floor, "Floor"))
        sk.name = "BaseBoltHoles"
        for sx in (-1, 1):
            for z in bolt_z:
                c = sk.modelToSketchSpace(P(sx * BOLT_X, y_floor, z))
                sk.sketchCurves.sketchCircles.addByCenterRadius(c, cm(BOLT_HOLE / 2))
        inp = extrudes.createInput(all_profiles(sk), OP.CutFeatureOperation)
        inp.setSymmetricExtent(VR(2 * bolt_depth), True)  # meta finisce nel vuoto
        extrudes.add(inp).name = "BaseBoltHoles"

        # --- 7. Fessure laterali per i dadi M3 prigionieri ---
        y_nut = y_floor + NUT_FLOOR + NUT_T / 2
        sk = root.sketches.add(plane_y(y_nut, "NutSlots"))
        sk.name = "NutSlots"
        x_in = BOLT_X - NUT_AF * 0.58 - 0.2  # oltre la mezza diagonale del dado
        x_out = BASE_W / 2 + 1
        for sx in (-1, 1):
            for z in bolt_z:
                a = sk.modelToSketchSpace(P(sx * x_in, y_nut, z - NUT_AF / 2))
                b = sk.modelToSketchSpace(P(sx * x_out, y_nut, z + NUT_AF / 2))
                sk.sketchCurves.sketchLines.addTwoPointRectangle(a, b)
        inp = extrudes.createInput(all_profiles(sk), OP.CutFeatureOperation)
        inp.setSymmetricExtent(VR(NUT_T), True)
        extrudes.add(inp).name = "NutSlots"

        app.activeViewport.fit()
        msg = ("Staffa creata.\n\n"
               "Asse motore a {:.2f} mm sopra la faccia d'appoggio.\n"
               "Impronta base: {:.0f} x {:.1f} mm.\n"
               "Fori per il telaio: x = +-{:.0f} mm (interasse {:.0f}), "
               "a {:.1f} e {:.1f} mm dalla faccia lato ruota (interasse {:.1f}).\n\n"
               "Viti: 2 x M3x6 testa cilindrica (motore), 2 x M3x16 + dado (morsetto),\n"
               "4 x M3 + dado (telaio): lunghezza = spessore telaio + {:.0f} mm circa."
               .format(-y_floor, BASE_W, base_len, BOLT_X, 2 * BOLT_X,
                       bolt_z[0], bolt_z[1], bolt_z[1] - bolt_z[0],
                       NUT_FLOOR + 2.4 + 1.5))
        if warnings:
            msg += "\n\nAttenzione:\n- " + "\n- ".join(warnings)
        ui.messageBox(msg)

    except Exception:
        if ui:
            ui.messageBox("Errore:\n{}".format(traceback.format_exc()))
