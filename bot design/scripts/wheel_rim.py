# Cerchione per ruota robotica in PLA - script per Autodesk Fusion (API Python)
#
# Pensato per la stampa 3D in piano (asse ruota verticale), SENZA supporti:
#  - cerchio Ø89.5 per O-ring 90 x 80 x 5 (Othmro, NBR 70A):
#    3 gole profonde 3.5 mm, fondo gola Ø82.5
#    -> O-ring tirato del ~3% (non slitta), ruota montata ~Ø92.4 (raggio ~46 mm)
#  - profilo delle gole pensato per la stampa (faccia esterna sul piatto):
#    pavimento piatto, 3.6 mm di parete verticale sul fondo, poi "soffitto" inclinato
#    a 45 gradi fino al bordo -> nessuna sporgenza orizzontale; apertura 7.1 mm
#  - bordi esterni e creste tra le gole da 2 mm; la larghezza della ruota
#    si ricava da gole + creste + bordi (29.3 mm)
#  - parete del cerchio da 4 mm sotto il fondo gola
#  - disco A FILO della faccia esterna (poggia sul piatto), a cono:
#    4 mm al cerchio, 8 mm al mozzo; raccordi verso cerchio e mozzo
#  - 5 razze curve: il disco e traforato da 5 finestre a goccia raccordate
#  - smussi da 0.5 mm sugli spigoli della faccia esterna (nascondono anche
#    il piede d'elefante) e sugli spigoli della faccia lato motore
#  - mozzo Ø22 con foro a D (Ø4, 3.5 sul piatto)
#
# LATO MOTORE = faccia SUPERIORE (quella opposta al disco).
# Con la staffa motor_mount la faccia della ruota sta a ~3 mm dal riduttore:
# l'albero entra per ~8.5 mm. Per questo:
#  - all'ingresso c'e una svasatura tonda per il tratto non fresato dell'albero
#  - bloccaggio con una vite a brugola M3x10 (testa cilindrica, dal kit) che
#    spinge sul piatto dell'albero, a 4.5 mm dalla faccia lato motore.
#    Il dado M3 sta in una sede aperta sulla faccia lato motore; la testa
#    appoggia su una sede piana sul mozzo e resta nello spazio tra mozzo e cerchio.
#  - la vite si stringe con una chiave a L da 2.5 mm: il braccio lungo passa
#    nella finestra allineata con la vite (direzione +X), il braccio corto
#    entra nella testa
#
# Le misure sono Parametri utente: Modifica > Cambia parametri.
# Forma del disco, razze e finestre e numero di gole: cambiare i valori
# qui sotto e rieseguire lo script.

import adsk.core
import adsk.fusion
import traceback
import math

# ---------------- PARAMETRI (nome, espressione, unita, commento) ----------------
PARAMS = [
    # Cerchio e gole
    ("outer_dia",       "89.5 mm", "mm", "Diametro esterno del cerchio (creste)"),
    ("groove_count",    "3",      "",   "Numero di gole (rieseguire lo script se cambia)"),
    ("groove_depth",    "3.5 mm", "mm", "Profondita gola (fondo = outer_dia - 2*depth = 82.5)"),
    ("groove_straight", "3.6 mm", "mm", "Tratto verticale della gola sul fondo, sotto il soffitto a 45 gradi"),
    ("groove_width",    "groove_straight + groove_depth", "mm", "Apertura della gola sulla superficie (calcolata)"),
    ("groove_ridge",    "2 mm",   "mm", "Cresta piena tra due gole"),
    ("groove_edge",     "2 mm",   "mm", "Bordo pieno sui due lati esterni"),
    ("rim_width",       "2 * groove_edge + groove_count * groove_width + (groove_count - 1) * groove_ridge",
                        "mm", "Larghezza totale della ruota (calcolata)"),
    ("rim_wall",        "4 mm",   "mm", "Spessore radiale del cerchio sotto il fondo gola"),
    # Disco a cono (rieseguire lo script se cambiano)
    ("web_t_rim",       "4 mm",   "mm", "Spessore del disco vicino al cerchio"),
    ("web_t_hub",       "8 mm",   "mm", "Spessore del disco vicino al mozzo"),
    ("fillet_r",        "2 mm",   "mm", "Raccordo disco-cerchio e disco-mozzo"),
    # Razze curve (rieseguire lo script se cambiano)
    ("spoke_count",     "5",      "",   "Numero di razze (e di finestre)"),
    ("spoke_width",     "7 mm",   "mm", "Larghezza di ogni razza"),
    ("spoke_sweep",     "24 deg", "deg", "Curvatura: rotazione della razza dal mozzo al cerchio"),
    ("win_r_in",        "15 mm",  "mm", "Raggio interno delle finestre"),
    ("win_r_out",       "33 mm",  "mm", "Raggio esterno delle finestre"),
    ("win_fillet",      "3 mm",   "mm", "Raccordo degli angoli delle finestre"),
    ("key_access_r",    "27 mm",  "mm", "Raggio a cui la finestra e centrata sulla vite"),
    # Finitura
    ("edge_chamfer",    "0.5 mm", "mm", "Smusso degli spigoli (faccia esterna e lato motore)"),
    # Mozzo e albero
    ("hub_dia",         "22 mm",  "mm", "Diametro del mozzo centrale"),
    ("shaft_dia",       "4 mm",   "mm", "Diametro albero a D"),
    ("shaft_flat",      "3.5 mm", "mm", "Misura sul piatto dell'albero a D"),
    ("shaft_clearance", "0.15 mm", "mm", "Gioco per la stampa (valore scelto col provino)"),
    ("shaft_relief_dia", "4.6 mm", "mm", "Svasatura tonda lato motore (tratto tondo albero)"),
    ("shaft_relief_depth", "1.5 mm", "mm", "Profondita svasatura lato motore"),
    # Vite di bloccaggio M3x10 a brugola
    ("screw_dia",       "3.2 mm", "mm", "Foro passante per la vite M3"),
    ("screw_z",         "rim_width - 4.5 mm", "mm", "Quota della vite (4.5 mm dalla faccia lato motore)"),
    ("screw_len",       "10 mm",  "mm", "Lunghezza della vite (sotto testa)"),
    ("screw_seat_x",    "(shaft_flat + shaft_clearance) - (shaft_dia + shaft_clearance) / 2 + screw_len - 1 mm",
                        "mm", "Distanza asse -> appoggio testa (la punta ha 1 mm di corsa oltre il piatto)"),
    ("seat_dia",        "6.5 mm", "mm", "Diametro della sede piana per la testa"),
    ("nut_af",          "5.7 mm", "mm", "Larghezza sede dado M3 (chiave 5.5 + gioco)"),
    ("nut_thick",       "2.7 mm", "mm", "Spessore sede dado M3 (2.4 + gioco)"),
    ("nut_pos",         "6.5 mm", "mm", "Distanza asse ruota -> centro del dado"),
    # O-ring (solo per il controllo di allungamento)
    ("oring_id",        "80 mm",  "mm", "Diametro interno O-ring"),
    ("oring_cord",      "5 mm",   "mm", "Spessore corda O-ring"),
]


def VI(expr):
    return adsk.core.ValueInput.createByString(expr)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Nuovo documento parametrico
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent
        extrudes = root.features.extrudeFeatures
        user_params = design.userParameters
        OP = adsk.fusion.FeatureOperations
        DIM = adsk.fusion.DimensionOrientations
        warnings = []

        # --- Parametri utente ---
        for name, expr, unit, comment in PARAMS:
            p = user_params.itemByName(name)
            if p:
                p.expression = expr
            else:
                user_params.add(name, VI(expr), unit, comment)

        def val(name):
            # valore interno: Fusion lavora in cm e radianti
            return user_params.itemByName(name).value

        # Controllo: bordi + gole + creste devono riempire la larghezza
        n_grooves = int(round(val("groove_count")))
        needed = 2 * val("groove_edge") + n_grooves * val("groove_width") \
            + (n_grooves - 1) * val("groove_ridge")
        if abs(needed - val("rim_width")) > 1e-4:
            warnings.append("Bordi + gole + creste = {:.2f} mm, ma rim_width = {:.2f} mm."
                            .format(needed * 10, val("rim_width") * 10))

        # --- Funzioni di supporto ---
        origin = adsk.core.Point3D.create(0, 0, 0)
        curve_types = (adsk.core.Circle3D.classType(), adsk.core.Arc3D.classType())

        def pt(x, y, z=0):
            return adsk.core.Point3D.create(x, y, z)

        def polar(r, a):
            return pt(r * math.cos(a), r * math.sin(a))

        def dim_circle(sk, dia_cm, expr):
            """Cerchio centrato nell'origine con quota diametro legata a un'espressione."""
            c = sk.sketchCurves.sketchCircles.addByCenterRadius(origin, dia_cm / 2)
            sk.geometricConstraints.addCoincident(c.centerSketchPoint, sk.originPoint)
            t = dia_cm / 2 * 0.7
            d = sk.sketchDimensions.addDiameterDimension(c, pt(t, t))
            d.parameter.expression = expr
            return c

        def ring_profile(sk):
            for p in sk.profiles:
                if p.profileLoops.count == 2:
                    return p
            raise RuntimeError("Profilo ad anello non trovato in " + sk.name)

        def largest_profile(sk):
            return max(sk.profiles, key=lambda p: p.areaProperties().area)

        def offset_plane(expr, name):
            pin = root.constructionPlanes.createInput()
            pin.setByOffset(root.xYConstructionPlane, VI(expr))
            pl = root.constructionPlanes.add(pin)
            pl.name = name
            return pl

        def circle_edges(body, specs, tol=1e-3):
            """Spigoli circolari centrati sull'asse, specs = [(raggio_cm, z_cm), ...]."""
            found = adsk.core.ObjectCollection.create()
            for e in body.edges:
                g = e.geometry
                if g.objectType not in curve_types:
                    continue
                if abs(g.center.x) > tol or abs(g.center.y) > tol:
                    continue
                if any(abs(g.radius - r) < tol and abs(g.center.z - z) < tol for r, z in specs):
                    found.add(e)
            return found

        def fillet(edges, expr, name):
            if edges.count == 0:
                raise RuntimeError("spigoli non trovati")
            f_in = root.features.filletFeatures.createInput()
            try:
                f_in.edgeSetInputs.addConstantRadiusEdgeSet(edges, VI(expr), True)
            except AttributeError:
                f_in.addConstantRadiusEdgeSet(edges, VI(expr), True)
            root.features.filletFeatures.add(f_in).name = name

        def chamfer(edges, expr, name):
            if edges.count == 0:
                raise RuntimeError("spigoli non trovati")
            chamfers = root.features.chamferFeatures
            try:
                ch_in = chamfers.createInput2()
                ch_in.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, VI(expr), True)
            except AttributeError:
                ch_in = chamfers.createInput(edges, True)
                ch_in.setToEqualDistance(VI(expr))
            chamfers.add(ch_in).name = name

        rim_inner_cm = val("outer_dia") - 2 * val("groove_depth") - 2 * val("rim_wall")
        rim_r = rim_inner_cm / 2
        hub_r = val("hub_dia") / 2
        t_rim, t_hub = val("web_t_rim"), val("web_t_hub")
        width = val("rim_width")

        # --- 1. Anello del cerchio ---
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Rim"
        dim_circle(sk, val("outer_dia"), "outer_dia")
        dim_circle(sk, rim_inner_cm, "outer_dia - 2 * groove_depth - 2 * rim_wall")
        rim = extrudes.addSimple(ring_profile(sk), VI("rim_width"), OP.NewBodyFeatureOperation)
        rim.name = "Rim"
        body = rim.bodies.item(0)
        body.name = "WheelRim"

        # --- 2. Mozzo centrale ---
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Hub"
        dim_circle(sk, val("hub_dia"), "hub_dia")
        extrudes.addSimple(sk.profiles.item(0), VI("rim_width"),
                           OP.JoinFeatureOperation).name = "Hub"

        # --- 3. Disco a cono (rivoluzione di un profilo nel piano XZ) ---
        sk = root.sketches.add(root.xZConstructionPlane)
        sk.name = "WebProfile"
        r_embed = rim_r + val("rim_wall") / 2      # entra per meta nella parete del cerchio
        r_axis = hub_r / 2                          # parte da dentro il mozzo
        pts_model = [pt(r_axis, 0, 0), pt(r_embed, 0, 0), pt(r_embed, 0, t_rim),
                     pt(rim_r, 0, t_rim), pt(hub_r, 0, t_hub), pt(r_axis, 0, t_hub)]
        pts = [sk.modelToSketchSpace(p) for p in pts_model]
        lines = sk.sketchCurves.sketchLines
        for i in range(len(pts)):
            lines.addByTwoPoints(pts[i], pts[(i + 1) % len(pts)])
        rev_in = root.features.revolveFeatures.createInput(
            sk.profiles.item(0), root.zConstructionAxis, OP.JoinFeatureOperation)
        rev_in.setAngleExtent(False, VI("360 deg"))
        root.features.revolveFeatures.add(rev_in).name = "Web"

        # --- 4. Raccordi disco-cerchio e disco-mozzo ---
        try:
            fillet(circle_edges(body, [(rim_r, t_rim), (hub_r, t_hub)]), "fillet_r", "WebFillets")
        except Exception as ex:
            warnings.append("Raccordi del disco non creati ({}): aggiungili a mano.".format(ex))

        # --- 5. Finestre a goccia tra 5 razze curve ---
        n = int(round(val("spoke_count")))
        r_in, r_out = val("win_r_in"), val("win_r_out")
        sweep, half_w = val("spoke_sweep"), val("spoke_width") / 2
        step = 2 * math.pi / n
        # la finestra numero 0 e centrata sulla direzione +X (grano) al raggio key_access_r
        a0 = -step / 2 - sweep * (val("key_access_r") - r_in) / (r_out - r_in)

        def spoke_angle(k, r):
            return a0 + k * step + sweep * (r - r_in) / (r_out - r_in)

        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "SpokeWindows"
        sk.isComputeDeferred = True
        sk.sketchCurves.sketchCircles.addByCenterRadius(origin, r_in)
        sk.sketchCurves.sketchCircles.addByCenterRadius(origin, r_out)
        rs = (r_in - 0.15, (r_in + r_out) / 2, r_out + 0.15)  # sporgono oltre i cerchi
        for k in range(n):
            for side in (-1, 1):
                p = [polar(r, spoke_angle(k, r) + side * half_w / r) for r in rs]
                sk.sketchCurves.sketchArcs.addByThreePoints(p[0], p[1], p[2])
        sk.isComputeDeferred = False

        def centroid_r(prof):
            c = prof.areaProperties().centroid
            return math.hypot(c.x, c.y)

        cands = [p for p in sk.profiles if r_in < centroid_r(p) < r_out]
        cands.sort(key=lambda p: p.areaProperties().area, reverse=True)
        if len(cands) < n:
            raise RuntimeError("Finestre non trovate nello sketch SpokeWindows")
        wins = adsk.core.ObjectCollection.create()
        for p in cands[:n]:
            wins.add(p)
        extrudes.addSimple(wins, VI("rim_width"), OP.CutFeatureOperation).name = "SpokeWindows"

        # Raccordo degli angoli delle finestre (spigoli verticali)
        try:
            corner = adsk.core.ObjectCollection.create()
            for e in body.edges:
                g = e.geometry
                if g.objectType != adsk.core.Line3D.classType():
                    continue
                a, b = g.startPoint, g.endPoint
                if abs(a.x - b.x) > 1e-4 or abs(a.y - b.y) > 1e-4:
                    continue
                if r_in - 1e-3 < math.hypot(a.x, a.y) < r_out + 1e-3:
                    corner.add(e)
            fillet(corner, "win_fillet", "WindowFillets")
        except Exception as ex:
            warnings.append("Raccordi delle finestre non creati ({}).".format(ex))

        # --- 6. Foro a D (piatto rivolto verso +X) ---
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "D_Hole"
        r = (val("shaft_dia") + val("shaft_clearance")) / 2
        x0 = (val("shaft_flat") + val("shaft_clearance")) - r  # distanza centro -> piatto
        circ = dim_circle(sk, 2 * r, "shaft_dia + shaft_clearance")
        y0 = math.sqrt(max(r * r - x0 * x0, 1e-6))
        line = sk.sketchCurves.sketchLines.addByTwoPoints(pt(x0, -y0), pt(x0, y0))
        gc = sk.geometricConstraints
        gc.addVertical(line)
        gc.addCoincident(line.startSketchPoint, circ)
        gc.addCoincident(line.endSketchPoint, circ)
        sk.sketchDimensions.addDistanceDimension(
            sk.originPoint, line.startSketchPoint, DIM.HorizontalDimensionOrientation,
            pt(x0 / 2, -r * 1.5)).parameter.expression = \
            "(shaft_flat + shaft_clearance) - (shaft_dia + shaft_clearance) / 2"
        extrudes.addSimple(largest_profile(sk), VI("rim_width"),
                           OP.CutFeatureOperation).name = "D_Hole"

        # --- 7. Svasatura tonda lato motore ---
        relief_pl = offset_plane("rim_width - shaft_relief_depth", "ShaftReliefPlane")
        sk = root.sketches.add(relief_pl)
        sk.name = "ShaftRelief"
        dim_circle(sk, val("shaft_relief_dia"), "shaft_relief_dia")
        extrudes.addSimple(sk.profiles.item(0), VI("shaft_relief_depth + 1 mm"),
                           OP.CutFeatureOperation).name = "ShaftRelief"

        # --- 8. Vite M3 radiale sul piatto, lato motore ---
        sk = root.sketches.add(root.yZConstructionPlane)
        sk.name = "Screw"
        cs = sk.modelToSketchSpace(pt(0, 0, val("screw_z")))
        sc = sk.sketchCurves.sketchCircles.addByCenterRadius(cs, val("screw_dia") / 2)
        sk.sketchDimensions.addDiameterDimension(
            sc, pt(cs.x + val("screw_dia"), cs.y + val("screw_dia"))).parameter.expression = "screw_dia"
        prof = sk.profiles.item(0)

        def screw_cut(direction):
            inp = extrudes.createInput(prof, OP.CutFeatureOperation)
            inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI("hub_dia")),
                                 direction)
            return extrudes.add(inp)

        screw = screw_cut(adsk.fusion.ExtentDirections.PositiveExtentDirection)
        max_x = max((f.boundingBox.maxPoint.x for f in screw.faces), default=1.0)
        if max_x < 0.05:  # e finito sul lato tondo: rifallo verso +X
            screw.deleteMe()
            screw = screw_cut(adsk.fusion.ExtentDirections.NegativeExtentDirection)
        screw.name = "ScrewHole"

        # Sede piana per la testa della vite, sul fianco del mozzo
        seat_pin = root.constructionPlanes.createInput()
        seat_pin.setByOffset(root.yZConstructionPlane, VI("screw_seat_x"))
        seat_pl = root.constructionPlanes.add(seat_pin)
        seat_pl.name = "ScrewSeatPlane"
        sk = root.sketches.add(seat_pl)
        sk.name = "ScrewSeat"
        cs = sk.modelToSketchSpace(pt(val("screw_seat_x"), 0, val("screw_z")))
        sk.sketchCurves.sketchCircles.addByCenterRadius(cs, val("seat_dia") / 2)
        seat_prof = sk.profiles.item(0)

        def seat_cut(direction):
            inp = extrudes.createInput(seat_prof, OP.CutFeatureOperation)
            inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI("5 mm")), direction)
            return extrudes.add(inp)

        seat = seat_cut(adsk.fusion.ExtentDirections.PositiveExtentDirection)
        min_x = min((f.boundingBox.minPoint.x for f in seat.faces), default=0.0)
        if min_x < val("screw_seat_x") - 0.01:  # e andata verso l'asse: rifalla verso l'esterno
            seat.deleteMe()
            seat = seat_cut(adsk.fusion.ExtentDirections.NegativeExtentDirection)
        seat.name = "ScrewSeat"

        # Sede del dado: dal centro della vite (meno mezza diagonale) fino alla faccia superiore
        nut_plane = offset_plane("screw_z - nut_af * 0.58", "NutTrapPlane")
        sk = root.sketches.add(nut_plane)
        sk.name = "NutTrap"
        nx, nt, na = val("nut_pos"), val("nut_thick"), val("nut_af")
        sk.sketchCurves.sketchLines.addTwoPointRectangle(
            pt(nx - nt / 2, -na / 2), pt(nx + nt / 2, na / 2))
        extrudes.addSimple(sk.profiles.item(0), VI("rim_width"),
                           OP.CutFeatureOperation).name = "NutTrap"

        # --- 9. Gole per O-ring ---
        groove_bottom_cm = val("outer_dia") - 2 * val("groove_depth")
        rb, R = groove_bottom_cm / 2, val("outer_dia") / 2
        h, gw = val("groove_straight"), val("groove_width")
        for i in range(n_grooves):
            z0 = val("groove_edge") + i * (gw + val("groove_ridge"))
            sk = root.sketches.add(root.xZConstructionPlane)
            sk.name = "Groove{}".format(i + 1)
            prof_pts = [pt(rb, 0, z0), pt(R + 0.3, 0, z0), pt(R + 0.3, 0, z0 + gw + 0.3),
                        pt(R, 0, z0 + h + (R - rb)), pt(rb, 0, z0 + h)]
            sp = [sk.modelToSketchSpace(q) for q in prof_pts]
            for k in range(len(sp)):
                sk.sketchCurves.sketchLines.addByTwoPoints(sp[k], sp[(k + 1) % len(sp)])
            rv = root.features.revolveFeatures.createInput(
                sk.profiles.item(0), root.zConstructionAxis, OP.CutFeatureOperation)
            rv.setAngleExtent(False, VI("360 deg"))
            root.features.revolveFeatures.add(rv).name = "Groove{}".format(i + 1)

        # --- 10. Smussi ---
        # Faccia esterna (sul piatto): tutti gli spigoli, foro a D e finestre compresi
        try:
            bed = adsk.core.ObjectCollection.create()
            for e in body.edges:
                bb = e.boundingBox
                if abs(bb.minPoint.z) < 1e-4 and abs(bb.maxPoint.z) < 1e-4:
                    bed.add(e)
            chamfer(bed, "edge_chamfer", "OuterFaceChamfer")
        except Exception as ex:
            warnings.append("Smussi della faccia esterna non creati ({}).".format(ex))
        # Faccia lato motore: bordo esterno e interno del cerchio, bordo del mozzo
        try:
            r_outer = val("outer_dia") / 2
            chamfer(circle_edges(body, [(r_outer, width), (rim_r, width), (hub_r, width)]),
                    "edge_chamfer", "MotorFaceChamfer")
        except Exception as ex:
            warnings.append("Smussi del lato motore non creati ({}).".format(ex))

        # --- Controllo O-ring ---
        cord = val("oring_cord")
        stretch = (groove_bottom_cm + cord) / (val("oring_id") + cord) - 1
        cord_mounted = cord / math.sqrt(1 + stretch)  # la corda si assottiglia tirandola
        info = ("O-ring {:.0f} x {:.1f}: allungamento ~{:.1f}%, corda montata ~{:.2f} mm, "
                "sporgenza ~{:.2f} mm, diametro con gomme ~{:.1f} mm."
                .format(val("oring_id") * 10, cord * 10, stretch * 100, cord_mounted * 10,
                        (cord_mounted - val("groove_depth")) * 10,
                        (groove_bottom_cm + 2 * cord_mounted) * 10))
        rc = cord_mounted / 2   # la corda poggia sul fondo e sul pavimento della gola
        clear = (val("groove_straight") - rc) * math.cos(math.pi / 4) + rc * math.sin(math.pi / 4) - rc
        info += "\nDistanza corda-soffitto della gola ~{:.2f} mm.".format(clear * 10)
        if clear < 0:
            warnings.append("Il soffitto a 45 gradi schiaccia l'O-ring: aumenta groove_straight.")

        app.activeViewport.fit()
        msg = ("Cerchione creato (larghezza {:.1f} mm).\nStampa in piano con la faccia esterna sul piatto, "
               "senza supporti.\nLato motore = faccia superiore.\n\n".format(width * 10) + info)
        if warnings:
            msg += "\n\nAttenzione:\n- " + "\n- ".join(warnings)
        ui.messageBox(msg)

    except Exception:
        if ui:
            ui.messageBox("Errore:\n{}".format(traceback.format_exc()))
