import adsk.core, adsk.fusion, math

def run():
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent

    # =========================================================================
    # 1. CLEANUP & PARAMETERS
    # =========================================================================
    while root.occurrences.count > 0:
        root.occurrences.item(0).deleteMe()
    while root.features.count > 0:
        root.features.item(0).deleteMe()
    for p in list(design.userParameters):
        p.deleteMe()

    params = design.userParameters
    p_defs = [
        ("head_w", "72 mm", "Head outer width"),
        ("head_h", "60 mm", "Head outer height"),
        ("head_d", "56 mm", "Head outer depth"),
        ("wall_th", "2.5 mm", "Enclosure wall thickness"),
        ("corner_r", "18 mm", "Front squircle corner radius"),
        ("visor_w", "48 mm", "Visor bezel outer width"),
        ("visor_h", "38 mm", "Visor bezel outer height"),
        ("screen_w", "29 mm", "1.5 inch LCD screen viewing window"),
        ("usbc_w", "12 mm", "USB-C port cutout width"),
        ("usbc_h", "6.8 mm", "USB-C port cutout height"),
        ("spk_hole_dia", "2.0 mm", "Speaker grille hole diameter"),
        ("ear_dia", "18 mm", "Ear puck diameter"),
        ("ear_th", "4.5 mm", "Ear puck thickness"),
        ("antenna_stem_dia", "3.5 mm", "Antenna stem diameter"),
        ("antenna_stem_len", "13 mm", "Antenna stem height"),
        ("antenna_ball_dia", "8 mm", "Antenna top sphere diameter")
    ]
    for name, expr, comment in p_defs:
        params.add(name, adsk.core.ValueInput.createByString(expr), "mm", comment)

    print("Parametric definitions registered.")

    def create_squircle_curves(sk, w, h, cr, offset_x=0.0, offset_y=0.0):
        cx = w/2.0 - cr
        cy = h/2.0 - cr
        arcs = sk.sketchCurves.sketchArcs
        lines = sk.sketchCurves.sketchLines
        p1 = adsk.core.Point3D.create(offset_x - cx, offset_y + cy + cr, 0)
        p2 = adsk.core.Point3D.create(offset_x + cx, offset_y + cy + cr, 0)
        p3 = adsk.core.Point3D.create(offset_x + cx + cr, offset_y + cy, 0)
        p4 = adsk.core.Point3D.create(offset_x + cx + cr, offset_y - cy, 0)
        p5 = adsk.core.Point3D.create(offset_x + cx, offset_y - cy - cr, 0)
        p6 = adsk.core.Point3D.create(offset_x - cx, offset_y - cy - cr, 0)
        p7 = adsk.core.Point3D.create(offset_x - cx - cr, offset_y - cy, 0)
        p8 = adsk.core.Point3D.create(offset_x - cx - cr, offset_y + cy, 0)
        lines.addByTwoPoints(p1, p2)
        arcs.addByCenterStartSweep(adsk.core.Point3D.create(offset_x + cx, offset_y + cy, 0), p2, -math.pi/2)
        lines.addByTwoPoints(p3, p4)
        arcs.addByCenterStartSweep(adsk.core.Point3D.create(offset_x + cx, offset_y - cy, 0), p4, -math.pi/2)
        lines.addByTwoPoints(p5, p6)
        arcs.addByCenterStartSweep(adsk.core.Point3D.create(offset_x - cx, offset_y - cy, 0), p6, -math.pi/2)
        lines.addByTwoPoints(p7, p8)
        arcs.addByCenterStartSweep(adsk.core.Point3D.create(offset_x - cx, offset_y + cy, 0), p8, -math.pi/2)

    # =========================================================================
    # 2. FRONT SHELL
    # =========================================================================
    occ_front = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    front_comp = occ_front.component
    front_comp.name = "Front_Shell"

    planes_f = front_comp.constructionPlanes
    exts_f = front_comp.features.extrudeFeatures

    # Loft sections along -Y (Y = 0 to -2.6 cm)
    sketches_f = []
    stations_f = [0.0, -0.9, -1.8, -2.6]
    for y_pos in stations_f:
        if abs(y_pos) < 1e-4:
            plane = front_comp.xZConstructionPlane
        else:
            p_in = planes_f.createInput()
            p_in.setByOffset(front_comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(y_pos))
            plane = planes_f.add(p_in)
        
        u = abs(y_pos) / 2.85
        factor = math.sqrt(max(0.1, 1.0 - 0.55 * (u**2)))
        w_curr = 7.2 * factor
        h_curr = 6.0 * factor
        cr_curr = 1.8 * factor
        
        sk = front_comp.sketches.add(plane)
        create_squircle_curves(sk, w_curr, h_curr, cr_curr)
        sketches_f.append(sk)

    loftFeats_f = front_comp.features.loftFeatures
    loftInput_f = loftFeats_f.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for sk in sketches_f:
        loftInput_f.loftSections.add(sk.profiles.item(0))
    loftInput_f.isSolid = True
    loft_f = loftFeats_f.add(loftInput_f)
    body_f = loft_f.bodies.item(0)

    # Fillet outer front cap (6 mm)
    edgeColl_fc = adsk.core.ObjectCollection.create()
    for f in body_f.faces:
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            mid = f.pointOnFace
            if mid.y < -2.0:
                for e in f.edges:
                    edgeColl_fc.add(e)
    if edgeColl_fc.count > 0:
        filletInput = front_comp.features.filletFeatures.createInput()
        filletInput.addConstantRadiusEdgeSet(edgeColl_fc, adsk.core.ValueInput.createByString("6 mm"), True)
        front_comp.features.filletFeatures.add(filletInput)

    # Shell from mating plane at Y = 0 (thickness 2.5 mm)
    faceColl_f = adsk.core.ObjectCollection.create()
    for f in body_f.faces:
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            mid = f.pointOnFace
            n = f.geometry.normal
            if abs(mid.y) < 0.01 and abs(abs(n.y) - 1.0) < 0.01:
                faceColl_f.add(f)
                break
    shellInput_f = front_comp.features.shellFeatures.createInput(faceColl_f, False)
    shellInput_f.insideThickness = adsk.core.ValueInput.createByString("2.5 mm")
    front_comp.features.shellFeatures.add(shellInput_f)

    # Front Visor Recess Pocket and Screen Aperture
    p_in = planes_f.createInput()
    p_in.setByOffset(front_comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(-3.5))
    front_cut_plane = planes_f.add(p_in)

    sk_visor = front_comp.sketches.add(front_cut_plane)
    create_squircle_curves(sk_visor, 4.8, 3.8, 1.2)   # 48 x 38 mm outer visor
    create_squircle_curves(sk_visor, 2.9, 2.9, 0.25)  # 29 x 29 mm inner screen aperture

    prof_both = adsk.core.ObjectCollection.create()
    prof_both.add(sk_visor.profiles.item(0))
    prof_both.add(sk_visor.profiles.item(1))

    # Visor pocket cut: 10.5 mm from Y=-3.5cm -> cuts ~1.7mm into front wall
    extInput_pocket = exts_f.createInput(prof_both, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_pocket.setDistanceExtent(False, adsk.core.ValueInput.createByString("10.5 mm"))
    exts_f.add(extInput_pocket)

    # Screen aperture through-cut
    extInput_aperture = exts_f.createInput(sk_visor.profiles.item(1), adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_aperture.setDistanceExtent(False, adsk.core.ValueInput.createByString("20.0 mm"))
    exts_f.add(extInput_aperture)

    # Visor outer rim fillet (0.8 mm)
    edgeColl_v = adsk.core.ObjectCollection.create()
    for e in body_f.edges:
        mid = e.pointOnEdge
        if mid.y < -2.4 and (abs(mid.x) > 1.5 or abs(mid.z) > 1.5):
            edgeColl_v.add(e)
    if edgeColl_v.count > 0:
        try:
            filletInput_v = front_comp.features.filletFeatures.createInput()
            filletInput_v.addConstantRadiusEdgeSet(edgeColl_v, adsk.core.ValueInput.createByString("0.8 mm"), True)
            front_comp.features.filletFeatures.add(filletInput_v)
        except:
            pass

    # 4 Corner Assembly Screw Bosses on mating plane (Y = 0)
    sk_bosses = front_comp.sketches.add(front_comp.xZConstructionPlane)
    bx, bz = 2.35, 1.75
    boss_outer_r, boss_inner_r = 0.30, 0.11 # outer dia 6.0mm, inner pilot hole dia 2.2mm
    circs_b = sk_bosses.sketchCurves.sketchCircles
    for sx in [-1, 1]:
        for sz in [-1, 1]:
            c = adsk.core.Point3D.create(sx * bx, sz * bz, 0)
            circs_b.addByCenterRadius(c, boss_outer_r)
            circs_b.addByCenterRadius(c, boss_inner_r)

    ring_profs = adsk.core.ObjectCollection.create()
    for p in sk_bosses.profiles:
        if p.profileLoops.count > 1:
            ring_profs.add(p)

    extInput_bosses = exts_f.createInput(ring_profs, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    extInput_bosses.setDistanceExtent(False, adsk.core.ValueInput.createByString("-14.0 mm"))
    exts_f.add(extInput_bosses)

    # Top Antenna Socket Hole (dia 4.2mm x 6.0mm deep)
    p_in = planes_f.createInput()
    p_in.setByOffset(front_comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(3.5))
    top_plane = planes_f.add(p_in)
    sk_ant_socket = front_comp.sketches.add(top_plane)
    sk_ant_socket.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, -0.6, 0), 0.21) # 4.2mm dia
    extInput_ant_s = exts_f.createInput(sk_ant_socket.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_ant_s.setDistanceExtent(False, adsk.core.ValueInput.createByString("-12.0 mm"))
    exts_f.add(extInput_ant_s)

    # Left & Right Ear Sockets
    p_in = planes_f.createInput()
    p_in.setByOffset(front_comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(-4.0))
    plane_l = planes_f.add(p_in)
    sk_el = front_comp.sketches.add(plane_l)
    sk_el.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(-0.6, 0, 0), 0.70) # 14mm dia recess
    sk_el.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(-0.6, 0, 0), 0.21) # 4.2mm dia pin hole
    extInput_el = exts_f.createInput(sk_el.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_el.setDistanceExtent(False, adsk.core.ValueInput.createByString("5.5 mm"))
    try:
        exts_f.add(extInput_el)
    except:
        pass

    p_in = planes_f.createInput()
    p_in.setByOffset(front_comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(4.0))
    plane_r = planes_f.add(p_in)
    sk_er = front_comp.sketches.add(plane_r)
    sk_er.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(-0.6, 0, 0), 0.70)
    sk_er.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(-0.6, 0, 0), 0.21)
    extInput_er = exts_f.createInput(sk_er.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_er.setDistanceExtent(False, adsk.core.ValueInput.createByString("-5.5 mm"))
    try:
        exts_f.add(extInput_er)
    except:
        pass

    # Desktop Table Stability Pad (Flat bottom pad at Z = -2.95 cm)
    p_in = planes_f.createInput()
    p_in.setByOffset(front_comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(-3.5))
    bot_plane_f = planes_f.add(p_in)
    sk_bot_f = front_comp.sketches.add(bot_plane_f)
    sk_bot_f.sketchCurves.sketchLines.addTwoPointRectangle(adsk.core.Point3D.create(-2.5, -2.5, 0), adsk.core.Point3D.create(2.5, 0.5, 0))
    extInput_bot_f = exts_f.createInput(sk_bot_f.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_bot_f.setDistanceExtent(False, adsk.core.ValueInput.createByString("5.5 mm"))
    try:
        exts_f.add(extInput_bot_f)
    except:
        pass

    for sk in front_comp.sketches:
        sk.isLightBulbOn = False
    for pl in front_comp.constructionPlanes:
        pl.isLightBulbOn = False

    print("Front_Shell complete.")

    # =========================================================================
    # 3. REAR SHELL
    # =========================================================================
    occ_rear = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    rear_comp = occ_rear.component
    rear_comp.name = "Rear_Shell"

    planes_r = rear_comp.constructionPlanes
    exts_r = rear_comp.features.extrudeFeatures

    # Loft sections along +Y (Y = 0 to +2.6 cm)
    sketches_r = []
    stations_r = [0.0, 0.9, 1.8, 2.6]
    for y_pos in stations_r:
        if abs(y_pos) < 1e-4:
            plane = rear_comp.xZConstructionPlane
        else:
            p_in = planes_r.createInput()
            p_in.setByOffset(rear_comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(y_pos))
            plane = planes_r.add(p_in)
        
        u = abs(y_pos) / 2.85
        factor = math.sqrt(max(0.1, 1.0 - 0.55 * (u**2)))
        w_curr = 7.2 * factor
        h_curr = 6.0 * factor
        cr_curr = 1.8 * factor
        
        sk = rear_comp.sketches.add(plane)
        create_squircle_curves(sk, w_curr, h_curr, cr_curr)
        sketches_r.append(sk)

    loftFeats_r = rear_comp.features.loftFeatures
    loftInput_r = loftFeats_r.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for sk in sketches_r:
        loftInput_r.loftSections.add(sk.profiles.item(0))
    loftInput_r.isSolid = True
    loft_r = loftFeats_r.add(loftInput_r)
    body_r = loft_r.bodies.item(0)

    # Fillet outer rear cap (6 mm)
    edgeColl_rc = adsk.core.ObjectCollection.create()
    for f in body_r.faces:
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            mid = f.pointOnFace
            if mid.y > 2.0:
                for e in f.edges:
                    edgeColl_rc.add(e)
    if edgeColl_rc.count > 0:
        filletInput = rear_comp.features.filletFeatures.createInput()
        filletInput.addConstantRadiusEdgeSet(edgeColl_rc, adsk.core.ValueInput.createByString("6 mm"), True)
        rear_comp.features.filletFeatures.add(filletInput)

    # Shell from Y = 0 (wall thickness 2.5 mm)
    faceColl_r = adsk.core.ObjectCollection.create()
    for f in body_r.faces:
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
            mid = f.pointOnFace
            n = f.geometry.normal
            if abs(mid.y) < 0.01 and abs(abs(n.y) - 1.0) < 0.01:
                faceColl_r.add(f)
                break
    shellInput_r = rear_comp.features.shellFeatures.createInput(faceColl_r, False)
    shellInput_r.insideThickness = adsk.core.ValueInput.createByString("2.5 mm")
    rear_comp.features.shellFeatures.add(shellInput_r)

    # Rear USB-C Cutout & Speaker Grille Holes
    p_in = planes_r.createInput()
    p_in.setByOffset(rear_comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(3.5))
    rear_cut_plane = planes_r.add(p_in)

    sk_rcuts = rear_comp.sketches.add(rear_cut_plane)

    # On rear plane: sketch Y is -Model Z!
    # USB-C at lower back: Model Z = -1.40 cm => sketch Y = +1.40 cm
    uw, uh, ucr = 1.20, 0.68, 0.18
    create_squircle_curves(sk_rcuts, uw, uh, ucr, offset_x=0.0, offset_y=1.40)

    # Audio Speaker Grille: directly ABOVE USB-C (Model Z = +0.55 cm => sketch Y = -0.55 cm)
    hole_r = 0.10 # 2.0mm dia
    pitch = 0.35 # 3.5mm pitch
    sk_grille_y = -0.55
    circs_g = sk_rcuts.sketchCurves.sketchCircles
    for row in [-2, -1, 0, 1, 2]:
        for col in [-2, -1, 0, 1, 2]:
            hx = col * pitch
            hy = sk_grille_y + row * pitch
            dist = math.sqrt(hx**2 + (hy - sk_grille_y)**2)
            if dist <= 0.85:
                circs_g.addByCenterRadius(adsk.core.Point3D.create(hx, hy, 0), hole_r)

    prof_rear_all = adsk.core.ObjectCollection.create()
    for p in sk_rcuts.profiles:
        prof_rear_all.add(p)

    extInput_rcuts = exts_r.createInput(prof_rear_all, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_rcuts.setDistanceExtent(False, adsk.core.ValueInput.createByString("-15.0 mm"))
    exts_r.add(extInput_rcuts)

    # 4 Corner Assembly Screw Through-Holes on Rear Shell
    sk_rholes = rear_comp.sketches.add(rear_comp.xZConstructionPlane)
    circs_rh = sk_rholes.sketchCurves.sketchCircles
    for sx in [-1, 1]:
        for sz in [-1, 1]:
            c = adsk.core.Point3D.create(sx * bx, sz * bz, 0)
            circs_rh.addByCenterRadius(c, 0.14) # 2.8mm dia clearance hole for M2.5 screw

    prof_rholes = adsk.core.ObjectCollection.create()
    for p in sk_rholes.profiles:
        prof_rholes.add(p)
    extInput_rholes = exts_r.createInput(prof_rholes, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_rholes.setDistanceExtent(False, adsk.core.ValueInput.createByString("28.0 mm"))
    try:
        exts_r.add(extInput_rholes)
    except:
        pass

    # Table Stability Pad on Rear Shell
    p_in = planes_r.createInput()
    p_in.setByOffset(rear_comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(-3.5))
    bot_plane_r = planes_r.add(p_in)
    sk_bot_r = rear_comp.sketches.add(bot_plane_r)
    sk_bot_r.sketchCurves.sketchLines.addTwoPointRectangle(adsk.core.Point3D.create(-2.5, -0.5, 0), adsk.core.Point3D.create(2.5, 2.5, 0))
    extInput_bot_r = exts_r.createInput(sk_bot_r.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_bot_r.setDistanceExtent(False, adsk.core.ValueInput.createByString("5.5 mm"))
    try:
        exts_r.add(extInput_bot_r)
    except:
        pass

    for sk in rear_comp.sketches:
        sk.isLightBulbOn = False
    for pl in rear_comp.constructionPlanes:
        pl.isLightBulbOn = False

    print("Rear_Shell complete.")

    # =========================================================================
    # 4. VISOR BEZEL
    # =========================================================================
    occ_visor = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    visor_comp = occ_visor.component
    visor_comp.name = "Visor_Bezel"

    planes_v = visor_comp.constructionPlanes
    p_in = planes_v.createInput()
    p_in.setByOffset(visor_comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(-2.62))
    v_plane = planes_v.add(p_in)
    sk_vb = visor_comp.sketches.add(v_plane)

    # 47.6 x 37.6 mm outer, 29 x 29 mm window
    create_squircle_curves(sk_vb, 4.76, 3.76, 1.18)
    create_squircle_curves(sk_vb, 2.90, 2.90, 0.25)

    exts_v = visor_comp.features.extrudeFeatures
    extInput_vb = exts_v.createInput(sk_vb.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput_vb.setDistanceExtent(False, adsk.core.ValueInput.createByString("-2.2 mm"))
    vb_ext = exts_v.add(extInput_vb)
    body_vb = vb_ext.bodies.item(0)

    # Fillet outer front perimeter of visor bezel (1.0 mm)
    edgeColl_vb = adsk.core.ObjectCollection.create()
    for e in body_vb.edges:
        mid = e.pointOnEdge
        if mid.y < -2.75 and (abs(mid.x) > 1.5 or abs(mid.z) > 1.5):
            edgeColl_vb.add(e)
    if edgeColl_vb.count > 0:
        try:
            filletInput_vb = visor_comp.features.filletFeatures.createInput()
            filletInput_vb.addConstantRadiusEdgeSet(edgeColl_vb, adsk.core.ValueInput.createByString("1.0 mm"), True)
            visor_comp.features.filletFeatures.add(filletInput_vb)
        except:
            pass

    for sk in visor_comp.sketches:
        sk.isLightBulbOn = False
    for pl in visor_comp.constructionPlanes:
        pl.isLightBulbOn = False

    print("Visor_Bezel complete.")

    # =========================================================================
    # 5. EARS (LEFT & RIGHT)
    # =========================================================================
    for side_name, sign_x in [("Ear_Left", -1.0), ("Ear_Right", 1.0)]:
        occ_ear = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        ear_comp = occ_ear.component
        ear_comp.name = side_name

        planes_e = ear_comp.constructionPlanes
        p_in = planes_e.createInput()
        p_in.setByOffset(ear_comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(sign_x * 3.55))
        ear_plane = planes_e.add(p_in)

        sk_ear = ear_comp.sketches.add(ear_plane)
        # Disk dia 18mm (radius 0.90cm), centered at Y = -0.60cm, Z = 0
        sk_ear.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(-0.60, 0, 0), 0.90)

        exts_e = ear_comp.features.extrudeFeatures
        extInput_e = exts_e.createInput(sk_ear.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extInput_e.setDistanceExtent(False, adsk.core.ValueInput.createByString(f"{sign_x * 4.5} mm"))
        ext_e = exts_e.add(extInput_e)
        body_e = ext_e.bodies.item(0)

        # Fillet outer rim of disk (1.2 mm)
        edgeColl_e = adsk.core.ObjectCollection.create()
        for edge in body_e.edges:
            mid = edge.pointOnEdge
            if abs(mid.x) > 3.8:
                edgeColl_e.add(edge)
        if edgeColl_e.count > 0:
            try:
                filletInput_e = ear_comp.features.filletFeatures.createInput()
                filletInput_e.addConstantRadiusEdgeSet(edgeColl_e, adsk.core.ValueInput.createByString("1.2 mm"), True)
                ear_comp.features.filletFeatures.add(filletInput_e)
            except:
                pass

        # Mounting pin: dia 4.0mm x length 4.5mm
        sk_pin = ear_comp.sketches.add(ear_plane)
        sk_pin.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(-0.60, 0, 0), 0.20)
        extInput_pin = exts_e.createInput(sk_pin.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
        extInput_pin.setDistanceExtent(False, adsk.core.ValueInput.createByString(f"{-sign_x * 4.5} mm"))
        try:
            exts_e.add(extInput_pin)
        except:
            pass

        for sk in ear_comp.sketches:
            sk.isLightBulbOn = False
        for pl in ear_comp.constructionPlanes:
            pl.isLightBulbOn = False

    print("Ears complete.")

    # =========================================================================
    # 6. TOP ANTENNA
    # =========================================================================
    occ_ant = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    ant_comp = occ_ant.component
    ant_comp.name = "Antenna"

    planes_a = ant_comp.constructionPlanes
    p_in = planes_a.createInput()
    p_in.setByOffset(ant_comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(2.95))
    ant_base_plane = planes_a.add(p_in)

    sk_ant_base = ant_comp.sketches.add(ant_base_plane)
    # Stem circle: dia 3.5mm (radius 0.175cm) at X=0, Y=-0.60cm
    sk_ant_base.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, -0.60, 0), 0.175)

    exts_a = ant_comp.features.extrudeFeatures
    extInput_stem = exts_a.createInput(sk_ant_base.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput_stem.setDistanceExtent(False, adsk.core.ValueInput.createByString("13.0 mm"))
    stem_ext = exts_a.add(extInput_stem)
    body_ant = stem_ext.bodies.item(0)

    # Antenna mounting pin: dia 4.0mm x 5.0mm down into socket
    sk_ant_pin = ant_comp.sketches.add(ant_base_plane)
    sk_ant_pin.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, -0.60, 0), 0.20)
    extInput_ant_pin = exts_a.createInput(sk_ant_pin.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    extInput_ant_pin.setDistanceExtent(False, adsk.core.ValueInput.createByString("-5.0 mm"))
    exts_a.add(extInput_ant_pin)

    # Sphere at top of stem:
    # Stem top is at (X=0, Y=-0.60cm, Z=4.25cm).
    # Using xZ plane offset to Y = -0.60cm:
    # On this plane, sketch Y is -Model Z! So Model Z = +4.25cm => sketch Y = -4.25cm!
    p_in = planes_a.createInput()
    p_in.setByOffset(ant_comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(-0.60))
    sphere_plane = planes_a.add(p_in)
    sk_sphere = ant_comp.sketches.add(sphere_plane)

    c_sph = adsk.core.Point3D.create(0, -4.25, 0)
    p_top = adsk.core.Point3D.create(0, -4.25 - 0.40, 0)
    p_bot = adsk.core.Point3D.create(0, -4.25 + 0.40, 0)
    axis_line = sk_sphere.sketchCurves.sketchLines.addByTwoPoints(p_bot, p_top)
    sk_sphere.sketchCurves.sketchArcs.addByCenterStartSweep(c_sph, p_top, math.pi)

    revFeats = ant_comp.features.revolveFeatures
    revInput = revFeats.createInput(sk_sphere.profiles.item(0), axis_line, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    revInput.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    revFeats.add(revInput)

    for sk in ant_comp.sketches:
        sk.isLightBulbOn = False
    for pl in ant_comp.constructionPlanes:
        pl.isLightBulbOn = False

    print("Antenna complete.")

    # =========================================================================
    # 7. LCD SCREEN MODULE & GLOWING ROBOT FACE
    # =========================================================================
    occ_lcd = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    lcd_comp = occ_lcd.component
    lcd_comp.name = "LCD_1_5_Inch_Module"

    planes_l = lcd_comp.constructionPlanes
    p_in = planes_l.createInput()
    p_in.setByOffset(lcd_comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(-2.60))
    lcd_plane = planes_l.add(p_in)

    # 1. Dark Screen Display Glass Panel (28 x 28 mm, 1.6mm thick)
    sk_lcd = lcd_comp.sketches.add(lcd_plane)
    create_squircle_curves(sk_lcd, 3.4, 3.6, 0.20) # Breakout PCB
    create_squircle_curves(sk_lcd, 2.85, 2.85, 0.15) # Display glass

    exts_l = lcd_comp.features.extrudeFeatures
    extInput_pcb = exts_l.createInput(sk_lcd.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput_pcb.setDistanceExtent(False, adsk.core.ValueInput.createByString("1.6 mm"))
    exts_l.add(extInput_pcb)

    extInput_glass = exts_l.createInput(sk_lcd.profiles.item(1), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput_glass.setDistanceExtent(False, adsk.core.ValueInput.createByString("1.6 mm"))
    glass_ext = exts_l.add(extInput_glass)

    # 2. Glowing Eyes and Smiling Mouth on front of LCD glass
    sk_face = lcd_comp.sketches.add(lcd_plane)

    # Pill Eyes: width 2.4mm, height 7.5mm, corner 1.2mm
    # Sketch Y is -Model Z, so Model Z = +0.20cm => sketch Y = -0.20cm
    ew, eh, ecr = 0.24, 0.75, 0.12
    ecx = ew/2.0 - ecr
    ecy = eh/2.0 - ecr
    eye_y = -0.20

    for ex_pos in [-0.55, 0.55]: # X = ±5.5mm
        p1 = adsk.core.Point3D.create(ex_pos - ecx, eye_y + ecy + ecr, 0)
        p2 = adsk.core.Point3D.create(ex_pos + ecx, eye_y + ecy + ecr, 0)
        p3 = adsk.core.Point3D.create(ex_pos + ecx + ecr, eye_y + ecy, 0)
        p4 = adsk.core.Point3D.create(ex_pos + ecx + ecr, eye_y - ecy, 0)
        p5 = adsk.core.Point3D.create(ex_pos + ecx, eye_y - ecy - ecr, 0)
        p6 = adsk.core.Point3D.create(ex_pos - ecx, eye_y - ecy - ecr, 0)
        p7 = adsk.core.Point3D.create(ex_pos - ecx - ecr, eye_y - ecy, 0)
        p8 = adsk.core.Point3D.create(ex_pos - ecx - ecr, eye_y + ecy, 0)
        sk_face.sketchCurves.sketchLines.addByTwoPoints(p1, p2)
        sk_face.sketchCurves.sketchArcs.addByCenterStartSweep(adsk.core.Point3D.create(ex_pos + ecx, eye_y + ecy, 0), p2, -math.pi/2)
        sk_face.sketchCurves.sketchLines.addByTwoPoints(p3, p4)
        sk_face.sketchCurves.sketchArcs.addByCenterStartSweep(adsk.core.Point3D.create(ex_pos + ecx, eye_y - ecy, 0), p4, -math.pi/2)
        sk_face.sketchCurves.sketchLines.addByTwoPoints(p5, p6)
        sk_face.sketchCurves.sketchArcs.addByCenterStartSweep(adsk.core.Point3D.create(ex_pos - ecx, eye_y - ecy, 0), p6, -math.pi/2)
        sk_face.sketchCurves.sketchLines.addByTwoPoints(p7, p8)
        sk_face.sketchCurves.sketchArcs.addByCenterStartSweep(adsk.core.Point3D.create(ex_pos - ecx, eye_y + ecy, 0), p8, -math.pi/2)

    # Smile arc: Model Z = -0.45cm => sketch Y = +0.45cm
    mouth_th = 0.12
    p_start = adsk.core.Point3D.create(-0.25, 0.45, 0)
    p_mid = adsk.core.Point3D.create(0, 0.58, 0)
    p_end = adsk.core.Point3D.create(0.25, 0.45, 0)
    sk_face.sketchCurves.sketchArcs.addByThreePoints(p_start, p_mid, p_end)
    p_start_in = adsk.core.Point3D.create(-0.25, 0.45 - mouth_th, 0)
    p_mid_in = adsk.core.Point3D.create(0, 0.58 - mouth_th, 0)
    p_end_in = adsk.core.Point3D.create(0.25, 0.45 - mouth_th, 0)
    sk_face.sketchCurves.sketchArcs.addByThreePoints(p_start_in, p_mid_in, p_end_in)
    sk_face.sketchCurves.sketchLines.addByTwoPoints(p_start, p_start_in)
    sk_face.sketchCurves.sketchLines.addByTwoPoints(p_end, p_end_in)

    prof_face = adsk.core.ObjectCollection.create()
    for p in sk_face.profiles:
        prof_face.add(p)

    extInput_face = exts_l.createInput(prof_face, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput_face.setDistanceExtent(False, adsk.core.ValueInput.createByString("-0.4 mm")) # project forward into viewing window
    exts_l.add(extInput_face)

    for sk in lcd_comp.sketches:
        sk.isLightBulbOn = False
    for pl in lcd_comp.constructionPlanes:
        pl.isLightBulbOn = False

    print("LCD Screen Module complete.")

    # =========================================================================
    # 8. ESP32-S3 BOARD REFERENCE MOCKUP
    # =========================================================================
    occ_esp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    esp_comp = occ_esp.component
    esp_comp.name = "ESP32_S3_Board"

    planes_esp = esp_comp.constructionPlanes
    p_in = planes_esp.createInput()
    p_in.setByOffset(esp_comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(-1.55))
    esp_plane = planes_esp.add(p_in)

    sk_esp = esp_comp.sketches.add(esp_plane)
    lines_esp = sk_esp.sketchCurves.sketchLines
    # Board PCB: 18mm wide (X: ±0.9cm) x 21.0mm long (Y: 0.1 to 2.20cm) - 100% inside enclosure
    lines_esp.addTwoPointRectangle(adsk.core.Point3D.create(-0.9, 0.1, 0), adsk.core.Point3D.create(0.9, 2.20, 0))

    exts_esp = esp_comp.features.extrudeFeatures
    extInput_esp = exts_esp.createInput(sk_esp.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput_esp.setDistanceExtent(False, adsk.core.ValueInput.createByString("1.2 mm"))
    exts_esp.add(extInput_esp)

    # USB-C Receptacle: 9.0 x 7.5 mm x 3.0 mm seated flush inside rear port cutout
    p_in = planes_esp.createInput()
    p_in.setByOffset(esp_comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(-1.43))
    usb_plane = planes_esp.add(p_in)
    sk_usb_rec = esp_comp.sketches.add(usb_plane)
    create_squircle_curves(sk_usb_rec, 0.90, 0.65, 0.15, offset_x=0.0, offset_y=1.95)
    extInput_rec = exts_esp.createInput(sk_usb_rec.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput_rec.setDistanceExtent(False, adsk.core.ValueInput.createByString("3.0 mm"))
    try:
        exts_esp.add(extInput_rec)
    except:
        pass

    for sk in esp_comp.sketches:
        sk.isLightBulbOn = False
    for pl in esp_comp.constructionPlanes:
        pl.isLightBulbOn = False

    print("ESP32-S3 Board complete.")

    # =========================================================================
    # 9. COLOR & MATERIAL ASSIGNMENT
    # =========================================================================
    fusion_lib = app.materialLibraries.itemByName("Fusion Appearance Library")

    def get_or_copy(lib_name):
        existing = design.appearances.itemByName(lib_name)
        if existing:
            return existing
        src = fusion_lib.appearances.itemByName(lib_name)
        if src:
            return design.appearances.addByCopy(src, lib_name)
        return None

    white_mat = get_or_copy("Plastic - Matte (White)")
    glossy_black = get_or_copy("Plastic - Glossy (Black)")
    matte_black = get_or_copy("Plastic - Matte (Black)")
    led_blue = get_or_copy("LED (Blue)")

    # Front & Rear Shells: Matte White
    if white_mat:
        front_comp.bRepBodies.item(0).appearance = white_mat
        rear_comp.bRepBodies.item(0).appearance = white_mat

    # Visor Bezel: Glossy Black
    if glossy_black:
        visor_comp.bRepBodies.item(0).appearance = glossy_black

    # Ears & Antenna: Matte Black
    if matte_black:
        for occ in root.occurrences:
            if "Ear" in occ.component.name or "Antenna" in occ.component.name:
                for b in occ.component.bRepBodies:
                    b.appearance = matte_black

    # LCD: Glass (Glossy Black), Eyes & Mouth (LED Blue)
    if glossy_black:
        if lcd_comp.bRepBodies.count > 0:
            lcd_comp.bRepBodies.item(0).appearance = matte_black # PCB
        if lcd_comp.bRepBodies.count > 1:
            lcd_comp.bRepBodies.item(1).appearance = glossy_black # Glass
    if led_blue:
        # Bodies 2, 3, 4 are the eyes and smile
        for i in range(2, lcd_comp.bRepBodies.count):
            lcd_comp.bRepBodies.item(i).appearance = led_blue

    # Activate root component so all parts are fully opaque and solid
    design.activateRootComponent()

    # Fit viewport
    vp = app.activeViewport
    cam = vp.camera
    cam.isFitView = True
    vp.camera = cam

    print("ALL 8 COMPONENTS GENERATED AND STYLED SUCCESSFULLY!")

run()
