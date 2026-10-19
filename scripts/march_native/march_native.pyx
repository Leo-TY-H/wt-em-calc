# cython: language_level=3, boundscheck=False, wraparound=False, cdivision=True, initializedcheck=False
"""Compiled evaluate-and-correct loop of the march chart (speed plan S2/S3, in production since S4).

Built by scripts/build_march_native.py into .native_march/ and used through
em_march_native only while the Python sources it was verified against are
unchanged (march_native/verified.json). Events: stall, wing force and
non-Instructor control corrections are native; the rest stay Python.

Evaluator: em_operating.operating_point with explicit stationary wing angles
(the JointProblem / Curve.correct residual) for direct-thrust aircraft, as a
typed transliteration that reproduces the baseline arithmetic bit for bit.
Per-Mach aerodynamic conditions and engine force vectors are taken from the
baseline Python model (condition_properties, engine.vectors) and cached by
their exact inputs; everything evaluated per state is native.
"""
import numpy as np
cimport numpy as cnp
from libc.string cimport memset, memcpy
from libc.stdlib cimport malloc, free, realloc

cnp.import_array()

include "frame.pxi"
include "tail.pxi"
include "op.pxi"
include "engine.pxi"
include "kinematics.pxi"
include "linalg.pxi"
include "corrector.pxi"
include "prop.pxi"
include "prop2.pxi"

cdef extern from "math.h" nogil:
    double fma(double x, double y, double z)

F_TINY = F(4e-19)
DEG_W = F(57.2957763671875)
RAD_W = F(0.01745329238474369)
PI_F = F(M_PI)

POLAR_FIELDS = ('cl0 cd0 indCoeff clLineCoeff cyCritH cyCritL aoaCritH aoaCritL '
                'aoaLineH aoaLineL parabCyCoeffH parabCyCoeffL aerCenterOffset '
                'clToCm0 clToCm1 parabAngle declineCoeff maxDistAng cdAfterCoeff '
                'clAfterCritL clAfterCritH kq clKq cyMult').split()


cdef void set_polar(Polar* p, dict d) except *:
    cdef double* q = <double*>p
    cdef int i
    for i, k in enumerate(POLAR_FIELDS):
        q[i] = float(d[k])


cdef void set_table(Table* t, rows, int width) except *:
    t.n = len(rows)
    if t.n > 12:raise ValueError('table too long')
    for i, row in enumerate(rows):
        t.x[i] = float(row[0]); t.inv[i] = float(row[1])
        values = row[2]
        if len(values) != width:raise ValueError('table width')
        for j in range(width):
            t.v[i][j] = float(values[j])


cdef void set_mix(Mix* m, dict props) except *:
    for i in range(3):
        m.angles[i][0] = float(props['angles'][i][0]); m.angles[i][1] = float(props['angles'][i][1])
        set_table(&m.t1[i], props['tables_1d'][i], 3)
        rows = props['tables_2d'][i]
        m.n2[i] = len(rows)
        if m.n2[i] > 10:raise ValueError('2D control table too long')
        for k, (dens, inv, table) in enumerate(rows):
            m.d2[i][k] = float(dens); m.i2[i][k] = float(inv)
            set_table(&m.t2[i][k], table, 3)
        m.use2[i] = bool(rows) and all(r[2] for r in rows)
    m.sensitivity = float(props['sensitivity'])
    set_table(&m.sens_curve, props['sensitivity_curve'], 1)
    m.cl[0] = float(props['cl'][0]); m.cl[1] = float(props['cl'][1])
    m.cd[0] = float(props['cd'][0]); m.cd[1] = float(props['cd'][1])
    m.wing_aoa = float(props['wing_aoa'])


cdef struct Cond:
    double mach
    double flap_values[4]
    Polar wing_raw
    Polar wing[2]
    TailProps tp
    int piece      # index of the stable Mach piece key (derivative branch)
    bint rough


cdef struct Vec:
    double u
    double force[3]
    double moment[3]


cdef struct History:
    double aoa[2]
    double cl[2]
    double body[2]
    double spin


cdef struct Frame:
    double force[3]          # total force
    double moment[3]         # stored (limited) moment
    double raw_force[3]
    double raw_moment[3]
    double helper[3]
    double engine_force[3]
    double engine_moment[3]
    double omega[3]          # omega_for_flow
    double component_forces[8][3]   # left_wing right_wing left_hstab right_hstab vstab fuselage chute parasite
    double component_points[7][3]
    double wc[2][5]
    double tc[3][5]
    double commands[3]
    Wing wing
    Tail tail
    History history


cdef struct Eval:
    double speed, load
    double x[7]
    double residual[5]
    double rate_residual[3]
    double angle_residual[2]
    double force_error_g
    double history_error
    double stall_margin, negative_stall_margin
    double wing_ratios[2]
    double velocity[3]
    Air air
    Geometry geom
    Allocation alloc
    History cycle_input
    Frame frame
    int cond
    Kinematic kin
    bint completed


cdef class Model:
    cdef WingGeom g
    cdef Mix mixers[3]       # Ailerons, Elevator, Rudder
    cdef Primary primary
    cdef DeviceDrag drag[2]
    cdef double mass, weight, G
    cdef double cog[3]
    cdef double inertia[3]
    cdef double dt, altitude, throttle, flaps, gear, ground_length, cockpit_door_cd, ext_thrust_base
    cdef bint convert_aoa, roll_leveling, torque_gyro, optimized, cfg_default_trim
    cdef double trim_limit
    cdef double fixed_trim[3]
    cdef double strength_force[2]
    cdef double strength_ias, strength_mach
    cdef Cond conds[32]
    cdef int ncond, cond_next
    cdef Vec vecs[64]
    cdef Unit units[8]
    cdef int nunits
    cdef public bint python_vectors
    cdef public bint prop
    cdef bint has_phase
    cdef double cur_phase[11]
    cdef int nvec, vec_next
    cdef public object solver, pieces, stats
    cdef public bint instructor
    cdef public long evaluations, cond_misses, vec_misses

    def __init__(self, solver):
        from wing_model import drag_terms
        self.solver = solver
        model = solver.model
        g = model['geometry'];fm = model['fm']
        self.g.span = g['span'];self.g.area = g['area']
        for i in range(2):
            for j in range(4):self.g.areas[i][j] = g['areas'][i][j]
        for i in range(3):self.g.arm[i] = g['arm'][i]
        for name, dst in (('FlapsShift', 0), ('GearShift', 1), ('AirbrakesShift', 2), ('ElevonShift', 3)):
            values = g['shifts'][name]
            for j in range(2):
                if dst == 0:self.g.flaps_shift[j] = values[j]
                elif dst == 1:self.g.gear_shift[j] = values[j]
                elif dst == 2:self.g.airbrakes_shift[j] = values[j]
                else:self.g.elevon_shift[j] = values[j]
        self.g.incidence = g['incidence'];self.g.dihedral = g['dihedral'];self.g.sine_aos = g['sine_aos']
        self.g.v_focus = g['v_focus'];self.g.aoa_shift = g['aoa_shift']
        pts = g['aoa_shift_add']
        if len(pts) > 10:raise ValueError('AoaShiftAdd table too long')
        self.g.n_shift_add = len(pts)
        for i, (x, y) in enumerate(pts):self.g.shift_add_x[i] = x;self.g.shift_add_y[i] = y
        self.g.shift_constant = not pts or all(y == pts[0][1] for _, y in pts)
        self.g.use_spin_loss = bool(g['use_spin_loss'])
        self.g.spin_loss[0] = g['spin_loss'][0];self.g.spin_loss[1] = g['spin_loss'][1]
        s = g['strength']
        self.strength_force[0] = s['force'][0];self.strength_force[1] = s['force'][1]
        self.strength_ias = s['ias'];self.strength_mach = s['mach']
        for i, name in enumerate(('Ailerons', 'Elevator', 'Rudder')):set_mix(&self.mixers[i], model['controls'][name])
        c = solver.controls
        for i in range(3):
            for j in range(2):
                self.primary.eff[i][j] = c['effective_speed'][i][j];self.primary.power[i][j] = c['power'][i][j]
            self.primary.minimum[i] = c['minimum'][i];self.primary.negative[i] = c['negative'][i]
            self.primary.trim_available[i] = bool(c['trim_available'][i])
        self.primary.strong = bool(c['strong']);self.primary.invert_elevator = bool(c['invert_elevator'])
        cfg = solver.config
        if cfg.get('experimental_instantaneous_aero') or cfg.get('experimental_lateral_trim', 'full') != 'full':
            raise ValueError('S2 native evaluator: full model only')
        if cfg.get('aircraft_trim_mode', 'discrete') != 'discrete':raise ValueError('discrete aircraft trim only')
        if getattr(solver, 'sideslip_attitude_deg', 0.) != 0.:raise ValueError('zero sideslip only')
        if model.get('mach_curve_mode') != 'stable':raise ValueError('stable Mach mode only')
        self.mass = solver.mass['mass'];self.weight = solver.weight
        for i in range(3):self.cog[i] = solver.mass['cog'][i];self.inertia[i] = solver.mass['inertia'][i]
        from body_dynamics import G
        self.G = float(G)
        self.dt = solver.dt;self.altitude = cfg['altitude_m'];self.throttle = cfg['throttle']
        self.flaps = solver.flaps;self.gear = solver.gear
        self.ground_length = fm['Length'];self.cockpit_door_cd = fm['Aerodynamics'].get('CockpitDoorCd', 0.)
        self.ext_thrust_base = fm.get('ExtThrustBaseMult', 1.)
        self.convert_aoa = bool(fm.get('ConvertAoa', False));self.roll_leveling = bool(fm.get('RollLeveling', True))
        self.torque_gyro = bool(cfg['torque_gyro'])
        self.instructor = bool(cfg['instructor'])
        self.optimized = cfg['trim_mode'] == 'optimized';self.trim_limit = cfg['trim_limit']
        for i in range(3):self.fixed_trim[i] = cfg['fixed_trim'][i]
        self.cfg_default_trim = cfg['trim_mode'] == 'optimized' and cfg['trim_limit'] == 1.
        drag = drag_terms(fm, gear_position=self.gear, airbrake_position=0., oil_radiator=solver.oil_radiator,
                          radiator=solver.radiator)
        for i in range(2):
            d = drag[i]
            self.drag[i].gear = d['gear'];self.drag[i].central_gear = d['central_gear'];self.drag[i].radiator = d['radiator']
            self.drag[i].oil_radiator = d['oil_radiator'];self.drag[i].fuselage = d['fuselage']
            self.drag[i].airbrake = d['airbrake'];self.drag[i].bomb_bay = d['bomb_bay']
        self.ncond = self.cond_next = self.nvec = self.vec_next = 0
        self.prop = bool(solver.is_prop)
        self.has_phase = False
        if self.prop:
            # Propeller propulsion stays a supplied sample (JointProblem.propulsion_at)
            self.nunits = 0
        else:
            units = solver.engine.units
            if len(units) > 8:raise ValueError('too many engines')
            self.nunits = len(units)
            for k, unit in enumerate(units):set_unit(&self.units[k], unit, cfg['throttle'])
        self.python_vectors = False
        self.pieces = {}
        self.evaluations = self.cond_misses = self.vec_misses = 0

    # ------------------------------------------------------------ caches
    cdef int condition(self, double mach) except -1:
        cdef int i
        for i in range(self.ncond):
            if self.conds[i].mach == mach:return i
        self.cond_misses += 1
        from aircraft_model import condition_properties
        from wing_area_normalization import intact_polars
        from em_mach_events import stable_piece_key, rough_mach
        values, wing, tp = condition_properties(self.solver.model, mach, self.flaps)
        if self.ncond < 32:
            i = self.ncond;self.ncond += 1
        else:
            i = self.cond_next;self.cond_next = (self.cond_next+1) % 32
        cdef Cond* c = &self.conds[i]
        c.mach = mach
        for j in range(4):c.flap_values[j] = values[j]
        set_polar(&c.wing_raw, wing)
        left, right = intact_polars(wing, self.solver.model['geometry']['areas'])
        set_polar(&c.wing[0], left);set_polar(&c.wing[1], right)
        set_polar(&c.tp.hstab, tp['polars']['hstab']);set_polar(&c.tp.vstab, tp['polars']['vstab'])
        set_polar(&c.tp.fuselage, tp['polars']['fuselage'])
        c.tp.span = tp['span'];c.tp.area = tp['area'];c.tp.sweep = tp['sweep'];c.tp.taper = tp['taper']
        c.tp.dihedral = tp['dihedral'];c.tp.downwash_coefficient = tp['downwash_coefficient']
        c.tp.travel_cap = tp.get('travel_cap', 1.);c.tp.engine_count = tp.get('engine_count', 1)
        c.tp.downwash_type = tp.get('downwash_type', 2);c.tp.clockwise = bool(tp.get('clockwise', False))
        for j in range(3):
            c.tp.arm_h[j] = tp['arms']['hstab'][j];c.tp.arm_v[j] = tp['arms']['vstab'][j];c.tp.arm_f[j] = tp['arms']['fuselage'][j]
        c.tp.inc_h = tp['incidence']['hstab'];c.tp.inc_v = tp['incidence']['vstab'];c.tp.inc_f = tp['incidence']['fuselage']
        c.tp.fi_h = tp['flow_inertia']['hstab'];c.tp.fi_v = tp['flow_inertia']['vstab'];c.tp.fi_f = tp['flow_inertia']['fuselage']
        a = tp['areas']
        c.tp.a_fuselage = a['fuselage'];c.tp.a_left_main = a['left_main'];c.tp.a_left_elevator = a['left_elevator']
        c.tp.a_right_main = a['right_main'];c.tp.a_right_elevator = a['right_elevator'];c.tp.a_v_main = a['v_main']
        c.tp.a_rudder = a['rudder']
        c.tp.slipstream_distance = tp.get('slipstream_distance', 5.72);c.tp.downwash_aspect = tp['downwash_aspect']
        key = stable_piece_key(self.solver, mach)
        c.piece = self.pieces.setdefault(key, len(self.pieces))
        c.rough = bool(rough_mach(self.solver, mach))
        return i

    cdef int vectors(self, double u, double* force, double* moment) except -1:
        cdef int i, j
        for i in range(self.nvec):
            if self.vecs[i].u == u:
                for j in range(3):force[j] = self.vecs[i].force[j];moment[j] = self.vecs[i].moment[j]
                return 0
        self.vec_misses += 1
        cdef double fsum[3]
        cdef double msum[3]
        cdef double fu[3]
        cdef double mu[3]
        if self.python_vectors:
            f, m = self.solver.engine.vectors(u, self.flaps)
        else:
            fsum[0] = fsum[1] = fsum[2] = 0.;msum[0] = msum[1] = msum[2] = 0.
            for k in range(self.nunits):
                unit_vectors(&self.units[k], u, self.flaps, self.cog, fu, mu)
                for j in range(3):fsum[j] += fu[j];msum[j] += mu[j]
            f = [fsum[0], fsum[1], fsum[2]];m = [msum[0], msum[1], msum[2]]
        if self.nvec < 64:
            i = self.nvec;self.nvec += 1
        else:
            i = self.vec_next;self.vec_next = (self.vec_next+1) % 64
        self.vecs[i].u = u
        for j in range(3):
            self.vecs[i].force[j] = float(f[j]);self.vecs[i].moment[j] = float(m[j])
            force[j] = self.vecs[i].force[j];moment[j] = self.vecs[i].moment[j]
        return 0

    # ------------------------------------------------------------ frame
    cdef void frame(self, const Cond* c, const double* v, const Air* air, const double* stored_omega,
                    const double* commands_in, const History* history, const double* engine_force,
                    const double* engine_moment, const double* wash, const double* momentum, const double* quaternion,
                    bint seed_only, Frame* out) noexcept nogil:
        cdef double scale, norm2, cap, x, y, z, w, b, bias, swirl, sp, tmp
        cdef double commands[3]
        cdef double omega[3]
        cdef double dt = F(self.dt)
        cdef bint inv_tl[3]
        cdef bint inv_tr[3]
        cdef bint inv_n[3]
        cdef double bias_parts[3]
        cdef double tmp5[5]
        cdef double gear_q, airbrake_q, vertical_area_scale, aero_scale, engine_scale, length, limit, mscale
        cdef double raw[3]
        cdef double aero[3]
        cdef double tm[3]
        cdef double gyro[3]
        cdef double rel[7][3]
        cdef double fcomp[8][3]
        cdef int i, j
        inv_tl[0] = True; inv_tl[1] = False; inv_tl[2] = False
        inv_tr[0] = False; inv_tr[1] = False; inv_tr[2] = True
        inv_n[0] = False; inv_n[1] = False; inv_n[2] = False
        # preprocess_angular_rate
        scale = pmax(1.0+F(air.ias_u)*(-1e-5), .8)
        x = stored_omega[0]*scale; y = stored_omega[1]*scale; z = stored_omega[2]*scale
        norm2 = z*z+(x*x+y*y)
        if norm2 > 25.0:
            cap = 5.0/sqrt(norm2)
            x = x*cap; y = y*cap; z = cap*z
        omega[0] = x; omega[1] = y; omega[2] = z
        for i in range(3):
            commands[i] = F(commands_in[i]); out.commands[i] = commands[i]
        # control mixing
        mix(&self.mixers[1], commands, inv_tl, 0., air.density, air.ias_u, air.mach, out.tc[0])
        mix(&self.mixers[1], commands, inv_tr, 0., air.density, air.ias_u, air.mach, out.tc[1])
        mix(&self.mixers[2], commands, inv_n, 0., air.density, air.ias_u, air.mach, out.tc[2])
        if self.mixers[1].wing_aoa == 0.:
            bias_parts[0] = 0.; bias_parts[1] = 0.
        else:
            mix(&self.mixers[1], commands, inv_tl, 0., F(self.altitude), air.ias_u, air.mach, tmp5); bias_parts[0] = tmp5[4]
            mix(&self.mixers[1], commands, inv_tr, 0., F(self.altitude), air.ias_u, air.mach, tmp5); bias_parts[1] = tmp5[4]
        if self.mixers[2].wing_aoa == 0.:
            bias_parts[2] = 0.
        else:
            mix(&self.mixers[2], commands, inv_n, 0., F(self.altitude), air.ias_u, air.mach, tmp5); bias_parts[2] = tmp5[4]
        bias = A(M(A(bias_parts[0], bias_parts[1]), .5), bias_parts[2])
        mix(&self.mixers[0], commands, inv_tl, bias, air.density, air.ias_u, air.mach, out.wc[0])
        mix(&self.mixers[0], commands, inv_n, bias, air.density, air.ias_u, air.mach, out.wc[1])
        # engine wash is zero for direct-thrust engines
        swirl = F(wash[1])*.5 if self.torque_gyro else 0.
        if fabs(swirl) < 1.1920928955078125e-7:
            swirl = 0.
        gear_q = M(<double>(<int>pmin(255., pmax(0., <double>(<int>(M(F(self.gear), 255.)))))), F(1./255))
        airbrake_q = M(<double>(<int>pmin(255., pmax(0., <double>(<int>(M(F(0.), 255.)))))), F(1./255))
        wing_evaluate(&self.g, c.wing, v, omega, self.cog, &out.wc[0][0], air.density, dt, history.aoa,
                      air.beta, air.alpha, c.flap_values[0], gear_q, airbrake_q, commands[1], self.drag,
                      self.inertia[0], 1e6, self.ground_length, self.convert_aoa, history.spin, F(.2),
                      F(self.throttle), commands[2], 1e6, F(wash[0]), swirl, &out.wing)
        if seed_only:
            for i in range(2):
                out.history.aoa[i] = out.wing.coef[i].aoa
                out.history.cl[i] = F(out.wing.coef[i].raw_cl) if c.tp.downwash_type == 2 else history.cl[i]
            out.history.body[0] = F(air.alpha); out.history.body[1] = F(air.beta)
            out.history.spin = out.wing.post.spin
            return
        vertical_area_scale = 1. if self.g.use_spin_loss else out.wing.post.vertical_area_scale
        tmp5[0] = out.wing.coef[0].raw_cl; tmp5[1] = out.wing.coef[1].raw_cl
        secondary_model(&c.tp, c.flap_values[2], out.wing.post.vertical_control_scale, vertical_area_scale,
                        self.convert_aoa, v, omega, self.cog, air.density, dt, air.alpha, air.beta,
                        history.body, history.cl, wash[0], wash[1], self.torque_gyro, &out.wing.base[0][0], tmp5,
                        &out.tc[0][0], &out.tail)
        # component forces: left_wing right_wing left_hstab right_hstab vstab fuselage chute parasite
        for j in range(3):
            fcomp[0][j] = out.wing.forces[0][j]; fcomp[1][j] = out.wing.forces[1][j]
            fcomp[2][j] = out.tail.forces[0][j]; fcomp[3][j] = out.tail.forces[1][j]
            fcomp[4][j] = out.tail.forces[2][j]; fcomp[5][j] = out.tail.forces[3][j]
            fcomp[6][j] = 0.
            out.component_points[0][j] = out.wing.points[0][j]; out.component_points[1][j] = out.wing.points[1][j]
            out.component_points[2][j] = out.tail.points[0][j]; out.component_points[3][j] = out.tail.points[1][j]
            out.component_points[4][j] = out.tail.points[2][j]; out.component_points[5][j] = out.tail.points[3][j]
            out.component_points[6][j] = 0.
        parasite_force(v, air.density, self.cockpit_door_cd, fcomp[7])
        for i in range(8):
            for j in range(3):
                out.component_forces[i][j] = fcomp[i][j]
                fcomp[i][j] = F(fcomp[i][j])
        # assemble_force
        raw[0] = A(A(A(fcomp[4][0], A(fcomp[3][0], A(fcomp[2][0], A(A(fcomp[0][0], fcomp[6][0]), fcomp[1][0])))), fcomp[5][0]), fcomp[7][0])
        raw[1] = A(A(A(A(A(A(fcomp[0][1], fcomp[6][1]), fcomp[1][1]), fcomp[3][1]), A(fcomp[4][1], fcomp[2][1])), fcomp[7][1]), fcomp[5][1])
        raw[2] = A(A(A(fcomp[5][2], A(fcomp[7][2], A(fcomp[1][2], fcomp[0][2]))), fcomp[6][2]), fcomp[4][2])
        for i in range(7):
            for j in range(3):
                rel[i][j] = S(F(out.component_points[i][j]), F(self.cog[j]))
        assemble_moment(fcomp, rel, out.raw_moment)
        for j in range(3):
            out.raw_force[j] = raw[j]
        out.helper[0] = roll_leveling(air.alpha, v[0], quaternion, self.mass) if self.roll_leveling else 0.
        out.helper[1] = 0.; out.helper[2] = 0.
        # limit_aerodynamic_force (body_dynamics)
        length = sqrt((raw[2]*raw[2]+raw[0]*raw[0])+raw[1]*raw[1])
        limit = M(M(F(self.G), F(self.mass)), 50)
        aero_scale = limit/length if length > limit else 1.
        for j in range(3):
            aero[j] = aero_scale*raw[j]
        engine_scale = A(M(S(F(1.), 1), F(self.ext_thrust_base)), 1)
        for j in range(3):
            out.engine_force[j] = engine_force[j]; out.engine_moment[j] = engine_moment[j]
            out.force[j] = (aero[j]+F(0.))+engine_force[j]*F(engine_scale)
            tm[j] = ((out.raw_moment[j]+out.helper[j])+F(0.))+engine_moment[j]
        if self.torque_gyro:
            sp = out.wing.post.gyroscopic_scale
            gyro[0] = sp*(omega[2]*momentum[1]-momentum[2]*omega[1])
            gyro[1] = sp*(omega[0]*momentum[2]-momentum[0]*omega[2])
            gyro[2] = sp*(momentum[0]*omega[1]-omega[0]*momentum[1])
        else:
            gyro[0] = gyro[1] = gyro[2] = 0.
        for j in range(3):
            tm[j] = tm[j]+gyro[j]
        length = sqrt((tm[0]*tm[0]+tm[1]*tm[1])+tm[2]*tm[2])
        limit = M(M(F(self.G), self.mass), 150)
        mscale = limit/length if length > limit else 1.0
        for j in range(3):
            out.moment[j] = mscale*tm[j]
            out.omega[j] = omega[j]
        for i in range(2):
            out.history.aoa[i] = out.wing.coef[i].aoa
            out.history.cl[i] = out.tail.previous_cl[i]
            out.history.body[i] = out.tail.previous_angles[i]
        out.history.spin = out.wing.post.spin

    # ------------------------------------------------------------ operating point
    cdef int evaluate(self, double speed, double load, const double* x, Eval* e) except -1:
        cdef double vel[3]
        cdef double ranges[6]
        cdef double commands[3]
        cdef double ef[3]
        cdef double em[3]
        cdef double acc[3]
        cdef double rate[3]
        cdef double required[3]
        cdef double fe[3]
        cdef double tangential, lateral_acceleration, a, b, num, sm
        cdef int i, ci
        cdef History h
        cdef Frame seed
        cdef Allocation canonical
        cdef bint same
        cdef double zero3[3]
        zero3[0] = zero3[1] = zero3[2] = 0.
        self.evaluations += 1
        e.completed = False
        e.speed = speed; e.load = load
        for i in range(7):e.x[i] = x[i]
        turn_quaternion(x[0], x[1], e.geom.quaternion)
        world_to_body_air(e.geom.quaternion, speed, vel)
        for i in range(3):e.velocity[i] = vel[i]
        air_cache(vel, self.altitude, &e.air)
        turn_geometry(x[0], x[1], speed, load, self.dt, e.air.ias_u, self.G, &e.geom)
        ci = self.condition(e.air.mach);e.cond = ci
        authority_ranges(&self.primary, e.air.ias_u, ranges)
        command_allocation(&self.primary, &x[2], ranges, True, 1., zero3, &canonical)
        if canonical.reachable:
            for i in range(3):commands[i] = canonical.delivered[i]
        else:
            for i in range(3):commands[i] = F(x[2+i])
        same = True
        for i in range(3):
            if commands[i] != canonical.commands[i]:same = False
        if self.cfg_default_trim and same:
            e.alloc = canonical
        else:
            command_allocation(&self.primary, commands, ranges, self.optimized, self.trim_limit, self.fixed_trim, &e.alloc)
        cdef double wash[2]
        cdef double momentum[3]
        if self.has_phase:
            for i in range(3):
                ef[i] = self.cur_phase[i];em[i] = self.cur_phase[3+i];momentum[i] = self.cur_phase[6+i]
            wash[0] = self.cur_phase[9];wash[1] = self.cur_phase[10]
        else:
            if self.prop:raise ValueError('propeller evaluation requires a propulsion sample')
            self.vectors(vel[0], ef, em)
            momentum[0] = momentum[1] = momentum[2] = 0.;wash[0] = wash[1] = 0.
        h.aoa[0] = x[5];h.aoa[1] = x[6];h.cl[0] = 0.;h.cl[1] = 0.
        h.body[0] = e.air.alpha;h.body[1] = e.air.beta;h.spin = 0.
        self.frame(&self.conds[ci], vel, &e.air, e.geom.omega, e.alloc.commands, &h, ef, em, wash, momentum, e.geom.quaternion, True, &seed)
        h.cl[0] = seed.history.cl[0];h.cl[1] = seed.history.cl[1]
        h.body[0] = seed.history.body[0];h.body[1] = seed.history.body[1]
        e.cycle_input = h
        self.frame(&self.conds[ci], vel, &e.air, e.geom.omega, e.alloc.commands, &h, ef, em, wash, momentum, e.geom.quaternion, False, &e.frame)
        # history error (cycle input vs new history)
        e.history_error = 0.
        for i in range(2):
            e.history_error = pmax(e.history_error, fabs(e.frame.history.aoa[i]-h.aoa[i])) if i else fabs(e.frame.history.aoa[0]-h.aoa[0])
        for i in range(2):e.history_error = pmax(e.history_error, fabs(e.frame.history.cl[i]-h.cl[i]))
        for i in range(2):e.history_error = pmax(e.history_error, fabs(e.frame.history.body[i]-h.body[i]))
        e.history_error = pmax(e.history_error, fabs(e.frame.history.spin-h.spin))
        # closure (em_numeric_core)
        cdef double ix = self.inertia[0], iy = self.inertia[1], iz = self.inertia[2]
        cdef double wx = e.frame.omega[0], wy = e.frame.omega[1], wz = e.frame.omega[2]
        cdef double tx = 0.+e.frame.moment[0], ty = 0.+e.frame.moment[1], tz = 0.+e.frame.moment[2]
        num = (wy*wz)*(iz-iy)+tx;acc[0] = num/ix if fabs(ix) > F_TINY else 0.
        num = (wz*wx)*(ix-iz)+ty;acc[1] = num/iy if fabs(iy) > F_TINY else 0.
        num = (wy*wx)*(iy-ix)+tz;acc[2] = num/iz if fabs(iz) > F_TINY else 0.
        for i in range(3):rate[i] = (e.frame.omega[i]+acc[i]*self.dt-e.geom.omega[i])/self.dt
        tangential = fma(e.frame.force[2], e.geom.forward[2], fma(e.frame.force[1], e.geom.forward[1], e.frame.force[0]*e.geom.forward[0]))/self.mass
        lateral_acceleration = (speed/self.dt+tangential)*tan(e.geom.turn_rate*self.dt)
        for i in range(3):
            required[i] = self.weight*e.geom.up[i]+(self.mass*lateral_acceleration)*e.geom.lateral[i]
            fe[i] = (e.frame.force[i]-required[i])/self.weight
        e.residual[0] = fma(fe[2], e.geom.normal[2], fma(fe[1], e.geom.normal[1], fe[0]*e.geom.normal[0]))
        e.residual[1] = fma(fe[2], e.geom.side[2], fma(fe[1], e.geom.side[1], fe[0]*e.geom.side[0]))
        for i in range(3):
            e.rate_residual[i] = rate[i]
            e.residual[2+i] = rate[i]/.1
        e.force_error_g = pmax(fabs(e.residual[0]), fabs(e.residual[1]))
        cdef const Cond* c = &self.conds[ci]
        a = pmax(e.frame.history.aoa[0], e.frame.history.aoa[1])
        b = pmin(e.frame.history.aoa[0], e.frame.history.aoa[1])
        e.stall_margin = c.wing_raw.aoaCritH-a
        e.negative_stall_margin = b-c.wing_raw.aoaCritL
        for i in range(2):
            e.wing_ratios[i] = pmax(e.frame.component_forces[i][1]/self.strength_force[0],
                                    e.frame.component_forces[i][1]/self.strength_force[1])
            e.angle_residual[i] = x[5+i]-e.frame.history.aoa[i]
        return 0

    def residual(self, double speed_mps, double load, x):
        """JointProblem.residual layout: 5 aircraft rows, then history rows / 10."""
        cdef double q[7]
        cdef Eval e
        for i in range(7):q[i] = float(x[i])
        self.evaluate(speed_mps, load, q, &e)
        out = np.empty(7)
        for i in range(5):out[i] = e.residual[i]
        out[5] = e.angle_residual[0]/10.;out[6] = e.angle_residual[1]/10.
        return out


    def record(self, double speed_mps, double load, x, bint search=True, bint complete=False):
        """Python value record (operating_point layout) of one native evaluation."""
        cdef double q[7]
        cdef Eval e
        for i in range(7):q[i] = float(x[i])
        self.evaluate(speed_mps, load, q, &e)
        return value_record(self, &e, search, complete)

    def set_propulsion(self, propulsion):
        """Supply the propulsion sample for following evaluations (None: direct-thrust vectors)."""
        if propulsion is None:
            self.has_phase = False;return 0
        phases = propulsion.get('cycle_samples')
        if phases:
            row = phases[0]
            frames = len(phases)
        else:
            row = list(propulsion['force'])+list(propulsion['moment'])+list(propulsion['angular_momentum'])+list(propulsion['wash'])
            frames = 0
        if len(row) != 11:raise ValueError('propulsion phase layout')
        for i in range(11):self.cur_phase[i] = float(row[i])
        self.has_phase = True
        return frames

    def value(self, double speed_mps, double load, x, angles, bint search, propulsion=None):
        """em_solver.operating_point(speed, load, x, stationary_angles=angles, _search=search,
        propulsion_sample=propulsion) record."""
        cdef double q[7]
        cdef Eval e
        for i in range(5):q[i] = float(x[i])
        q[5] = float(angles[0]);q[6] = float(angles[1])
        frames = self.set_propulsion(propulsion)
        try:
            self.evaluate(speed_mps, load, q, &e)
        finally:
            self.has_phase = False
        out = value_record(self, &e, search, not search)
        out['equilibrium_coordinates'] = list(x)
        if propulsion is not None:
            out['propulsion'] = propulsion;out['propulsion_phase_frames'] = frames
        return out
    def bench(self, rows, int repeats=20):
        """Seconds per native evaluation over (speed_mps, load, x7[, phase11]) rows (C loop, no Python per call)."""
        import time
        cdef int n = len(rows), i, r, j
        cdef double* data = <double*>malloc(n*20*sizeof(double))
        cdef bint* has = <bint*>malloc(n*sizeof(bint))
        cdef Eval e
        for i, row in enumerate(rows):
            data[20*i] = row[0];data[20*i+1] = row[1]
            for j in range(7):data[20*i+2+j] = row[2][j]
            has[i] = len(row) > 3 and row[3] is not None
            if has[i]:
                for j in range(11):data[20*i+9+j] = row[3][j]
        for i in range(n):
            # warm the condition (Mach) cache outside the timed loop
            self.has_phase = has[i]
            if has[i]:
                for j in range(11):self.cur_phase[j] = data[20*i+9+j]
            self.evaluate(data[20*i], data[20*i+1], &data[20*i+2], &e)
        t0 = time.perf_counter()
        for r in range(repeats):
            for i in range(n):
                self.has_phase = has[i]
                if has[i]:
                    for j in range(11):self.cur_phase[j] = data[20*i+9+j]
                self.evaluate(data[20*i], data[20*i+1], &data[20*i+2], &e)
        t1 = time.perf_counter()
        self.has_phase = False
        free(data);free(has)
        return (t1-t0)/(n*repeats)

    def detail(self, double speed_mps, double load, x):
        cdef double q[7]
        cdef Eval e
        for i in range(7):q[i] = float(x[i])
        self.evaluate(speed_mps, load, q, &e)
        return dict(residual=[e.residual[i] for i in range(5)], rate_residual=[e.rate_residual[i] for i in range(3)],
                    angle_residual=[e.angle_residual[0], e.angle_residual[1]], force_error_g=e.force_error_g,
                    history_error=e.history_error, stall_margin=e.stall_margin,
                    negative_stall_margin=e.negative_stall_margin, wing_ratios=[e.wing_ratios[0], e.wing_ratios[1]],
                    force=[e.frame.force[i] for i in range(3)], moment=[e.frame.moment[i] for i in range(3)],
                    raw_force=[e.frame.raw_force[i] for i in range(3)], raw_moment=[e.frame.raw_moment[i] for i in range(3)],
                    commands=[e.alloc.commands[i] for i in range(3)], delivered=[e.alloc.delivered[i] for i in range(3)],
                    sticks=[e.alloc.sticks[i] for i in range(3)], trim=[e.alloc.trim[i] for i in range(3)],
                    bounds=[[e.alloc.bounds[i][0], e.alloc.bounds[i][1]] for i in range(3)], reachable=e.alloc.reachable,
                    air=dict(alpha=e.air.alpha, beta=e.air.beta, mach=e.air.mach, ias_u=e.air.ias_u, density=e.air.density, tas=e.air.tas),
                    omega_for_flow=[e.frame.omega[i] for i in range(3)], geometry_omega=[e.geom.omega[i] for i in range(3)],
                    wing_aoa=[e.frame.wing.coef[i].aoa for i in range(2)], wing_cl=[e.frame.wing.coef[i].raw_cl for i in range(2)],
                    tail_angles=[e.frame.tail.angles[i] for i in range(4)], blend=e.frame.wing.blend,
                    new_history=dict(wing_aoa=[e.frame.history.aoa[0], e.frame.history.aoa[1]],
                                     wing_cl=[e.frame.history.cl[0], e.frame.history.cl[1]],
                                     body_angles=[e.frame.history.body[0], e.frame.history.body[1]], spin=e.frame.history.spin),
                    component_forces=[[e.frame.component_forces[i][j] for j in range(3)] for i in range(8)],
                    component_points=[[e.frame.component_points[i][j] for j in range(3)] for i in range(7)])


cdef void parasite_force(const double* body_velocity, double density, double cockpit_door_cd, double* out) noexcept nogil:
    cdef double x = F(body_velocity[0]), y = F(body_velocity[1]), z = F(body_velocity[2])
    cdef double norm2, speed, reciprocal, q, door, area, xy_scale
    cdef double d[3]
    norm2 = A(M(z, z), A(M(x, x), M(y, y)))
    speed = F(sqrt(norm2))
    if speed > 1e-9:
        reciprocal = F(1./speed)
        d[0] = M(x, reciprocal); d[1] = M(y, reciprocal); d[2] = M(z, reciprocal)
    else:
        d[0] = d[1] = d[2] = 0.
    q = M(norm2, M(F(density), .5))
    door = F(0.)
    area = A(M(door, F(cockpit_door_cd)), F(0.))
    xy_scale = M(area, -q)
    out[0] = M(xy_scale, d[0]); out[1] = M(xy_scale, d[1]); out[2] = M(M(-q, d[2]), area)


cdef inline double term(double f[8][3], double r[7][3], int n, int fi, int ri) noexcept nogil:
    return M(f[n][fi], r[n][ri])


cdef double paired(double f[8][3], double r[7][3], int fi, int ri) noexcept nogil:
    # indices: 0 left_wing 1 right_wing 2 left_hstab 3 right_hstab 4 vstab 5 fuselage 6 chute
    cdef double wings, tails, main, extra
    wings = A(term(f, r, 1, fi, ri), term(f, r, 0, fi, ri))
    tails = A(term(f, r, 2, fi, ri), term(f, r, 3, fi, ri))
    main = A(wings, tails)
    extra = A(term(f, r, 5, fi, ri), term(f, r, 4, fi, ri))
    return A(A(main, extra), term(f, r, 6, fi, ri))


cdef void assemble_moment(double f[8][3], double r[7][3], double* out) noexcept nogil:
    cdef double pos_w, pos_h, pos, neg_w, neg_h, neg
    pos_w = A(term(f, r, 1, 1, 2), term(f, r, 0, 1, 2))
    pos_h = A(term(f, r, 3, 1, 2), term(f, r, 2, 1, 2))
    pos = A(A(term(f, r, 6, 1, 2), A(term(f, r, 5, 1, 2), term(f, r, 4, 1, 2))), A(pos_h, pos_w))
    neg_w = A(term(f, r, 1, 2, 1), term(f, r, 0, 2, 1))
    neg_h = A(term(f, r, 3, 2, 1), term(f, r, 2, 2, 1))
    neg = A(term(f, r, 6, 2, 1), A(A(term(f, r, 5, 2, 1), term(f, r, 4, 2, 1)), A(neg_h, neg_w)))
    out[0] = S(pos, neg)
    out[1] = S(paired(f, r, 2, 0), paired(f, r, 0, 2))
    out[2] = S(paired(f, r, 0, 1), paired(f, r, 1, 0))


cdef double roll_leveling(double alpha, double velocity_x, const double* quaternion, double mass) noexcept nogil:
    cdef double x, y, z, w, test, roll, numerator, denominator, angle, gain, command, weight
    if fabs(F(alpha)) >= 12.:
        return 0.
    x = F(quaternion[0]); y = F(quaternion[1]); z = F(quaternion[2]); w = F(quaternion[3])
    test = A(M(w, z), M(y, x))
    if test >= F(.49999) or test <= F(-.49999):
        roll = 0.
    else:
        numerator = S(M(w, x), M(z, y)); denominator = A(M(-z, z), S(.5, M(x, x)))
        roll = F(atan2(numerator, denominator)) if (numerator != 0. or denominator != 0.) else 0.
    angle = pmin(30., pmax(-30., M(roll, F(-57.2957763671875))))
    gain = pmin(velocity_x-50., 50.)*F(.003)
    command = F(angle*gain)
    weight = M(F(9.8100004196167), F(mass))
    return M(M(F(-.01), weight), command)
include "record.pxi"
include "problem.pxi"
include "instr.pxi"
include "finish.pxi"
include "march.pxi"
include "column.pxi"
include "tracer.pxi"
include "curves.pxi"
include "point.pxi"
include "recovery.pxi"
