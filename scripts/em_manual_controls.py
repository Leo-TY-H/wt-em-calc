"""Aircraft-specific manual controls and steady axial-flow RPM boundaries."""
import math


def flag(value):
    # BLK duplicate values are represented by lists; the first is effective.
    return value[0] is True if isinstance(value, list) and value else value is True


def manual_rpm_governor(properties):
    """Types 1/2 retain RPM regulation with a manually selected target.

    Types 6/8 instead deliver blade angle when automatic control is disabled.
    The availability of an automatic-control toggle is a separate capability.
    """
    return properties['governor'] in (1, 2)


def governs(properties):
    """Whether the propeller's governor has a blade to move: it has one and its two pitch stops differ.

    With equal stops (Ba.65 K.14 L: 24 degrees both) the blade never moves, so the propeller is a fixed-pitch one and
    its shaft speed is free. Solved as a governed one it has no state the solver can reach: nothing changes over the
    governor's own range, and its states lie on both sides of it.
    """
    return bool(properties['governor']) and properties['pitch_max'] > properties['pitch_min']


def auto_pitch_available(propeller):
    """Whether the game can run this propeller automatically under manual engine controls.

    This is propeller type +0x369, which the FM loaders derive from the governor type (aces.exe
    0x143edb2a6 for PropellerType/Propeller sections, 0x143edcd34 for Engine.Propellor):
    HasAutoPitchControl/AllowAutoProp (+0x35c) only for RPM governors 1/2, always for blade-angle
    governors 5/8 and types above 10, never for 0, 3, 4, 6, 7, 9, 10. The per-frame control applier
    (0x1430b20a0) runs a propeller automatically under manual engine controls when the engine's auto
    toggle (on at spawn, 0x142fe1f84) is set and +0x369 is set; +0x35c is never read at runtime.
    So the Fw 190 (governor 8, HasAutoPitchControl false) keeps automatic pitch, as in the game.
    """
    report = propeller['properties'].get('pitch_command_report')
    if report is not None:
        return bool(report)
    governor = propeller['governor']
    return bool(propeller.get('automatic')) if governor in (1, 2) else governor in (5, 8) or governor > 10


def manual_pitch(propeller):
    """Whether the pilot's propeller lever drives this propeller (HasManualPitchControl/IsControllable).

    Governor 0 has a fixed blade angle that no lever moves, so it offers no control."""
    return bool(propeller.get('manual')) and propeller['governor'] != 0


def profile(fm, propulsion):
    from jet_catalog import engines
    controls = [e.get('Controls', {}) for _, e in engines(fm)
                if e.get('Main', {}).get('Type') not in ('Jet', 'Rocket')]
    props = [p for p in propulsion['propellers'] if manual_pitch(p)]
    kinds = {'RPM governor' if manual_rpm_governor(p['properties']) else 'Blade angle' for p in props}
    common = bool(controls) and all(flag(c.get('hasCommonRadiator')) for c in controls)
    # One Automatic option per aircraft, as the game's auto-pitch key toggles every selected engine;
    # each propeller then follows it only where the game allows automatic pitch (propeller_commands).
    return dict(propeller_pitch=bool(props), propeller_auto=any(auto_pitch_available(p) for p in props),
                pitch_kind=' / '.join(sorted(kinds)),
                radiator=any(flag(c.get('hasRadiatorControl')) for c in controls),
                oil_radiator=any(flag(c.get('hasOilRadiatorControl')) and
                                 not flag(c.get('hasCommonRadiator')) for c in controls),
                common_radiator=common)


def radiator_positions(config, capability):
    if not config.get('manual_engine_controls'):
        return 0., 0.
    water = config.get('radiator_percent', 0.) / 100. if capability.get('radiator') else 0.
    oil = (water if capability.get('common_radiator') else
           config.get('oil_radiator_percent', 0.) / 100. if capability.get('oil_radiator') else 0.)
    return water, oil


def propeller_commands(properties, config):
    """Per-propeller controls as the game's applier (0x1430b20a0) sets them each frame.

    Without manual engine controls every propeller is automatic. With them, a propeller is automatic
    when the Automatic option (the game's per-engine auto toggle, on by default) is on and the game
    allows automatic pitch for it; otherwise the lever command drives it. Propellers without a lever
    stay automatic."""
    mec = config.get('manual_engine_controls', False)
    toggle = config.get('propeller_pitch_auto', True)
    command = int(math.floor(config.get('propeller_pitch_percent', 50.) * 255. / 100. + .5))
    automatic = [not (mec and manual_pitch(p) and not (toggle and auto_pitch_available(p)))
                 for p in properties['propellers']]
    return dict(commands=[255 if auto else command for auto in automatic], automatic=automatic)


def rpm_boundary(engine, minimum, maximum, cancelled=None):
    """Locate the first over-rev onset, then stop the speed scan.

    This is a speed reference at zero body incidence/rate, not a universal
    turning-flight limit. The EM points retain their own actual RPM diagnostics.
    """
    limits = [e['properties']['omega_limit'] * 30. / math.pi for e in engine.properties['engines']]
    linked = sorted({l['index'] for t in engine.properties['transmissions'] for l in t['engines']})
    samples = {}
    def sample(speed):
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
        if speed not in samples:
            r = engine.condition((speed / 3.6, 0., 0.), (0., 0., 0.), canonical=True)
            if not r['converged']:
                r = engine.condition((speed / 3.6, 0., 0.), (0., 0., 0.), canonical=False)
            samples[speed] = ([e['omega'] * 30. / math.pi for e in r['state']['engines']]
                              if r['converged'] and not r['stopped_shafts'] else None)
        return samples[speed]
    if maximum < minimum:
        return dict(crossings=[],reference='Steady axial flow; zero body incidence and rotation',
                    speed_range_kmh=[minimum,maximum],scan_step_kmh=0.,unresolved_intervals_kmh=[],
                    over_rev_at_minimum=None,stop_speed_kmh=None,stop_reason=None,samples=[])
    count = max(1, int(math.ceil((maximum - minimum) / 10.)))
    speeds = [minimum + (maximum - minimum) * i / count for i in range(count + 1)]
    crossings = []; unresolved = []; stop_speed=None; stop_reason=None
    previous = sample(speeds[0])
    over_at_minimum = (any(previous[i] > limits[i] * (1+1e-6) for i in linked)
                       if previous is not None else None)
    if over_at_minimum:
        stop_speed=minimum;stop_reason='Over-rev'
    elif previous is None:
        stop_speed=minimum;stop_reason='Unresolved RPM limit'
        unresolved.append([minimum,minimum])
    for a, b in zip(speeds, speeds[1:]):
        if stop_speed is not None:break
        current = sample(b)
        if current is None:
            unresolved.append([a, b])
            stop_speed=a;stop_reason='Unresolved RPM limit'
        else:
            for i in linked:
                threshold=limits[i]*(1+1e-6)
                fa, fb = previous[i] - threshold, current[i] - threshold
                if abs(fa) < .01 and abs(fb) < .01:continue
                if not fa <= 0 < fb:continue
                lo, hi = a, b
                failed = False
                while hi - lo > .1:
                    mid = (lo + hi) / 2.; value = sample(mid)
                    if value is None:
                        unresolved.append([lo, hi]); failed = True
                        stop_speed=lo if stop_speed is None else min(stop_speed,lo)
                        stop_reason='Unresolved RPM limit';break
                    fm = value[i] - threshold
                    if fa * fm <= 0:hi = mid
                    else:lo = mid; fa = fm
                if not failed:
                    speed = (lo + hi) / 2.
                    crossings.append(dict(speed_kmh=speed, engine_index=i, rpm_limit=limits[i],
                                          over_rev_side='higher speed', bracket_kmh=[lo, hi]))
                    if stop_speed is None or lo<stop_speed:
                        stop_speed=lo;stop_reason='Over-rev'
        previous = current
    if crossings:
        first_crossing=min(crossings,key=lambda c:c['speed_kmh'])
        crossings=[c for c in crossings if abs(c['speed_kmh']-first_crossing['speed_kmh'])<.1]
    return dict(crossings=crossings, reference='Steady axial flow; zero body incidence and rotation',
                speed_range_kmh=[minimum, maximum], scan_step_kmh=(maximum-minimum)/count,
                unresolved_intervals_kmh=unresolved,
                over_rev_at_minimum=over_at_minimum,stop_speed_kmh=stop_speed,stop_reason=stop_reason,
                samples=[dict(speed_kmh=v, engine_rpm=rpms) for v, rpms in sorted(samples.items())])
