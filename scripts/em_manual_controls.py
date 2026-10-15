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


def profile(fm, propulsion):
    from jet_catalog import engines
    controls = [e.get('Controls', {}) for _, e in engines(fm)
                if e.get('Main', {}).get('Type') not in ('Jet', 'Rocket')]
    props = [p for p in propulsion['propellers'] if p.get('manual')]
    kinds = {'RPM governor' if manual_rpm_governor(p['properties']) else 'Blade angle' for p in props}
    common = bool(controls) and all(flag(c.get('hasCommonRadiator')) for c in controls)
    return dict(propeller_pitch=bool(props), propeller_auto=bool(props) and all(p.get('automatic') for p in props),
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
    manual = config.get('manual_engine_controls', False) and not config.get('propeller_pitch_auto', True)
    command = int(math.floor(config.get('propeller_pitch_percent', 50.) * 255. / 100. + .5))
    return dict(commands=[command if manual and p['manual'] else 255 for p in properties['propellers']],
                automatic=[not (manual and p['manual']) for p in properties['propellers']])


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
