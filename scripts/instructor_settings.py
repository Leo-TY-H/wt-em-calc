"""Recovered Instructor-related aircraft settings; research integration only."""
from component_assembly import f32,mul


def payload_scales(snapshot_flags,base_multiplier,instructor_multiplier):
    """101a3f8c0..3f8fb, before 101a3db30 mass update.

    The snapshot's autotrim bit selects an additional multiplier. Both CG and
    inertia inputs receive the same result; mass itself remains unscaled.
    Their downstream use is defined by mass_model, including its CG option.
    """
    scale=mul(instructor_multiplier if snapshot_flags&1 else 1.,base_multiplier)
    return [scale,scale]


def trim_retained(manual_trim,ground_trim,snapshot_flags,autotrim_allowed):
    """101a4e670..4e7dd, availability gates before engine command delivery.

    Public order roll,pitch,yaw. Auto trim bypasses unavailable pitch and yaw
    trim, but never the roll availability gate. Ground-trim availability also
    retains a preset even in flight; these flags are not a ground-contact test.
    """
    auto=bool(autotrim_allowed and snapshot_flags&1)
    return [bool(manual_trim[i] or ground_trim[i] or (i!=0 and auto)) for i in range(3)]


def deliver_keyboard_commands(properties,snapshot,state,ranges,dt,*,autotrim=True,
                              autotrim_allowed=True,ground_trim=(False,False,False)):
    """Same selected jet consumer with independently recovered trim gates."""
    from control_snapshots import deliver_selected_jet_commands
    p=dict(properties,trim_available=trim_retained(properties['trim_available'],ground_trim,int(autotrim),autotrim_allowed))
    return deliver_selected_jet_commands(p,snapshot,state,ranges,dt)


def restored_wing_normalization(areas,health=((1.,1.,1.),(1.,1.,1.))):
    """101a34302..34429: prediction-restore refresh of wing area and CL scales.

    First three area entries on each side exclude the aileron. Even intact
    area sums must preserve grouping instead of using the authored total.
    """
    from component_assembly import add
    from instructor_protection import divide
    totals=[];ratios=[]
    for a,h in zip(areas,health):
        nominal=add(add(a[1],a[0]),a[2])
        actual=add(mul(a[2],h[2]),add(mul(a[1],h[1]),mul(a[0],h[0])))
        totals.append(actual);ratios.append(mul(actual,divide(1.,nominal)))
    return {0x8438:add(totals[1],totals[0]),0x843c:ratios[0],0x8440:ratios[1]}
