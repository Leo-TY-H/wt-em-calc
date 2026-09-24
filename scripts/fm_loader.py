"""Statically traced legacy BLK adapters, preserving original field provenance.

Native loaders: wing1019e0a10, fuselage1019e2670, hstab1019e2b30,
vstab1019e3080; legacy controls1019e47a0/4ba0/5030. No aircraft is used as a
template for another aircraft. This module converts the two source layouts into
the same property-loader input schema; it does not replace runtime equations.
"""
import copy
from component_assembly import f32,add,mul

POLAR_DEFAULTS=dict(lineClCoeff=.85,AfterCritParabAngle=5.,AfterCritDeclineCoeff=.007,
    AfterCritMaxDistanceAngle=30.,CxAfterCoeff=.01,ClAfterCritLow=-1.09,ClAfterCritHigh=1.09,
    Cl0=0.,alphaCritHigh=16.,alphaCritLow=-16.,ClCritHigh=1.,ClCritLow=-1.,CdMin=.02,
    MachFactor=0,CombinedCl=True)
MACH_DEFAULTS=[(.6,1.,7.,-5.2,1.),(.65,.97,6.7,-3.7,1.),(.3,1.,.32,-.44,.25),
               (.3,1.,.4,-.2,.25),(.6,1.5,2.,1.1,5.),(0.,0.,0.,0.,0.),(0.,1.,1.,0.,1.)]
for i,row in enumerate(MACH_DEFAULTS,1):
    for key,value in zip(['MachCrit','MachMax','MultMachMax','MultLineCoeff','MultLimit'],row):POLAR_DEFAULTS[key+str(i)]=value


def polar(config,cm=0.,defaults=None):
    p=dict(POLAR_DEFAULTS);p.update(defaults or {});p.update(config)
    # Datamine preserves repeated scalar parameters as arrays. Typed native
    # getters select the first occurrence (not the final exported value).
    for key in ['OswaldsEfficiencyNumber',*POLAR_DEFAULTS]:
        if key in p and isinstance(p[key],list):p[key]=p[key][0]
    for key,value in list(p.items()):
        if key.startswith('ClToCmByMach') and isinstance(value,list) and value and isinstance(value[0],list):p[key]=value[0]
    if not any(k.startswith('ClToCmByMach') for k in p):p['ClToCmByMach']=[0.,cm]
    return p


def normalize(source):
    fm=copy.deepcopy(source);ad=fm['Aerodynamics'];focus=fm.get('Focus',{});areas=fm.get('Areas',{})
    for key in ['GearCd','GearCentralCd','AirbrakeCd','RadiatorCd','OilRadiatorCd','BombBayCd','FuseCd','CockpitDoorCd']:
        if isinstance(ad.get(key),list):ad[key]=ad[key][0]
    # Named typed getters stop at the first repeated boolean. In particular,
    # bool([False,False]) must not invert the Kfir's elevator controls.
    # Independently checked against the original BLK getter and controller.
    for key in ['InvertElevator','AllowStrongControlsRestrictions','ConvertAoa',
                'ConvertAoaAI','RollLeveling','AllowModsToChangeLongidutialBalance']:
        value=fm.get(key)
        if isinstance(value,list):
            if not value or not all(isinstance(x,bool) for x in value):raise ValueError('Invalid boolean '+key)
            fm[key]=value[0]
    if 'WingPlane' not in ad and not any(k.startswith('WingPlaneSweep') for k in ad):
        a={side+part:areas.get('Wing'+side+part,0.) for side in ['Left','Right'] for part in ['In','Mid','Out']}
        a['Aileron']=mul(f32(areas.get('Aileron',0.)),.5)
        arm=dict(Arm=[focus.get('FocusOffset',0.),focus.get('WingVertPos',0.),focus.get('WingMidFocus',2.28)],
                 ClToCmCoeff=ad.get('ClToCmCoeff',.055),SineAosMultiplier=focus.get('SineAOSMultiplier',1.),
                 VFocusMultiplier=focus.get('WingVWingFocusMultiplier',0.),AoaShift=focus.get('AlphaShift',.6),AoaShiftAdd=0.)
        for new,old in [('FlapsShift','FlapsShift'),('AirbrakesShift','AirbrakesShift'),('GearShift','GearFocusShift'),('ElevonShift','ElevonFocusShift')]:
            v=focus.get(old,0.);arm[new]=v if isinstance(v,list) else [v,0.]
        wing=dict(Span=fm.get('Wingspan',10.),SweptAngle=fm.get('SweptWingAngle',0.),TaperRatio=fm.get('WingTaperRatio',2.),
                  Angle=fm.get('WingAngle',0.),VAngle=focus.get('WingV',1.5),Areas=a,Arm=arm,
                  DownwashType=ad.get('DownwashType',1),DownwashCoeff=ad.get('DownwashCoeff',1.),
                  UseSpinLoss=ad.get('UseSpinLoss',False),SpinCdloss=ad.get('SpinCdLoss',-1.),SpinClloss=ad.get('SpinClLoss',-1.),
                  Strength=dict(CritOverload=fm['Mass'].get('WingCritOverload',[-2147440000.,2147440000.]),VNE=fm.get('Vne',300.),MNE=fm.get('VneMach',.8)))
        for i,(key,value) in enumerate([('NoFlaps',0.),('FullFlaps',1.)]):
            if key in ad:wing['FlapsPolar'+str(i)]=dict(polar(dict(ad,**ad[key]),arm['ClToCmCoeff']),Flaps=value)
        if not any(k.startswith('FlapsPolar') for k in wing):raise ValueError('Legacy wing lacks explicit polars')
        ad['WingPlane']=wing
    wing=ad.get('WingPlane',ad.get('WingPlaneSweep0'))
    wingarea=0.
    for key in ['LeftMid','LeftIn','LeftOut','RightIn','RightMid','RightOut']:wingarea=add(wingarea,f32(wing['Areas'].get(key,0.)))
    for name,key,span,angle,area,position,inertia in [
        ('FuselagePlane','Fuselage',wing['Span'],0.,dict(Main=areas.get('Fuselage',wingarea)),focus.get('Fuselage',[-1.3,1.,0.]),0.),
        ('HorStabPlane','Stab',fm.get('StabWidth',3.),fm.get('StabAngle',-2.),dict(Main=areas.get('Stabilizer',0.),Elevator=areas.get('Elevator',0.)),focus.get('LeftStab',[-5.1,0.,1.]),ad.get('StabFlowInertia',0.)),
        ('VerStabPlane','Fin',fm.get('FinHeight',2.),fm.get('KeelAngle',0.),dict(Main=areas.get('Keel',0.),Rudder=areas.get('Rudder',0.)),focus.get('VertStab',[-5.4,1.,0.]),ad.get('VertStabFlowInertia',0.))]:
        if name not in ad:
            if key not in ad:raise ValueError('Legacy '+key+' lacks explicit polar block')
            defaults=dict(lineClCoeff=.00647 if key=='Fuselage' else .085,Cl0=0.,alphaCritHigh=17.,alphaCritLow=-17.,ClCritHigh=.11 if key=='Fuselage' else 1.1,ClCritLow=-.11 if key=='Fuselage' else -1.1,CdMin=0. if key=='Fuselage' else .02)
            # Legacy fresh fuselage polar has zero span/area before its first
            # load;10198ce30 therefore supplies an Oswald default of zero.
            if key=='Fuselage':defaults['OswaldsEfficiencyNumber']=0.
            ad[name]=dict(Span=span,Angle=angle,Areas=area,Arm=position,FlowInertia=inertia,Polar=polar(ad[key],defaults=defaults))
    for component,axis,root,default in [('Ailerons','Roll','Aileron',.4),('Elevator','Pitch','Elevator',.5),('Rudder','Yaw','Rudder',.3)]:
        if component not in ad:
            c={k:[0.,0.] for k in ['AnglesRoll','AnglesPitch','AnglesYaw']}
            c['Angles'+axis]=fm.get(root+'Angles',[30.,30.])
            c.update(Sensitivity=fm.get(root+'Sens',default),SensitivityCl=[fm.get('ElevatorSensCl',0.)]*2 if axis=='Pitch' else [0.,0.],
                     SensitivityCd=fm.get('AileronCd',[.015,.0027]) if axis=='Roll' else [0.,0.],
                     SensitivityWingAoa=fm.get('ElevonPitchAngleSens',fm.get('ElevonPitchAngleMultiplier',0.)) if axis=='Pitch' else 0.)
            for new,old in [('SensitivityMultiplier',root+'SensMultipler'),('ArcadeSensitivityMultiplier','Arcade'+root+'SensMultiplier')]:
                for suffix in ['']+[str(i) for i in range(10)]:
                    if old+suffix in fm:c[new+suffix]=fm[old+suffix]
            ad[component]=c
    ad['VerStabPlane'].setdefault('SlipStreamDistance',focus.get('Rudder',5.72))
    ad['HorStabPlane'].setdefault('ClockWiseAOA',ad.get('clockWiseStabAOA0',False))
    for name,plane in list(ad.items()):
        if name in ['WingPlane','HorStabPlane','VerStabPlane','FuselagePlane'] or name.startswith('WingPlaneSweep'):
            for key,value in list(plane.items()):
                if key=='Polar' or key.startswith('FlapsPolar'):plane[key]=polar(value,plane.get('Arm',{}).get('ClToCmCoeff',.055) if isinstance(plane.get('Arm'),dict) else 0.)
    # Aircraft constructor101a35540 / loader101a35ca0; gameparams values.
    fm.setdefault('InvertElevator',False)  #101a3719a
    #1072085b0 fourth lane -> d20, then101a36db7 converts back to km/h.
    fm.setdefault('AileronEffectiveSpeed',mul(f32(100.),f32(3.6)))
    fm.setdefault('WingWaveMassRel',.25)
    fm.setdefault('WingSpringDampJointMult',[.5,.005])
    # Aircraft loader101a39524..39669: rates default from the prepared
    # control-surface areas at the initial (forward) wing position.
    for key,numerator,area in [('AileronMaxDv',5.,wing['Areas'].get('Aileron',0.)),
        ('ElevatorMaxDv',6.5,ad['HorStabPlane']['Areas'].get('Elevator',0.)),
        ('RudderMaxDv',6.,ad['VerStabPlane']['Areas'].get('Rudder',0.))]:
        if key not in fm:fm[key]=f32(numerator/f32(area)) if abs(f32(area))>f32(4e-19) else 0.
    for key in ['ElevatorsEffectiveSpeed','ElevatorPowerLoss']:
        value=fm.get(key)
        if isinstance(value,list) and value and isinstance(value[0],list):fm[key]=value[0]
    for key in ['AileronEffectiveSpeed','RudderEffectiveSpeed','AileronPowerLoss','RudderPowerLoss',
                'AlphaAileronMin','AlphaElevatorMin','AlphaRudderMin','AileronMaxDv','ElevatorMaxDv','RudderMaxDv']:
        if isinstance(fm.get(key),list):fm[key]=fm[key][0]
    # Repeated fuel-capacity scalars use the first named BLK value.
    for key,value in fm['Mass'].items():
        if key.startswith('MaxFuelMass') and isinstance(value,list):fm['Mass'][key]=value[0]
    parts=fm['Mass'].get('Parts',{})
    for key,value in parts.items():
        if key.endswith('_capacity') and isinstance(value,list):parts[key]=value[0]
    for axis in ['Aileron','Elevator','Rudder']:
        fm['AvailableControls'].setdefault('dv'+axis+'Trim',.125)  # native1071ea9e8
    # Datamine lists can encode duplicate scalar keys; the BLK getter takes
    # the first occurrence, as for the existing duplicate control properties.
    if isinstance(fm['AvailableControls'].get('hasFlapsControl'),list):
        fm['AvailableControls']['hasFlapsControl']=fm['AvailableControls']['hasFlapsControl'][0]
    if 'EngineType0' not in fm and 'Engine0' in fm:fm['EngineType0']=copy.deepcopy(fm['Engine0'])
    return fm
