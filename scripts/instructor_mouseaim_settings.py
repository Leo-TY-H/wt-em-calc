"""MouseAim settings read by original 101a9d850.

Byte offsets are relative to global107d70000. This packs authored settings for
the offline oracle; it does not invent controller properties or run prediction.
Absent settings retain their original initialized values. Unknown names remain
ignored, as in the native named getters (notably angVelFactorMin is NOT the
native getter's angVelFactor0). Original getter/packing parity has a separate
test using the real loader and explicit typed data-provider adapters.
"""
import struct

FIELDS=(
    ('mouseAimDisableControlTreshold',0x80,'f'),('resetAimByElevatorAndRudder',0x84,'B'),
    ('numSteps',0x94,'i'),('maxNumSteps',0x98,'i'),('predictionTime',0x9c,'3f'),
    ('scoreTreshold',0xa8,'f'),('analyticalScore',0xac,'f'),('rotationStopTimeMax',0xb0,'f'),
    ('angDistToRadialAngleAdd',0xb4,'4f'),('rollRateToRadialAngleAdd',0xc4,'f'),
    ('radialAngleAddMax',0xc8,'f'),('angVelFactor0',0xcc,'f'),('rollRateToAngVelFactor',0xd0,'f'),
    ('applyFactorSpeedRange',0xd4,'2f'),('applyFactorRudderSpeedRange',0xdc,'3f'),
    ('disableSolvingForDeadElevator',0xe8,'B'),('disableSolvingForDeadRudder',0xe9,'B'),
    ('reducedFlightModelLoop',0xea,'B'),('elevRuddTreshold',0xec,'f'),
    ('elevRuddLinearSearchIterMax',0xf0,'i'),('elevRuddPosCaching',0xf4,'B'),
    ('elevRuddPosCachingTolerance',0xf8,'f'),('elevRuddDiffTreshold',0xfc,'f'),
    ('sceneCollisionDetectionTickInterval',0x100,'i'),('omegaMaxMultiplier',0x104,'f'),
    ('preciseRotationStopTimeStepMult',0x108,'f'),('preciseRotationStopTickMult',0x10c,'i'))

PROPERTY_FIELDS=(
    ('forceAdvanced',0x110,'B'),('preciseRotStopDetection',0x111,'B'),
    ('localDirYawPitchRotYZMax0',0x114,'f'),('autoBankLevelMultToLocalDirYawPitchRotYZMax',0x118,'f'),
    ('rollAndPullUpWishDirYMin',0x11c,'f'),('rollandPullUpWishDirMin',0x120,'f'),
    ('allowedRollRateCoeff',0x124,'4f'),('rollTimeMult',0x134,'f'),('rollPidKpMult',0x138,'f'),
    ('dirXZFactorPitch0',0x13c,'4f'),('dirXZFactorPitchMin',0x14c,'f'),
    ('dirXZFactorYaw0',0x150,'4f'),('dirXZFactorYawMin',0x160,'f'),
    ('dirXZToRollRateLim',0x164,'2f'),('rollRateLimToDirXZFactorMin',0x16c,'2f'),
    ('rollRateLimToDirXZFactorMax',0x174,'2f'))


def property_fields(properties):
    props=dict(properties)
    # Native stores a zero default for this field when the child block exists.
    props.setdefault('dirXZToRollRateLim',[0.,0.])
    legacy=props.get('rollRateLimToDirXZFactor')
    if legacy is not None:
        props.setdefault('rollRateLimToDirXZFactorMin',legacy)
        props.setdefault('rollRateLimToDirXZFactorMax',legacy)
    return _pack(PROPERTY_FIELDS,props)


def aircraft_property_fields(properties):
    """101a977e0: override an initialized object's explicitly named fields.

    Unlike the global propsDefault loader, this procedure neither clears an
    omitted dirXZToRollRateLim nor reads the legacy rollRateLimToDirXZFactor.
    Returned offsets use the same global layout as PROPERTY_FIELDS; callers
    translate them to the embedded object's property block.
    """
    return _pack(PROPERTY_FIELDS,properties)


def _pack(fields,values):
    out=[]
    for key,off,fmt in fields:
        if key not in values:continue
        value=values[key]
        out.append((off,struct.pack('<'+fmt,*(value if isinstance(value,(list,tuple)) else [value]))))
    return out


def configured_fields(config):
    out=_pack(FIELDS,config)
    if isinstance(config.get('propsDefault'),dict):out.extend(property_fields(config['propsDefault']))
    return out
