"""Research oracle: apply selected upgrades through the original config modifier."""
from aircraft_upgrades import profile, effect_scales
from propulsion_config import decode


def apply(native, name):
    """Operate on a PropulsionGeneralNative installation, then decode its state."""
    for modification in profile(name)['applied']:
        a, b = effect_scales()
        native.xmm(0, [a]); native.xmm(1, [b])
        native.run(0x101a14130, [native.config_address, native.block(modification['effects'])])
    native.config = decode(native, native.config_address)
    return native.config
