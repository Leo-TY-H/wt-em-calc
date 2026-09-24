"""Intact wing-area producer and per-wing induced-drag consumer.

Native101a31e80/101a34302 and106c5df20..df89/106c5f22f..f2fd.
Even intact area * reciprocal(area) need not round to one. The right wing
reuses the already adjusted left polar when the two fractions are equal,
then applies its own adjustment again. Keep that original copy order.
"""
from component_assembly import f32,add,mul


def reciprocal(value):
    return f32(1./value) if abs(value)>f32(4e-19) else 0.


def intact_ratios(areas):
    return [mul(total,reciprocal(total)) for total in
            (add(add(a[1],a[0]),a[2]) for a in areas)]


def intact_polars(polar,areas):
    left_ratio,right_ratio=intact_ratios(areas)
    left=dict(polar,indCoeff=mul(reciprocal(left_ratio),polar['indCoeff']))
    # Intact EM devices prescribe the same flap fraction on both wings.
    base=left if left_ratio==right_ratio else polar
    right=dict(base,indCoeff=mul(reciprocal(right_ratio),base['indCoeff']))
    return [left,right]
