"""Select the configured portable equation backend."""
from em_backend import activate


def prepare():
    return activate()


prepare()
