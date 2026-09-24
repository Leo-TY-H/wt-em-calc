"""Fast copies of private numerical result graphs (never untrusted input)."""
import pickle


def clone(value):
    # The C pickler preserves aliases just as deepcopy does, including arrays
    # and cached interpolation objects. Bytes never leave this process here.
    return pickle.loads(pickle.dumps(value,protocol=pickle.HIGHEST_PROTOCOL))
