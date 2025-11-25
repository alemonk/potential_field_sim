# controller/geometry.py
import math

def quantize(v, q):
    return q * round(v / q)

def clampf(v, lo, hi):
    return max(lo, min(hi, v))

def wrapToPi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
