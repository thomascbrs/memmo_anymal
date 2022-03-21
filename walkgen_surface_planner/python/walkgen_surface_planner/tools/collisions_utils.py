import numpy as np
import hppfcl

_EPS = 0.01

def distance(object1, object2, tf1 = hppfcl.Transform3f(),tf2 = hppfcl.Transform3f() ):
    """
    Returns the distance between object1 and object2
    """
    guess = np.array([1., 0., 0.])
    support_hint = np.array([0, 0], dtype=np.int32)

    shape = hppfcl.MinkowskiDiff()
    shape.set(object1, object2, tf1, tf2 )
    gjk = hppfcl.GJK(150, 1e-8)
    gjk.evaluate(shape, guess, support_hint)
    return gjk.distance

def convert_to_convexFcl(vertices):
    """ Convert to hppfcl convex object for collision checking.
    """
    verts = hppfcl.StdVec_Vec3f ()
    verts.extend( [np.array(p) for p in vertices])
    verts.extend( [np.array([p[0], p[1], p[2] + _EPS]) for p in vertices]) # 3D convex hull of epsilon width.
    return hppfcl.Convex.convexHull(verts, False, "")