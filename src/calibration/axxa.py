import numpy as np


def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def test_skew():
    print 'A matrix V := skew(v) is skew symmetric if V.T == -V:',
    sv = skew(v=np.random.random_sample((3, 1)))
    print 'passed.' if np.isclose(sv.T, -sv).all() else 'failed!'


def random_rotation_matrix():
    q, r = np.linalg.qr(np.random.random_sample((3, 3)) - 0.5)
    return q


def test_rotation_matrix(rot):
    print 'A matrix R is a rotation matrix if and only if'
    print ' - inv(R) == R.T:', 'passed.' if \
        np.isclose(np.linalg.inv(rot), rot.T).all() else 'failed!'
    print ' - R.T R == I:', 'passed.' if \
        np.isclose(np.dot(rot.T, rot), np.eye(3)).all() else 'failed!'
    print ' - det(R) == 1:', 'passed.' if \
        np.isclose(np.linalg.det(rot), 1.0) else 'failed!'


def test_random_rotation_matrix():
    print 'Test random rotation matrix generation:'
    test_rotation_matrix(rot=random_rotation_matrix())


def angle_axis_rotation_matrix(angle, axis):
    # R = d d.T + cos(a) (I - d d.T) + sin(a) skew(d)
    mag = np.linalg.norm(axis, ord=2)
    if mag == 0.0:
        return np.eye(3)
    d = axis/mag
    ddt = np.outer(d, d)
    rot = ddt + np.cos(angle)*(np.eye(3) - ddt) + np.sin(angle)*skew(d)
    return rot


def test_angle_axis_rotation_matrix():
    print 'Test angle-axis rotation matrix construction:'
    rot = angle_axis_rotation_matrix(angle=np.random.random_sample(1),
                                     axis=np.random.random_sample((3, 1)))
    test_rotation_matrix(rot=rot)


def random_trafo_matrix():
    trafo = np.eye(4)
    trafo[:-1, :-1] = random_rotation_matrix()
    trafo[:-1, -1] = np.random.random_sample(3)
    return trafo


def inv_trafo_matrix(trafo):
    it = np.eye(4)
    it[:-1, :-1] = trafo[:-1, :-1].T
    it[:-1, -1] = -np.dot(trafo[:-1, :-1].T, trafo[:-1, -1])
    return it


def test_inv_trafo_matrix():
    print 'The inverse of a homogeneous transform satisfies inv(T) T == I:',
    trafo = random_trafo_matrix()
    print 'passed.' if np.isclose(np.dot(inv_trafo_matrix(trafo=trafo), trafo),
                                  np.eye(4)).all() else 'failed!'


def quaternion_from_rotation_matrix(rot):
    quat = np.array([rot[2, 1] - rot[1, 2], 
                     rot[0, 2] - rot[2, 0], 
                     rot[1, 0] - rot[0, 1],
                     1 + np.trace(rot)])
    return quat/(2*np.sqrt(1 + np.trace(rot)))


def rotation_matrix_from_quaternion(quat):
    p = np.dot(quat.T, quat)
    if p == 0.0:
        return np.eye(3)
    if not np.isclose(p, 1.0):
        raise ValueError('Expected a unit quaternion, got {} (norm={})!'.format(quat, p))
    return np.eye(3) + 2*(np.outer(quat[:-1], quat[:-1]) - np.dot(quat[:-1].T, quat[:-1])
*np.eye(3) + quat[-1]*skew(quat[:-1]))


def test_quaternion_rotation_matrix():
    rot_1 = random_rotation_matrix()
    print 'A unit quaternion satisfies q.T q = 1:',
    quat = quaternion_from_rotation_matrix(rot=rot_1)
    print 'passed.' if np.isclose(np.dot(quat.T, quat), 1.0) else 'failed!'
    rot_2 = rotation_matrix_from_quaternion(quat=quat)
    test_rotation_matrix(rot=rot_2)
    print 'Original and reconstructed rotation matrix are the same:',
    print 'passed.' if np.isclose(rot_1, rot_2).all() else 'failed!'



def tsai_lenz_89(a, b):
    # We draw inspiration from the implementation by Zoran Lazarevic found at 
    # http://lazax.com/www.cs.columbia.edu/~laza/html/Stewart/matlab/handEye.m
    if not (isinstance(a, list) and isinstance(b, list)):
        raise ValueError('Expected lists of relative transformations, '
                         'got {} and {}!'.format(type(a), type(b)))
    if not len(a) == len(b):
        raise ValueError('Require the same number of relative transformations '
                         'for a and b, got {} and {}!'.format(len(a), len(b)))

    # solve for rotation
    n = len(a)
    na = np.zeros(3*n)
    nb = np.zeros(3*n)
    snapnb = np.zeros((3*n, 3))
    for i, (trafo_a, trafo_b) in enumerate(zip(a, b)):
        # TODO: why are we allowed to replace the eigenvector computation 
        # with the quaternion represenation?
        # evals, evecs = np.linalg.eig(trafo_a[:-1, :-1])
        # evec_a = evecs[:, np.argmax(evals.real)].real
        evec_a = quaternion_from_rotation_matrix(rot=trafo_a[:-1, :-1])[:-1]
        # treat quaternion quat := [v1, v2, v3, p] as quat = p*v, 
        # where p = sin(theta/2) and |v| = 1
        na[3*i:3*i+3] = evec_a

        # evals, evecs = np.linalg.eig(trafo_b[:-1, :-1])
        # evec_b = evecs[:, np.argmax(evals.real)].real
        evec_b = quaternion_from_rotation_matrix(rot=trafo_b[:-1, :-1])[:-1]
        nb[3*i:3*i+3] = evec_b

        snapnb[3*i:3*i+3, :] = skew(v=evec_a + evec_b)
    m, res, rank, _ = np.linalg.lstsq(a=snapnb, b=na-nb)
    if rank < 3:
        print 'Matrix skew(na + nb) is singular with rank={}!'.format(rank)
    print 'Least squares solution is {} with residual={}.'.format(
        m, float(res) if len(res) > 0 else 'NaN')
    mag = np.linalg.norm(m, ord=2)
    # TODO: why is this minus required?
    axis = -m/(mag if mag > 0 else (mag + np.finfo(float).eps))
    angle = 2.0*np.arctan(mag)
    print 'This is a rotation about axis={} by an angle={} deg.'.format(
        axis, np.rad2deg(angle))
    rot = angle_axis_rotation_matrix(angle=angle, axis=axis)
    # rot = rotation_matrix_from_quaternion(quat=np.concatenate([axis, [0]]))

    # with rotation, solve for translation
    n = len(a)
    rnami = np.zeros((3*n, 3))
    ta = np.zeros(3*n)
    rtb = np.zeros(3*n)
    for i, (trafo_a, trafo_b) in enumerate(zip(a, b)):
        rnami[3*i:3*i+3] = trafo_a[:-1, :-1] - np.eye(3)
        ta[3*i:3*i+3] = trafo_a[:-1, -1]
        rtb[3*i:3*i+3] = np.dot(rot, trafo_b[:-1, -1])
    trans, res, rank, _ = np.linalg.lstsq(a=rnami, b=rtb - ta)
    if rank < 3:
        print 'Matrix Ra - I is singular with rank={}!'.format(rank)
    print 'Least squares solution is {} with residual={}.'.format(
        trans, float(res) if len(res) > 0 else 'NaN')

    # assemble homogeneous transformation
    trafo = np.eye(4)
    trafo[:-1, :-1] = rot
    trafo[:-1, -1] = trans
    return trafo


def test():
    print 'Test utility functions:'
    test_skew()
    test_random_rotation_matrix()
    test_inv_trafo_matrix()
    test_angle_axis_rotation_matrix()
    test_quaternion_rotation_matrix()
    print ''


def main():
    print 'Create dummy data:'
    # define transform
    trafo = random_trafo_matrix()
    # trafo = np.eye(4); trafo[:-1, -1] = [0.001, 0, 300]
    # trafo[:-1, :-1] = angle_axis_rotation_matrix(angle=-np.pi/9, axis=(0.0, 0.0, 1.0))
    test_rotation_matrix(trafo[:-1, :-1])
    print 'original transformation'
    print trafo

    # sample dummy relative transforms
    n = 2
    a = [random_trafo_matrix() for _ in xrange(n)]
    print 'Sampled n={} relative trafos'.format(n)
    # print a

    # compute corresponding relative transforms using original trafo
    trafo_inv = inv_trafo_matrix(trafo=trafo)
    b = [np.dot(np.dot(trafo_inv, x), trafo) for x in a]
    print 'Computed corresponding n={} relative trafos'.format(n)
    # print b

    print ''
    print 'Apply algorithm by Tsai and Lenz (1989):'
    trafo_est = tsai_lenz_89(a=a, b=b)
    print 'estimated transformation'
    print trafo_est

    print 'Difference between original and reconstructed trafo: ' \
          'Frobenius={}.'.format(np.linalg.norm(trafo - trafo_est, ord='fro'))

    # test with dummy points
    print ''
    print 'Create dummy test:'
    print '   orig.       est.        delta'
    print '-'*40
    n = 5
    data = [np.concatenate((np.random.random_sample(3), [1]), axis=0)
            for _ in xrange(n)]
    for x in data:
        orig = np.dot(trafo, x)
        est = np.dot(trafo_est, x)
        delta = orig - est
        print np.concatenate((orig[:, np.newaxis],
                              est[:, np.newaxis],
                              delta[:, np.newaxis]), axis=1)


if __name__ == '__main__':
    test()
    main()
