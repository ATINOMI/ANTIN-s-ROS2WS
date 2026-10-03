import numpy as np
from scipy.spatial.transform import Rotation
from mini_nav_fastlivo.geometry import transform
from mini_nav_fastlivo.registration import PriorMatcher


def test_prior_registration_recovers_transform():
    rng = np.random.default_rng(7)
    # 多面墙和地面提供足够三维约束，不使用有歧义的单一平面。
    p = rng.uniform(-2, 2, (5000, 3))
    p[:1600, 0] = -2
    p[1600:3300, 1] = 2
    p[3300:, 2] = -1
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler('z', 0.08).as_matrix()
    expected[:3, 3] = [0.12, -0.08, 0.03]
    source = transform(p, np.linalg.inv(expected))
    result, diagnostics = PriorMatcher(p).align(source, np.eye(4))
    assert diagnostics['valid']
    assert np.allclose(result, expected, atol=0.025)


def test_wrong_place_is_not_localized():
    rng = np.random.default_rng(11)
    points = rng.normal(size=(2000, 3))
    _, diagnostics = PriorMatcher(points).align(points + [30, 30, 0], np.eye(4))
    assert not diagnostics['valid']
