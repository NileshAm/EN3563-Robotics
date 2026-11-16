"""Cross-check independently parameterized DH and CAD transform chains."""
import numpy as np
from scipy.spatial.transform import Rotation
from robot_kinematics import H06
from robot_kinematics.dh import dh_fk


def test_nominal_dh_matches_cad_chain():
    for q in np.random.default_rng(3563).uniform(-70, 70, (30, 6)):
        nominal, cad = dh_fk(q), H06(*q)
        assert np.linalg.norm(nominal[:3, 3] - cad[:3, 3]) < 0.03
        assert Rotation.from_matrix(nominal[:3, :3] @ cad[:3, :3].T).magnitude() < 0.0001
