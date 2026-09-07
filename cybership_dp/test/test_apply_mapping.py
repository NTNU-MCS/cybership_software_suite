import numpy as np
import pytest
from geometry_msgs.msg import Wrench

from cybership_dp.force_controller_base import BaseForceControllerROS

MAPPING = {"force.x": 0, "force.y": 1, "torque.z": 2}


@pytest.mark.parametrize(
    "vector",
    [
        np.array([[1.0], [2.0], [3.0]]),  # (3, 1) column vector
        np.array([1.0, 2.0, 3.0]),  # (3,) flat vector
        np.array([[1.0, 2.0, 3.0]]),  # (1, 3) row vector
    ],
)
def test_apply_mapping_shapes(vector):
    msg = Wrench()
    BaseForceControllerROS._apply_mapping(msg=msg, vector=vector, mapping=MAPPING)

    assert msg.force.x == pytest.approx(1.0)
    assert msg.force.y == pytest.approx(2.0)
    assert msg.torque.z == pytest.approx(3.0)


def test_apply_mapping_out_of_range_index_defaults_to_zero():
    msg = Wrench()
    vector = np.array([[1.0], [2.0]])  # (2, 1) column vector, no index 2

    BaseForceControllerROS._apply_mapping(msg=msg, vector=vector, mapping=MAPPING)

    assert msg.force.x == pytest.approx(1.0)
    assert msg.force.y == pytest.approx(2.0)
    assert msg.torque.z == pytest.approx(0.0)
