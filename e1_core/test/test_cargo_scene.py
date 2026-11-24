import xml.etree.ElementTree as ET

import pytest

pytest.importorskip('mujoco')

from e1_core.sim.mujoco_sim import build_cargo_xml, build_swarm_xml  # noqa: E402


POSITIONS = [
    (1, (-0.25, -0.25, 0.0198), 0.0),
    (2, (0.25, -0.25, 0.0198), 0.0),
    (3, (-0.25, 0.25, 0.0198), 0.0),
    (4, (0.25, 0.25, 0.0198), 0.0),
]


def test_cargo_xml_contains_box_and_unilateral_tethers():
    root = ET.fromstring(build_cargo_xml(POSITIONS))

    payload = root.find("./worldbody/body[@name='payload']")
    assert payload is not None
    assert payload.find("freejoint[@name='payload_root']") is not None
    assert payload.find("inertial").get("mass") == "0.1"

    tendons = root.findall("./tendon/spatial")
    assert len(tendons) == 4
    assert {tendon.get("range") for tendon in tendons} == {"0 0.3"}
    assert all(tendon.get("limited") == "true" for tendon in tendons)
    assert all(len(tendon.findall("site")) == 2 for tendon in tendons)


def test_default_swarm_xml_has_no_payload_or_tethers():
    root = ET.fromstring(build_swarm_xml(POSITIONS))

    assert root.find("./worldbody/body[@name='payload']") is None
    assert root.find("tendon") is None


@pytest.mark.parametrize(
    "kwargs",
    [
        {"box_mass": 0},
        {"box_mass": float("inf")},
        {"tether_length": 0},
        {"tether_length": float("nan")},
    ],
)
def test_cargo_xml_rejects_non_positive_or_non_finite_parameters(kwargs):
    with pytest.raises(ValueError):
        build_cargo_xml(POSITIONS, **kwargs)


def test_cargo_xml_requires_four_drones_in_separate_quadrants():
    with pytest.raises(ValueError, match="exactly four"):
        build_cargo_xml(POSITIONS[:3])

    overlapping_quadrants = [*POSITIONS[:3], (4, (-0.2, -0.2, 0.0198), 0.0)]
    with pytest.raises(ValueError, match="quadrants"):
        build_cargo_xml(overlapping_quadrants)
