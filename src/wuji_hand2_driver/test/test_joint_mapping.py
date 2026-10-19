from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from wuji_hand2_driver.joint_mapping import JointLimits, joint_names, nid_to_index, ordered_entries


def test_nids_skip_tactile_slots_and_frames_are_scattered():
    nids = [bus * 5 + node for bus in range(5) for node in range(1, 5)]
    assert [nid_to_index(nid) for nid in nids] == list(range(20))
    entries = [SimpleNamespace(nid=nid) for nid in reversed(nids)]
    assert [e.nid for e in ordered_entries(entries)] == nids
    for invalid in [0, 5, 10, 15, 20, 25, 26, -1, True, 1.5]:
        with pytest.raises(ValueError):
            nid_to_index(invalid)
    for invalid in [entries[:19], entries[:-1] + [entries[0]],
                    entries[:-1] + [SimpleNamespace(nid=5)]]:
        with pytest.raises(ValueError):
            ordered_entries(invalid)


def test_urdf_order_is_explicit_not_xml_order(tmp_path):
    root = ET.Element("robot")
    expected = joint_names("left")
    for index, name in reversed(list(enumerate(expected))):
        joint = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(joint, "limit", lower=str(-index - 1), upper=str(index + 1), velocity="1.0")
    path = tmp_path / "left.urdf"
    ET.ElementTree(root).write(path)
    limits = JointLimits.from_urdf(path, "left")
    assert limits.names == expected
    assert limits.upper == tuple(range(1, 21))
    values = tuple(i / 20 for i in range(20))
    assert limits.validate_command(values[::-1], expected[::-1]) == values
    assert limits.validate_command(values, [n[2:] for n in expected]) == values
    with pytest.raises(ValueError):
        limits.validate_command(values, joint_names("right"))
    with pytest.raises(ValueError):
        limits.validate_command(values, [expected[0]] * 20)
    for invalid in [[0] * 19, [float("nan")] * 20, [float("inf")] * 20, [100] * 20]:
        with pytest.raises(ValueError):
            limits.validate_command(invalid)
