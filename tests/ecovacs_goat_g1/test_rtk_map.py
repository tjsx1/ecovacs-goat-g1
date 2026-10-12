"""Tests for the O-series (RTK) map geometry decoder."""

import base64
import json
import lzma
from pathlib import Path
import sys
import types

PACKAGE_PATH = Path(__file__).parents[2] / "custom_components" / "ecovacs_goat_g1"

custom_components = types.ModuleType("custom_components")
custom_components.__path__ = [str(PACKAGE_PATH.parent)]
sys.modules.setdefault("custom_components", custom_components)

ecovacs_goat_g1 = types.ModuleType("custom_components.ecovacs_goat_g1")
ecovacs_goat_g1.__path__ = [str(PACKAGE_PATH)]
sys.modules.setdefault("custom_components.ecovacs_goat_g1", ecovacs_goat_g1)

from custom_components.ecovacs_goat_g1.mower_messages import (  # noqa: E402
    _rtk_chain,
    apply_command_data,
)
from custom_components.ecovacs_goat_g1.mower_models import MowerState  # noqa: E402


def _info(payload) -> str:
    """Encode like the mower: LZMA-alone with a 4-byte size, then base64."""
    data = json.dumps(payload).encode()
    raw = lzma.compress(data, format=lzma.FORMAT_ALONE)
    return base64.b64encode(raw[:5] + len(data).to_bytes(4, "little") + raw[13:]).decode()


def _chunks(payload, pieces: int = 1) -> list[dict]:
    info = _info(payload)
    size = -(-len(info) // pieces)
    return [
        {"mid": "1", "batid": "abc", "serial": str(pieces), "index": str(index),
         "info": info[index * size:(index + 1) * size]}
        for index in range(pieces)
    ]


def test_chain_code_square_closes() -> None:
    # 4 = east, 6 = south, 8 = west, 2 = north; "(n)" repeats the last digit.
    square = _rtk_chain("0,0", "4(10)6(10)8(10)2(10)")
    xs = {p.x for p in square}
    ys = {p.y for p in square}
    assert (square[0].x, square[0].y) == (square[-1].x, square[-1].y) == (0, 0)
    assert xs == {0, 500} and ys == {0, -500}
    assert len(square) == 5  # collinear steps are simplified away


def test_repeat_applies_to_last_digit_only() -> None:
    path = _rtk_chain("0,0", "24(3)")
    end = path[-1]
    assert (end.x, end.y) == (150, 50)


def test_lawns_obstacles_and_lanes() -> None:
    state = MowerState()
    lawns = [["1", "s1;1;0,0;4(20)6(20)8(20)2(20)", "2;1;5000,0;4(10)6(10)8(10)2(10)"], ["2", "1"]]
    for chunk in _chunks(lawns, pieces=2):
        state = apply_command_data(state, "onMI", chunk)
    assert len(state.map.info.lawns) == 2
    assert {p.x for p in state.map.info.outline} == {0, 1000}  # largest lawn

    obstacles = [["1", "1", "0"], ["1", "3", "1", "100;200,-200;4(2)6(2)8(2)2(2)"]]
    state = apply_command_data(state, "onArI", _chunks(obstacles)[0])
    assert len(state.map.info.obstacles) == 1

    snapshot = [["1", "1", "1;1;10;100,-900;100,-500;100,-300;100,-100", "1;1;11;200,-900;200,-100"]]
    state = apply_command_data(state, "onMapTrack", {**_chunks(snapshot)[0], "mid": "0"})
    assert state.map.trace.lanes["1:1:10"] == ((100, -900, 100, -500), (100, -300, 100, -100))
    update = [["1", "2", "1;1;12;300,-900;300,-100"]]
    state = apply_command_data(state, "onMapTrack", {**_chunks(update)[0], "mid": "0"})
    assert len(state.map.trace.lanes) == 3
    exported = state.map.as_dict()
    assert len(exported["trace"]["lanes"]) == 4
    assert len(exported["info"]["lawns"]) == 2


def test_geometry_mid_does_not_reset_map() -> None:
    state = MowerState()
    state = apply_command_data(state, "onPos", {"mid": "0", "deebotPos": {"x": 1, "y": 2}})
    lawns = [["1", "s1;1;0,0;4(4)6(4)8(4)2(4)"], ["2", "1"]]
    state = apply_command_data(state, "onMI", _chunks(lawns)[0])
    state = apply_command_data(state, "onPos", {"mid": "0", "deebotPos": {"x": 3, "y": 4}})
    assert state.map.info.lawns
    assert state.map.mid == "0"
