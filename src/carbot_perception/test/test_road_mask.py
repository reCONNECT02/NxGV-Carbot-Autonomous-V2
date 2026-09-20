"""Block 03 end to end on synthetic camera images of a straight lane."""
import numpy as np
from synth import MOUNTS, fisheye, pinhole, render

from carbot_perception.camera_model import Intrinsics
from carbot_perception.road_mask import (PAINT, ROAD, ROLE_CODE, UNSEEN, BodyBox, Classify,
                                         GridSpec, OverlayLookup, RoadMask, Seed,
                                         build_camera_map)

GRID = GridSpec(100, 0.018, -0.65, -0.90)
BODY = BodyBox(0.042, 0.258, 0.096)


def make(intrs):
    rm = RoadMask(GRID, Seed())
    images = {}
    for role, intr in intrs.items():
        rm.maps[role] = build_camera_map(role, GRID, intr, MOUNTS[role], BODY, 0.025, 0.1, 2.0)
        images[role] = render(intr, MOUNTS[role])
    return rm, images


def test_lane_is_found_and_lines_are_paint():
    intrs = {'front': Intrinsics.ideal(480, 360, 60.0), 'left_rear': fisheye(), 'right_rear': pinhole()}
    rm, images = make(intrs)
    res = rm.process(images, Classify())
    gx, gy = GRID.centres()
    seen = res.kind != UNSEEN
    assert res.coverage > 0.2
    road_cells = seen & (np.abs(gy) < 0.33)
    grass_cells = seen & (np.abs(gy) > 0.40)
    assert (res.kind[road_cells] == ROAD).mean() > 0.97
    assert (res.grown[road_cells] > 0).mean() > 0.9
    assert (res.kind[grass_cells] == ROAD).mean() < 0.01
    line = seen & (np.abs(np.abs(gy) - 0.36) < 0.004)
    if line.any():
        assert (res.kind[line] == PAINT).mean() > 0.6
    # the left camera must win cells far out on the left, the right camera on the right
    assert (res.source[seen & (gy > 0.6)] == ROLE_CODE['left_rear']).mean() > 0.9
    assert (res.source[seen & (gy < -0.6)] == ROLE_CODE['right_rear']).mean() > 0.9


def test_stale_camera_drops_out():
    intrs = {'front': Intrinsics.ideal(480, 360, 60.0), 'left_rear': fisheye(), 'right_rear': pinhole()}
    rm, images = make(intrs)
    full = rm.process(images, Classify())
    images['left_rear'] = None
    part = rm.process(images, Classify())
    assert part.coverage < full.coverage
    assert not (part.source == ROLE_CODE['left_rear']).any()


def test_body_is_never_observed():
    intrs = {'left_rear': fisheye()}
    rm, images = make(intrs)
    res = rm.process(images, Classify())
    gx, gy = GRID.centres()
    inside = (gx > -0.042) & (gx < 0.258) & (np.abs(gy) < 0.096)
    assert (res.kind[inside] == UNSEEN).all()


def test_overlay_draws_on_road():
    intr = pinhole()
    rm, images = make({'right_rear': intr})
    res = rm.process(images, Classify())
    lk = OverlayLookup(GRID, intr, MOUNTS['right_rear'], 240)
    out = lk.draw(images['right_rear'], res)
    assert out.shape == (136, 240, 3)
    assert (lk.cell >= 0).mean() > 0.1
