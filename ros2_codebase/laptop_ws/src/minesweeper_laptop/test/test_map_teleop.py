import math

import pytest

from minesweeper_laptop.mapping import MineGrid
from minesweeper_laptop.teleop import axis_value


def test_first_detection_is_recorded_without_legacy_glitch_skip():
    grid = MineGrid()
    assert grid.record(0.5, 0.5, 0., 0.55, True)[:3] == (1, 0, 1)
    assert grid.tables()['surface'] == [{'column': 1, 'row': 0}]
    assert grid.record(0.5, 0.5, 0., 0.55, True) is None


def test_classification_can_upgrade_but_not_erase_surface():
    grid = MineGrid()
    assert grid.record(0.5, 0.5, 0., 0.55, False)[2] == 2
    assert grid.record(0.5, 0.5, 0., 0.55, True)[2] == 1
    assert grid.record(0.5, 0.5, 0., 0.55, False) is None


@pytest.mark.parametrize('x,y', [(-0.01, 0.), (0., -0.01), (20., 0.), (0., 20.), (math.nan, 0.)])
def test_out_of_bounds_never_wraps_to_another_row(x, y):
    grid = MineGrid()
    assert grid.record(x, y, 0., 0., True) is None
    assert all(v == 0 for v in grid.cells)


def test_start_corner_and_heading_transform_in_radians():
    grid = MineGrid(start_x=9.5, start_y=0.5, heading=math.pi/2)
    assert grid.transform(1., 0., 0.) == pytest.approx((9.5, 1.5, math.pi/2))


@pytest.mark.parametrize('axes,index,expected', [([], 0, 0.), ([1.], -1, 0.), ([float('nan')], 0, 0.),
                                               ([0.05], 0, 0.), ([2.], 0, 1.), ([-0.7], 0, -0.7)])
def test_axes_handle_missing_deadzone_and_bad_values(axes, index, expected):
    assert axis_value(axes, index, 0.08) == expected
