"""使用实际 OctoMap 二进制验证地板、低箱和错位命中的可撤销更新。"""
import os
from pathlib import Path
import subprocess
import numpy as np
import pytest
from mini_nav_fastlivo.bundle import write_pcd


@pytest.mark.parametrize('voxel', [.02, .03])
def test_floor_low_box_and_reversible_ghost(tmp_path, voxel):
    binary = os.environ.get('MINI_NAV_OFFLINE_BIN')
    if not binary:
        pytest.skip('Set MINI_NAV_OFFLINE_BIN to compiled offline binary directory')
    directory = tmp_path / 'replay'
    (directory / 'pcd').mkdir(parents=True)
    np.savetxt(directory / 'poses_map.txt', np.tile([0, 0, 0, 1, 0, 0, 0], (15, 1)))
    np.savetxt(directory / 'projection.txt', [[.05, 4, .02, .4, .032, 0, .232, .032, 0, -.078, 0, 0, 1, 0]])
    sensor = np.array([.032, 0, .232])
    wall = np.array([1., 0, .10])
    ghost = sensor + .8*(wall-sensor)
    floor = np.array([[x, y, 0.] for x in np.arange(.4, 1.5, .05) for y in np.arange(.1, .9, .05)])
    box = np.array([[.6, .5, .03]])
    for number in range(15):
        point = ghost if number < 4 else wall
        write_pcd(directory / 'pcd' / f'{number}.pcd', np.vstack([floor, box, point, [[1., 0, .35]]] if number == 14 else [floor, box, point]))
    subprocess.run([str(Path(binary) / 'replay_occupancy'), str(directory), str(voxel)], check=True, capture_output=True, env=dict(os.environ, LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0'))
    data = np.fromfile(directory / 'grid.bin', np.int8).reshape(80, 80)
    def cell(point):
        x, y = np.floor((point[:2]+2)/.05).astype(int)
        return data[y, x]
    assert cell(box[0]) == 100, '3cm low box must remain occupied'
    assert cell(wall) == 100, 'Repeated real wall must remain occupied'
    assert cell(np.array([.5, 0, .1])) == 0, 'A one-off no-hit ray in another height must not override strong free evidence'
    assert cell(ghost) != 100, 'Later true rays must remove misplaced hit'
    assert int((data == 100).sum()) < 12, 'Floor returns must not paint a black floor'
