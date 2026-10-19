from threading import Event
import pytest
from mono_hydra_perception.latest_frame_worker import LatestFrameWorker


def test_source_rate_and_bounded_latest_pending_frame():
    entered, release, finished = Event(), Event(), Event()
    seen, errors = [], []
    def callback(value):
        seen.append(value)
        if value == 0:
            entered.set()
            assert release.wait(3)
        else:
            finished.set()
    worker = LatestFrameWorker(callback, 5., errors.append)
    try:
        assert worker.submit(0, (0,))
        assert entered.wait(3)
        assert not worker.submit(33_333_333, (1,))
        assert worker.submit(200_000_000, (2,))
        assert worker.submit(400_000_000, (3,))
        assert worker.replaced == 1
        release.set()
        assert finished.wait(3)
    finally:
        release.set(); worker.close()
    assert seen == [0, 3]
    assert errors == []
    assert worker.statistics() == dict(received=4, selected=3, completed=2, replaced=1, errors=0)


def test_errors_do_not_kill_worker_and_clock_rewind_resets_selection():
    entered, done = Event(), Event(); seen = []; errors = []
    def callback(value):
        seen.append(value)
        if value == 1:
            entered.set()
            raise RuntimeError('test')
        done.set()
    worker = LatestFrameWorker(callback, 5., errors.append)
    try:
        assert worker.submit(10**9, (1,))
        assert entered.wait(3)
        assert not worker.submit(10**9, (1,))
        assert worker.submit(0, (2,))
        assert done.wait(3)
    finally:
        worker.close()
    assert seen == [1, 2]
    assert len(errors) == 1
    assert not worker.submit(10**9, (3,))


def test_invalid_rate():
    with pytest.raises(ValueError):
        LatestFrameWorker(lambda: None, float('nan'), lambda _: None)
