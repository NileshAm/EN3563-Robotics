"""Dashboard editing and wire requests, including its actual Tk widgets."""

import importlib.util
import json
from pathlib import Path
import sys
import tkinter as tk

import numpy as np
import pytest
import rclpy
from rclpy.task import Future
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location(
    'dashboard_under_test', Path(__file__).parents[1] / 'scripts' / 'dashboard.py')
dashboard = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = dashboard
spec.loader.exec_module(dashboard)


@pytest.fixture
def panel():
    rclpy.init(domain_id=193)
    try:
        root = tk.Tk()
    except tk.TclError:
        rclpy.shutdown()
        pytest.skip('Tk widget tests require a display')
    root.withdraw()
    panel = dashboard.Dashboard(root)
    try:
        yield panel
    finally:
        panel.close()


def test_defaults_preserve_original_tool_poses_and_gripper_order():
    defaults = dashboard.default_waypoints()
    assert [w.pose[:3] for w in defaults] == [
        (450, -200, 470), (450, -200, 170), (450, -200, 470),
        (500, 500, 470), (500, 500, 372), (500, 500, 470)]
    assert [w.gripper for w in defaults] == ['keep', 'grab', 'keep', 'keep', 'release', 'keep']
    for index, waypoint in enumerate(defaults):
        request = waypoint.request()
        assert request.move_linear == (index != 0)
        assert request.gripper_action == waypoint.gripper
        np.testing.assert_allclose(
            [request.target.position.x, request.target.position.y, request.target.position.z],
            np.array(waypoint.pose[:3]) / 1000)
        q = request.target.orientation
        np.testing.assert_allclose(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                   Rotation.from_euler('y', 90, degrees=True).as_matrix(), atol=1e-12)


def test_move_sends_current_edits_once_and_preserves_grab_choice(panel):
    class Client:
        requests = []

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return Future()

    panel.client = Client()
    assert panel.pending is None  # Constructing/selecting waypoints never moves.
    panel.table.selection_set('1')
    panel.select_waypoint()
    panel.name.set('Custom pickup')
    panel.fields[0].delete(0, tk.END)
    panel.fields[0].insert(0, '460')
    panel.request_move()
    panel.request_move()
    assert len(panel.client.requests) == 1
    request = panel.client.requests[0]
    assert request.target.position.x == 0.46
    assert request.gripper_action == 'grab'
    assert panel.waypoints[1].name == 'Custom pickup'
    assert panel.send.instate(['disabled'])


def test_edit_add_remove_and_invalid_input_do_not_dispatch(panel):
    panel.name.set('Extra waypoint')
    panel.add_waypoint()
    assert len(panel.waypoints) == 7
    assert panel.selected() == 6
    panel.fields[2].delete(0, tk.END)
    panel.fields[2].insert(0, 'nan')
    assert not panel.update_waypoint()
    assert panel.pending is None
    assert panel.waypoints[6].pose[2] == 470
    panel.remove_waypoint()
    assert len(panel.waypoints) == 6
    panel.restore_defaults()
    assert panel.selected() == 0


def test_file_round_trip_and_bad_input(tmp_path):
    path = tmp_path / 'waypoints.json'
    waypoint = dashboard.Waypoint('Custom', (400, 200, 300, 5, 85, 10), False, 'release')
    path.write_text(json.dumps([dashboard.asdict(waypoint)]))
    loaded = dashboard.read_waypoints(path)[0]
    assert loaded.request() == waypoint.request()
    for invalid in [[], {}, [{'name': 'bad', 'pose': [0] * 5}],
                    [{'name': 'bad', 'pose': [0] * 6, 'gripper': 'close'}]]:
        path.write_text(json.dumps(invalid))
        with pytest.raises(ValueError):
            dashboard.read_waypoints(path)


@pytest.fixture
def plan_client(panel, monkeypatch):
    class Client:
        def __init__(self):
            self.requests = []
            self.futures = []
            self.ready = True

        def service_is_ready(self):
            return self.ready

        def call_async(self, request):
            self.requests.append(request)
            future = Future()
            self.futures.append(future)
            return future

    panel.client = Client()
    monkeypatch.setattr(rclpy, 'spin_once', lambda *args, **kwargs: None)
    return panel.client


def tick(panel):
    panel.root.after_cancel(panel.poll_id)
    panel.poll()


def progress(panel, state, percent=0):
    panel.on_progress(dashboard.MoveProgress(state=state, percent=float(percent), message=state))


def accept(client):
    client.futures[-1].set_result(dashboard.MoveToPose.Response(accepted=True))


def test_plan_runs_every_waypoint_in_order_and_waits_for_gripper(panel, plan_client):
    panel.table.selection_set('1')
    panel.select_waypoint()
    panel.fields[0].delete(0, tk.END)
    panel.fields[0].insert(0, '460')
    panel.request_plan()
    panel.request_plan()  # A second click cannot duplicate the run.
    assert len(plan_client.requests) == 1
    assert plan_client.requests[0].gripper_action == 'keep'
    assert panel.execute.instate(['disabled'])
    # A previous move's latched completion cannot skip the first waypoint.
    progress(panel, 'succeeded', 100)
    accept(plan_client)
    tick(panel)
    assert len(plan_client.requests) == 1
    for index in range(6):
        if index:
            accept(plan_client)
        progress(panel, 'moving', 50)
        tick(panel)
        assert len(plan_client.requests) == index + 1
        if index in (1, 4):
            progress(panel, 'grabbing' if index == 1 else 'releasing', 99)
            tick(panel)
            assert len(plan_client.requests) == index + 1
        progress(panel, 'succeeded', 100)
        tick(panel)
    assert panel.plan is None
    assert panel.percent.get() == 100
    assert panel.status.get() == 'Motion plan completed: 6 waypoints'
    assert not panel.execute.instate(['disabled'])
    assert [r.gripper_action for r in plan_client.requests] == [
        'keep', 'grab', 'keep', 'keep', 'release', 'keep']
    assert plan_client.requests[1].target.position.x == 0.46


@pytest.mark.parametrize('failure', ['rejected', 'failed', 'exception', 'disconnected'])
def test_plan_stops_without_dispatching_later_waypoints(panel, plan_client, failure):
    panel.request_plan()
    progress(panel, 'moving', 10)
    if failure == 'rejected':
        plan_client.futures[-1].set_result(
            dashboard.MoveToPose.Response(accepted=False, message='Unreachable'))
    elif failure == 'exception':
        plan_client.futures[-1].set_exception(RuntimeError('Connection failed'))
    else:
        accept(plan_client)
        if failure == 'failed':
            progress(panel, 'failed', 99)
        else:
            plan_client.ready = False
    tick(panel)
    assert panel.plan is None
    assert len(plan_client.requests) == 1
    assert 'Motion plan stopped at waypoint 1' in panel.status.get()
    tick(panel)
    assert len(plan_client.requests) == 1


def test_plan_waits_for_acceptance_even_if_completion_arrives_first(panel, plan_client):
    panel.request_plan()
    progress(panel, 'moving', 1)
    progress(panel, 'succeeded', 100)
    tick(panel)
    assert len(plan_client.requests) == 1
    accept(plan_client)
    tick(panel)
    assert len(plan_client.requests) == 2
    # A duplicate completion from waypoint 1 must not advance waypoint 2.
    progress(panel, 'succeeded', 100)
    accept(plan_client)
    tick(panel)
    assert len(plan_client.requests) == 2


def test_invalid_or_empty_plan_never_moves(panel, plan_client):
    panel.fields[0].delete(0, tk.END)
    panel.fields[0].insert(0, 'nan')
    panel.request_plan()
    assert not plan_client.requests and panel.plan is None
    panel.waypoints.clear()
    panel.refresh()
    panel.request_plan()
    assert not plan_client.requests and panel.plan is None
