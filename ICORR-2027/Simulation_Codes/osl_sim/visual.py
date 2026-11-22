"""Display helpers shared by the live viewer and the video tool (visual settings only)."""


def brighten(model):
    """Stronger headlight -- the CAD meshes are dark against the default background."""
    model.vis.headlight.ambient[:] = (0.45, 0.45, 0.45)
    model.vis.headlight.diffuse[:] = (0.75, 0.75, 0.75)
    model.vis.headlight.specular[:] = (0.15, 0.15, 0.15)


def set_side_camera(cam, model):
    """Sagittal (side) view of the fixed-base leg: knee flexion is in the image plane."""
    cam.lookat[:] = model.stat.center
    cam.distance = 1.6 * model.stat.extent
    cam.azimuth = 90.0
    cam.elevation = -5.0


class LiveViewer:
    """MuJoCo passive viewer fed from its OWN MjData by a background thread.

    The physics loop only hands over a copy of qpos (``show``), so a slow GPU (a
    Raspberry Pi needs ~40 ms per frame) can never delay the real-time simulation.
    """

    def __init__(self, model, data, fps=30.0):
        import threading
        import warnings
        import mujoco
        import mujoco.viewer
        # e.g. "Wayland: The platform does not provide the window position" -- harmless
        warnings.filterwarnings("ignore", module="glfw")
        self._mujoco = mujoco
        self.model = model
        self.data = mujoco.MjData(model)
        self.data.qpos[:] = data.qpos
        mujoco.mj_forward(model, self.data)
        brighten(model)
        self.viewer = mujoco.viewer.launch_passive(model, self.data, show_left_ui=False,
                                                   show_right_ui=False)
        with self.viewer.lock():
            set_side_camera(self.viewer.cam, model)
        self._pending = data.qpos.copy()
        self._period = 1.0 / fps
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def show(self, qpos):
        self._pending = qpos.copy()

    def _loop(self):
        import time
        while not self._stop and self.viewer.is_running():
            t0 = time.perf_counter()
            with self.viewer.lock():
                self.data.qpos[:] = self._pending
                self._mujoco.mj_forward(self.model, self.data)
            self.viewer.sync()
            time.sleep(max(0.0, self._period - (time.perf_counter() - t0)))

    def is_running(self):
        return self.viewer.is_running()

    def close(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        self.viewer.close()


def open_viewer(model, data, log=print):
    """LiveViewer, or None if no display / OpenGL is available."""
    try:
        return LiveViewer(model, data)
    except Exception as exc:                                    # noqa: BLE001
        log(f"[viewer] Could not open the MuJoCo viewer ({exc}); continuing without it.")
        return None
