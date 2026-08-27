#!/usr/bin/env python3
"""Cross-process serialization for memory-constrained Jetson inference."""

from contextlib import contextmanager
import os
import threading

try:
    import fcntl
except ImportError:  # pragma: no cover - production is Linux/Jetson
    fcntl = None


_PROCESS_LOCK = threading.Lock()
DEFAULT_LOCK_PATH = '/tmp/parkingbot-yolo-gpu.lock'


class GpuInferenceGuard:
    """Allow one model initialization/inference section per Jetson at a time.

    Front and rear camera nodes intentionally remain separate ROS processes.
    ``flock`` coordinates those processes, while the thread lock also covers
    multi-threaded executors in one process. A crashed owner cannot leave the
    lock latched because the kernel releases its file descriptor.
    """

    def __init__(self, lock_path=None):
        self.lock_path = str(
            lock_path or os.environ.get(
                'PARKINGBOT_YOLO_GPU_LOCK', DEFAULT_LOCK_PATH))

    @contextmanager
    def hold(self):
        with _PROCESS_LOCK:
            handle = open(self.lock_path, 'a+', encoding='ascii')
            try:
                if fcntl is not None:
                    fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
                yield
            finally:
                if fcntl is not None:
                    fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
                handle.close()


GPU_INFERENCE_GUARD = GpuInferenceGuard()


def release_unused_cuda_cache():
    """Return allocator cache after CPU materialization; keep model on GPU."""
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except (ImportError, RuntimeError):
        # Host tests and CPU-only deployments have no CUDA allocator.
        return
