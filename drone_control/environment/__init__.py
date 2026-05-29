"""Environment abstraction: one interface over the simulator and the real swarm.

An ``Environment`` is where drones operate (simulated or real). It exposes a
uniform surface — drone ids, latest camera frame per drone, believed
trajectories/poses, and the shared world model — so the ``SessionService`` can
funnel both sim and real through one realtime path.
"""

from .base import Environment
from .sim_env import SimEnvironment
from .real_env import RealEnvironment

__all__ = ["Environment", "SimEnvironment", "RealEnvironment"]
