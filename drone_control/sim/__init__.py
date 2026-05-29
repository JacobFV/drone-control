"""
Lightweight, batched, headless quadrotor-swarm simulator.

A torch-vectorised 6-DOF multirotor sim (no Isaac/MuJoCo dependency) that runs
many environments x many drones in parallel, exposes a gym-like API, renders
optional synthetic camera frames, and collects data in the diffusion-VLA
training format. Designed so policies trained here speak the same
``DroneAction`` (roll/pitch/throttle/yaw bytes) interface as the real stack.
"""

from .dynamics import QuadParams, QuadrotorDynamics, SwarmState

__all__ = ["QuadParams", "QuadrotorDynamics", "SwarmState"]
