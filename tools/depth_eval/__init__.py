"""Eval-only depth/SLAM evaluation harness.

NOTHING in here is imported by the perception runtime. It uses sim-privileged
ground truth (raycast against the scene geometry) to *evaluate* a depth/SLAM
front-end — exactly the privilege the perception path is forbidden from using.
Keep the import direction one-way: this package may import perception + sim;
perception/sim must never import this.
"""
