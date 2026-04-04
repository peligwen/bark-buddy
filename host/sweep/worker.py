"""
Single trial worker — runs one physics simulation with given parameters
and scenario, returns metrics dict.

Designed to be called from multiprocessing.Pool. Each worker instantiates
its own DogPhysics (pure Python, no shared state).
"""

import sys
import os

# Ensure host/ is on path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sim.physics import (
    DogPhysics, Vec3, Quat,
    standing_height, FOOT_R,
)

SIM_DT = 1.0 / 240.0


def apply_physics_params(phys: DogPhysics, params: dict):
    """Patch physics constants on a DogPhysics instance based on sweep params.

    We import and monkey-patch the module-level constants that the physics
    engine reads. This is safe because each worker runs in its own process.
    """
    import sim.physics as pm

    mapping = {
        # Gait
        "hip_amplitude": "GAIT_HIP_AMPLITUDE",
        "knee_amplitude": "GAIT_KNEE_AMPLITUDE",
        "lift_height": "GAIT_LIFT_HEIGHT",
        "gait_frequency": "GAIT_FREQUENCY",
        # Contact
        "contact_k": "CONTACT_K",
        "contact_c": "CONTACT_C",
        "contact_mu": "CONTACT_MU",
        # Joint servo
        "joint_kp": "JOINT_KP",
        "joint_kd": "JOINT_KD",
        "max_torque": "JOINT_MAX_TORQUE",
        # Movement
        "forward_speed": "FORWARD_SPEED",
        "turn_speed": "TURN_SPEED",
    }

    for param_name, module_attr in mapping.items():
        if param_name in params:
            setattr(pm, module_attr, params[param_name])


def run_trial(args: tuple) -> dict:
    """Run a single trial. Called by multiprocessing.Pool.map.

    Args:
        args: (trial_id, params_dict, scenario_name, scenario_config)

    Returns:
        dict with trial_id, params, metrics, and pass/fail info.
    """
    trial_id, params, scenario_name, scenario_config = args

    # Import scenarios here (inside worker process)
    from sweep.scenarios import SCENARIOS

    scenario_fn = SCENARIOS[scenario_name]

    # Create fresh physics and apply params
    phys = DogPhysics()
    apply_physics_params(phys, params)

    # Reset after applying params (standing height may change)
    phys.pos = Vec3(0, standing_height(), 0)
    phys.vel = Vec3()
    phys.orient = Quat()
    phys.omega = Vec3()

    # Run scenario
    try:
        metrics = scenario_fn(phys, params, scenario_config)
    except Exception as e:
        metrics = {"error": str(e), "fell": True}

    return {
        "trial_id": trial_id,
        "params": params,
        "scenario": scenario_name,
        "metrics": metrics,
    }
