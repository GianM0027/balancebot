"""Verifica dell'ambiente: numpy, MuJoCo (pendolo che cade) e JAX sulla GPU.

Uso:  python check_env.py           # solo i test
      python check_env.py --viewer  # apre anche il viewer di MuJoCo
"""
import sys

ok = True


def check(name, fn):
    global ok
    try:
        msg = fn()
        print(f"[OK]   {name}: {msg}")
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"[FAIL] {name}: {type(e).__name__}: {e}")


def t_numpy():
    import numpy as np
    return f"numpy {np.__version__}"


PENDULUM = """
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="1 1 .1" rgba=".8 .8 .8 1"/>
    <body pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.5" size="0.02" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""


def t_mujoco():
    import mujoco
    import numpy as np
    m = mujoco.MjModel.from_xml_string(PENDULUM)
    d = mujoco.MjData(m)
    d.qpos[0] = 0.05  # parte quasi in verticale, leggermente inclinato
    for _ in range(1000):  # 1 secondo simulato
        mujoco.mj_step(m, d)
    ang = np.degrees(d.qpos[0])
    assert abs(ang) > 30, "il pendolo inverso doveva cadere"
    return f"mujoco {mujoco.__version__}, pendolo caduto a {ang:.0f} gradi dopo 1 s"


def t_jax():
    import jax
    import jax.numpy as jnp
    devs = jax.devices()
    x = jnp.ones((2048, 2048))
    (x @ x).block_until_ready()
    gpu = any(d.platform == "gpu" for d in devs)
    if not gpu:
        raise RuntimeError(f"JAX non vede la GPU, solo: {devs}")
    return f"jax {jax.__version__}, dispositivi: {devs}"


check("numpy", t_numpy)
check("MuJoCo", t_mujoco)
check("JAX + GPU", t_jax)

if "--viewer" in sys.argv:
    import mujoco
    import mujoco.viewer
    m = mujoco.MjModel.from_xml_string(PENDULUM)
    d = mujoco.MjData(m)
    d.qpos[0] = 0.05
    mujoco.viewer.launch(m, d)

print("\nTutto pronto." if ok else "\nQualcosa non va: incollami l'output.")
