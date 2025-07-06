import jax.numpy as jnp
import jax
import jax_dataclasses as jdc

@jdc.pytree_dataclass
class Bar:
    x: jnp.ndarray
    y: jnp.ndarray
    name: jdc.Static[str]

bar = Bar(jnp.ones(3), jnp.ones(3), name="robot")

print(jax.tree_util.tree_leaves(bar))
