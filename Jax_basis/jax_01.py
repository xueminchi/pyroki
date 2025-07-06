import jax
import jax.numpy as jnp
import time

from jax import jit

@jit
def f(x):
    return x**2 + 2*x + 1

print(f(3.0))  # 输出 16