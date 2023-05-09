#!/usr/bin/env python3
import functools
import gym
import itertools
import numpy as np
import scipy.optimize

@functools.singledispatch
def iter_space(space):
    yield from space

@iter_space.register(gym.spaces.Discrete)
def _iter_space_discrete(space):
        if hasattr(space, "start"):
            yield from range(space.start, space.start+space.n)
        else:
            yield from range(space.n)

@iter_space.register(gym.spaces.Tuple)
def _iter_space_tuple(space):
    yield from itertools.product(*(iter_space(s) for s in space))

def reset_and_step(env, s, a):
    _s, _ = env.reset(state=s)
    if s != _s:
        raise RuntimeError("Env does not support reset to a specific space")
    sp, r, _, _, _ = env.step(a)
    return sp, r


@functools.singledispatch
def linprog(space, c):
     raise NotImplementedError()

@linprog.register(gym.spaces.Box)
def _linprog_box(space, c):

    bounds = [ (low, high) if low > -np.inf or high < np.inf else None for low, high in zip(space.low, space.high) ]
    
    res = scipy.optimize.linprog(-c, bounds=bounds)
    if res.success:
        return res.fun, res.x
    else:
         return None, None