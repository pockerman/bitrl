"""Make a GIF of an episode in a Gymnasium environment.

The algorithms of bitrl train on Gymnasium environments that run on the
bitrl-rest-api server. This script renders an episode of what they learned,
in two ways:

- with --policy FILE, it plays the policy that MaxTabularPolicy::save wrote,
  a state and an action per row, for environments with discrete states such
  as FrozenLake and CliffWalking;
- with --actions FILE, it replays the actions an example wrote, one per row,
  as numbers separated by commas; a first line "# seed N" gives the seed the
  episode was reset with. The environment is the same Gymnasium version, so
  the episode is the same.

Usage:
  python episode_gif.py --env FrozenLake-v1 --kwargs is_slippery=False --policy policy.csv --out run.gif
  python episode_gif.py --env CartPole-v1 --actions episode.csv --out run.gif --title "REINFORCE"

Needs gymnasium, with pygame for the pictures, imageio and Pillow.
"""
import argparse
import ast

import gymnasium as gym
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw


def parse_kwargs(items):
    kwargs = {}
    for item in items or []:
        key, value = item.split("=", 1)
        try:
            kwargs[key] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            kwargs[key] = value
    return kwargs


def read_policy(path):
    policy = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            state, action = line.rstrip(",").split(",")[:2]
            policy[int(state)] = int(action)
    return policy


def read_actions(path):
    seed = None
    actions = []
    with open(path) as f:
        for line in f:
            line = line.strip().rstrip(",")
            if not line:
                continue
            if line.startswith("#"):
                words = line[1:].split()
                if len(words) == 2 and words[0] == "seed":
                    seed = int(words[1])
                continue
            values = [float(v) for v in line.split(",")]
            actions.append(values)
    return seed, actions


def caption(frame, text, height=28):
    image = Image.fromarray(frame)
    out = Image.new("RGB", (image.width, image.height + height), (245, 245, 245))
    out.paste(image, (0, 0))
    ImageDraw.Draw(out).text((8, image.height + 7), text, fill=(20, 20, 20))
    return np.asarray(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--env", required=True)
    parser.add_argument("--kwargs", nargs="*")
    parser.add_argument("--policy")
    parser.add_argument("--actions")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--title", default="")
    parser.add_argument("--hold", type=int, default=10, help="frames to show the end for")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    env = gym.make(args.env, render_mode="rgb_array", **parse_kwargs(args.kwargs))
    seed = args.seed
    actions = None
    policy = None
    if args.actions:
        file_seed, actions = read_actions(args.actions)
        seed = file_seed if file_seed is not None else seed
    elif args.policy:
        policy = read_policy(args.policy)
    else:
        parser.error("give --policy or --actions")

    discrete = isinstance(env.action_space, gym.spaces.Discrete)
    observation, _ = env.reset(seed=seed)
    total = 0.0
    frames = [caption(env.render(), f"{args.title}   step 0   return 0")]
    for step in range(args.max_steps):
        if actions is not None:
            if step >= len(actions):
                break
            action = int(actions[step][0]) if discrete else np.array(actions[step], dtype=np.float32)
        else:
            action = policy[int(observation)]
        observation, reward, terminated, truncated, _ = env.step(action)
        total += reward
        frames.append(caption(env.render(), f"{args.title}   step {step + 1}   return {total:g}"))
        if terminated or truncated:
            break
    frames += [frames[-1]] * args.hold
    env.close()

    imageio.mimsave(args.out, frames, duration=1.0 / args.fps, loop=0)
    print(f"Wrote {args.out}: {len(frames) - args.hold} frames, return {total:g}")


if __name__ == "__main__":
    main()
