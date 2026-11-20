import gymnasium as gym
from pathlib import Path
import csv
import time


def load_policy(filename: Path) -> dict:
    with open(filename, 'r', newline='\n') as f:
        csv_reader = csv.reader(f, delimiter=',')

        policy = {}
        for line in csv_reader:
            if line[0].startswith('#'):
                continue

            state = int(line[0])
            action = int(line[1])
            policy[state] = action
        return policy


if __name__ == '__main__':


    policy_path = Path('/home/alex/qi3/bitrl/build/examples/rl/rl_example_9/policy.csv')
    policy = load_policy(policy_path)
    max_episode_steps = 200
    version = 'v0'
    env_tag = f"CliffWalking-{version}"
    env = gym.make(id=env_tag,
                   max_episode_steps=max_episode_steps,
                   render_mode="human")

    state, _ = env.reset()
    done = False
    total_reward = 0

    while not done:
        
        action = policy[state]
        print(f'At state {state} taking action {action}')
        
        observation, reward, done, truncated, info = env.step(action)

        print(f"New state: {observation}, truncated: {truncated} done: {done}")

        total_reward += reward
        env.render()
        time.sleep(5)
        if truncated:
            done = True

        state = observation

    print(f'Total reward {total_reward}')
        



