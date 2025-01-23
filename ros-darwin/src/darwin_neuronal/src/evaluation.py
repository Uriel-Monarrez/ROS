num_eval_episodes = 10
eval_rewards = []
 
for _ in range(num_eval_episodes):
    state = env.reset()
    eval_reward = 0
 
    for _ in range(max_steps_per_episode):
        action = np.argmax(dqn_agent(state[np.newaxis, :]))
        next_state, reward, done, _ = env.step(action)
        eval_reward += reward
        state = next_state
 
        if done:
            break
 
    eval_rewards.append(eval_reward)
 
average_eval_reward = np.mean(eval_rewards)
print(f"Average Evaluation Reward: {average_eval_reward}")