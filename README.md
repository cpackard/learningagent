Online Learning Agent
==========================
This package implements the maze-navigating agent from chapter 4 of Artificial Intelligence: A Modern Approach. Among its features are:

- **Location Discovery:** The agent only has a map of the environment and the obstacles it can see around itself. Using these, the agent decides its possible locations and formulates the best plan to the goal.
- **Plan Reformulation:** After reaching the goal, the agent is teleported to a random location on the map. From the new location, the agent can plan a new path to the goal.
- **Real-Time Learning:** The agent is guided by the Learning Real-Time A-Star algorithm, so it is encouraged to explore unseen parts of the map in search of the best path to the goal. While this creates a high initial overhead during the first round of exploration, subsequent rounds become significantly easier as the agent updates its internal map of the environment.

Below is an example simulation of the agent in an environment. The agent is penalized 1 point for every unit of distance moved, rewarded 1000 points every time it reaches the goal, and has 250 turns to maximize its score.

[![maze-agent](http://i.imgur.com/VlmpVvR.png)](https://youtu.be/XS-P-DK7nnE "maze-agent")

Future improvements to the project are:

- **Hidden obstacles:** Add obstacles in the environment that the agent cannot see until it collides with an obstacle. Upon discovering the obstacle, the agent should update its internal map and avoid it in the future.
- **Fault Recovery:** Add a twist where due to faulty sensors, the agent has a 30% chance to end in an unintended location after planning a move. The agent should detect that it isn't in the planned location and have a recovery scheme to get back.