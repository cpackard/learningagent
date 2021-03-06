from learningagent import agent_percepts as percepts

class State():
    def __init__(self, location, certainty):
        self.location = location
        self.certainty = certainty

    def __repr__(self):
        return '({}, {})'.format(self.location, self.certainty)

    def __eq__(self, other):
        return (self.location == other.location
                and self.certainty == other.certainty)


class Agent():
    def __init__(self, goal, belief_state, visible_obstacles):
        self.goal = goal
        self.belief_state = belief_state
        self.visible_obstacles = visible_obstacles
        # a table indexed by state and action
        self.result = {}
        # a table of cost estimates index by state
        self.cost_estimates = {}
        self.prev_state = State(None, 1)
        self.prev_action = None
        self.prev_result = self.result
        self.prev_cost_estimates = self.cost_estimates
        self.score = 0
        self.reached_goal = False
        self.belief_history = []


    @staticmethod
    def LRTA_star_cost(prev_state, action, state, cost_estimates, goal):
        """
        Returns the f-value of a given state: g(s) + h(s)
        """
        if state is None:
            return percepts.heuristic(prev_state, goal)
        else:
            return (percepts.actual_cost(prev_state, action, state)
                    + cost_estimates[state])


    def goal_test(self, p):
        """
        Give a position p and a goal point, determine if there is an unblocked
        path from p to the goal.
        """
        return p == self.goal


    def best_actions(self, s, action_cost):
        """
        Given a state s and a cost function action_cost, return the actions with
        the lowest cost.
        """
        actions_from_s = sorted([action for action in percepts.actions(s)],
                                key=action_cost)

        best_actions_from_s = [v for v in actions_from_s
                               if action_cost(v) == action_cost(actions_from_s[0])]

        return best_actions_from_s


    def _prev_prob(self, a):
        """
        Given that agent is currently at A, return the
        probability that the agent was previously at prev_state.
        """
        def action_cost(action):
            return self.LRTA_star_cost(
                self.prev_state.location,
                action,
                self.prev_result.get((self.prev_state.location, action)),
                self.prev_cost_estimates,
                self.goal)

        visible_from_a = percepts.visible_vertices(a, self.visible_obstacles)

        best_actions_prev = self.best_actions(self.prev_state.location, action_cost)

        if (self.prev_state.location in visible_from_a
            and a in best_actions_prev):
            # Only consider vertices visible from A which have
            # A as one of the lowest values for lrta_star_cost
            best_actions_v = [v for v in visible_from_a
                              if a in self.best_actions(v, action_cost)]

            if len(best_actions_v) == 0:
                return 0
            else:
                return (len(best_actions_prev) / len(best_actions_v))
        else:
            return 0


    def _current_prob(self, a):
        """
        Given that the agent was previously in prev_state, return the
        probability that the agent is currently in state a.
        """
        a_locations = len(percepts.get_locations(
            percepts.visible_vertices(a, self.visible_obstacles),
            self.visible_obstacles))

        b_locations = len(percepts.get_locations(
            percepts.visible_vertices(self.prev_state.location,
                                      self.visible_obstacles),
            self.visible_obstacles))

        p_of_a = 1 / a_locations
        p_of_b = 1 / b_locations
        p_of_b_given_a = self._prev_prob(a)

        if p_of_b == 0:
            # Unless the agent is somehow teleported inside of a circle,
            # this should never happen
            return 0
        else:
            # Bayes theorem
            return p_of_b_given_a * p_of_a / p_of_b


    def _refine_loc(self, possible_locations):
        """
        Given a list of possible locations from agent percepts, narrow
        down the possibile locations based on agent's previous location.
        """
        possible_states = []

        for s in possible_locations:
            p = self._current_prob(s)
            if p > 0:
                possible_states.append(State(s, p))

        return possible_states


    def _update_certain(self, possible_locations):
        """
        Given that the agent is certain of its current location,
        update its past beliefs (if applicable) and
        return its updated location and belief history.
        """
        # Agent is sure of its current location
        self.belief_state = State(possible_locations[0], 1)

        if self.prev_state and self.prev_state.certainty < 1:
            # now that agent is sure of its location, update past
            # beliefs to correct any false assumptions
            # TODO agent.update_previous_beliefs(belief_history, agent_belief_state)
            pass

        # TODO Assuming prev_action is relative, this should add
        # prev_state + prev_action == agent_belief_state.location
        if self.prev_action == self.belief_state.location:
            # agent is certain of current and past position
            # and is at its intended destination, no further work to do
            pass
        else:
            # agent did not end at intended destination
            # TODO agent.recovery_scheme()
            pass

        self.belief_history = []


    def _update_uncertain(self, possible_locations):
        """
        Given that the agent is uncertain of its current location,
        refine its estimates to maximize the certainty of its current location.
        """
        # See if we can narrow down the location
        refined_locations = self._refine_loc(possible_locations)

        if len(refined_locations) == 1:
            return self.update_agent_location([refined_locations[0].location])
        elif len(refined_locations) == 0:
            # None of the current states are possible with the prev_state
            prev_possible_states = self.belief_history[-1]

            for l in sorted(prev_possible_states, key=lambda l: l.certainty):
                self.prev_state = l
                refined_locations = self._refine_loc(possible_locations)

                if not refined_locations:
                    # Current percepts and past state are incompatible,
                    # check the next past state with the most certainty
                    continue
                else:
                    return self._update_uncertain([s.location for s in refined_locations])
        else:
            # agent is unsure of past or current location, make best guess
            self.belief_history.append(refined_locations)
            self.belief_state = max(refined_locations, key=lambda s: s.certainty)


    def update_agent_location(self, possible_locations):
        """
        Given a list of possible locations, determine the agent's current location.
        If agent is certain of current location, update belief history if
        applicable. Otherwise, return the state with the highest certainty and
        add this set of beliefs to the belief history.
        """
        self.prev_state = self.belief_state

        if len(possible_locations) == 1:
            self._update_certain(possible_locations)
        else:
            if not self.prev_state or not self.belief_history:
                # no previous information, make best guess
                p = 1 / len(possible_locations)
                current_beliefs = [State(location, p)
                                   for location in possible_locations]
                self.belief_history.append(current_beliefs)

                # TODO pick one with min of lrta_action_cost
                if self.goal in possible_locations:
                    self.belief_state = State(self.goal, p)
                else:
                    self.belief_state = State(possible_locations[0], p)
            else:
                self._update_uncertain(possible_locations)


    # LRTA* algorithm from AIMA
    def LRTA_star_agent(self):
        """
        Given a percept that identifies the current state, return the next
        action to take, or None if the goal is reached.

        LRTA_star_agent select an action according to the values of neighboring
        states, which are updated as the agent moves about the state space.
        """
        loc = self.belief_state.location
        prev_loc = self.prev_state.location

        if self.goal_test(loc):
            self.prev_action = None
            return

        if loc not in self.cost_estimates:
            self.cost_estimates[loc] = percepts.heuristic(loc, self.goal)

        if self.prev_state.location:
            self.result[(prev_loc, self.prev_action)] = loc

            self.cost_estimates[prev_loc] = min(
                [self.LRTA_star_cost(
                    prev_loc,
                    action,
                    self.result.get((prev_loc, action)),
                    self.cost_estimates,
                    self.goal)
                for action in percepts.actions(prev_loc)])

        def lrta_action_cost(action):
            return self.LRTA_star_cost(
                loc,
                action,
                self.result.get((loc, action)),
                self.cost_estimates,
                self.goal)

        if percepts.line_is_unblocked(loc, self.goal, self.visible_obstacles):
            self.prev_action = self.goal
        else:
            self.prev_action = min([action for action in percepts.actions(loc)],
                                   key=lrta_action_cost)

        self.prev_state = self.belief_state
