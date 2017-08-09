"""
 * scene configuration script
"""
import numpy as np
import sys

def step_function(x, y, z):
    """
    in: x, y, z
    out: x, y, z, dx/dt, dy/dt, dz/dt
    """
    return [x, y, z, 0., 0., 1.]

def scene_config(history):
    """
    what to do: create a list of objects to place initially
    [x, y, z, property, step_function, model_file]
    where property can be one of the following:
        'agent', 'block', 'movable' and 'goal'
    Agent can be only one. Goals can be multiple.
    The behaviors of agent and goal are pre-defined, their step_functions are
        ignored.
    """
    ret_list = []
    # goals
    '''[x, y, z, property, step_function, model_file]'''
    ret_list.append([5., 5., 3., 'goal', None, 'models_household/apple/apple.urdf'])
    # agent
    # ret_list.append([5., 0.0, 0., 'agent', None, 'models_household/ball/ball.urdf'])
    ret_list.append([-1, 5.0, 0., 'agent', None, 'models_household/ball-small/ball.urdf'])
    # movable
    # ret_list.append([3., 2., 0., 'movable', step_function, 'models_household/cube/cube.urdf'])
    # ret_list.append([5., 2., 0., 'movable', step_function, 'models_household/cube/cube.urdf'])
    # block
    ret_list.append([4., 5., 2., 'block', step_function, 'models_household/block/block.urdf'])
    ret_list.append([3., 5., 1., 'block', step_function, 'models_household/block/block.urdf'])
    ret_list.append([2., 5., 0., 'block', step_function, 'models_household/block/block.urdf'])
    return random_maze_config(history)

max_dist = 2.1
def random_maze_config(history):
    global max_dist
    if len(history) > 100:
        history.pop(0)
    if sum(history) > 95:
        del history[:]
        max_dist = max_dist + 1
    pos_reg = np.zeros((5, 5), dtype=np.int8)
    agent_x = np.random.randint(1, 6)
    agent_y = np.random.randint(1, 6)
    pos_reg[agent_x - 1][agent_y - 1] = 1
    while True:
        goal_x = np.random.randint(1, 6)
        goal_y = np.random.randint(1, 6)
        if goal_x != agent_x or goal_y != agent_y:
            current_dist = abs(goal_x - agent_x) + abs(goal_y - agent_y)
            if current_dist <= max_dist and current_dist > 1.1:
                print ('[Dist Info] {:03d} / {:03d}'.format(current_dist,
                    int(max_dist)))
                sys.stdout.flush()
                pos_reg[goal_x - 1][goal_y - 1] = 2
                break
    o_list = []
    while True:
        del o_list[:]
        pos_copy = pos_reg.copy()
        for xx in range(5):
            for yy in range(5):
                if pos_reg[xx][yy] != 0:
                    continue
                if np.random.rand() > 0.3:
                    continue
                o_list.append([xx, yy])
                pos_copy[xx][yy] = 3
        reachable = False
        conn_list = [[agent_x - 1, agent_y - 1], ]
        conn_vist = np.zeros((5, 5), dtype=np.int8)
        while len(conn_list) > 0:
            x, y = conn_list.pop(0)
            if conn_vist[x][y] > 0:
                continue
            conn_vist[x][y] = 1
            xxyy = [[x - 1, y], [x + 1, y], [x, y - 1], [x,  y + 1]]
            for xx, yy in xxyy:
                if xx < 0 or xx >= 5 or yy < 0 or yy >= 5:
                    continue
                if pos_copy[xx][yy] == 0:
                    conn_list.append([xx, yy])
                elif pos_copy[xx][yy] == 2:
                    reachable = True
                    break
            if reachable:
                break
        if reachable:
            break
    ret_list = []
    ret_list.append([goal_x, goal_y, 0, 'goal', None, 'models_household/apple/apple.urdf'])
    ret_list.append([agent_x, agent_y, 0, 'agent', None, 'models_household/ball-small/ball.urdf'])
    for i in range(1, 6):
        ret_list.append([0, i, 0., 'block', step_function, 'models_household/block/block.urdf'])
        ret_list.append([i, 0, 0., 'block', step_function, 'models_household/block/block.urdf'])
        ret_list.append([6, i, 0., 'block', step_function, 'models_household/block/block.urdf'])
        ret_list.append([i, 6, 0., 'block', step_function, 'models_household/block/block.urdf'])
    for o in o_list:
        ret_list.append([o[0] + 1, o[1] + 1, 0, 'block', step_function, 'models_household/block/block.urdf'])
    # return: the object list, the maximum step allowed, and the actions allowed
    return ret_list, 10 + 200 * (current_dist - 1), ['a', 'w', 'd', 'c', 'z']
