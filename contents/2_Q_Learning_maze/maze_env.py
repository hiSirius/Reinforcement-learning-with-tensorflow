"""
Reinforcement learning maze example.
Red rectangle:          explorer.
Black rectangles:       hells       [reward = -1].
Yellow bin circle:      paradise    [reward = +1].
All other states:       ground      [reward = 0].
This script is the environment part of this example. The RL is in RL_brain.py.
View more on my tutorial page: https://morvanzhou.github.io/tutorials/
"""


import numpy as np
import time
import sys
if sys.version_info.major == 2:
    import Tkinter as tk
else:
    import tkinter as tk


UNIT = 40   # pixels
MAZE_H = 8  # grid height
MAZE_W = 8  # grid widt

# 1:障碍物， 9:寻找目标
maze = np.array(
    [
        [0,0,0,0,0,0,0,0],
        [0,1,1,1,0,1,1,0],
        [0,1,0,0,0,1,0,0],
        [0,0,0,9,0,1,1,0],
        [0,1,1,1,0,0,0,0],
        [0,1,0,0,0,1,0,0],
        [0,1,0,0,0,1,1,0],
        [0,0,0,0,0,0,0,0],
    ]
)

obstacleList = []

class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.title('maze')
        self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_H * UNIT))
        self._build_maze()

    def _build_maze(self):
        self.canvas = tk.Canvas(self, bg='white',
                           height=MAZE_H * UNIT,
                           width=MAZE_W * UNIT)

        # create grids
        for c in range(0, MAZE_W * UNIT, UNIT):
            x0, y0, x1, y1 = c, 0, c, MAZE_H * UNIT
            self.canvas.create_line(x0, y0, x1, y1)
        for r in range(0, MAZE_H * UNIT, UNIT):
            x0, y0, x1, y1 = 0, r, MAZE_W * UNIT, r
            self.canvas.create_line(x0, y0, x1, y1)

        # create origin
        #origin = np.array([20, 20])

        # 根据maze Array 中为1的点进行描绘
        # 并加入List 里
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] == 1:
                    self.hell = self.canvas.create_rectangle(UNIT*i,UNIT*j,UNIT*(i+1),UNIT*(j+1),fill='black')
                    obstacleList.append(self.canvas.coords(self.hell))
                elif maze[i][j] == 9:
                    #create oval
                    #oval_center = origin + UNIT * 2
                    self.oval = self.canvas.create_oval(
                    UNIT*6, UNIT*2,
                    UNIT*7, UNIT*3,
                    fill='yellow')

        #create red rect
        self.rect = self.canvas.create_rectangle(0, 0,UNIT, UNIT,fill='red')

        # pack all
        self.canvas.pack()

    def reset(self):
        self.update()
        time.sleep(0.5)
        self.canvas.delete(self.rect)
        #origin = np.array([20, 20])
        self.rect = self.canvas.create_rectangle(
            0, 0,
           UNIT, UNIT,
            fill='red')
        # return observation
        return self.canvas.coords(self.rect)

    def step(self, action):
        s = self.canvas.coords(self.rect)
        base_action = np.array([0, 0])
        if action == 0:   # up
            if s[1] > UNIT and maze[round(s[2]/UNIT) -1 ,  round((s[3]-UNIT)/UNIT) -1  ] !=1: #没有到边界，且路可走
                base_action[1] -= UNIT
        elif action == 1:   # down
            if s[1] < (MAZE_H - 1) * UNIT and  maze[round(s[2]/UNIT) - 1, round((s[3]+UNIT)/UNIT) -1] !=1: #没有到边界，且路可走
                base_action[1] += UNIT
        elif action == 2:   # right
            if s[0] < (MAZE_W - 1) * UNIT and  maze[round((s[2]+UNIT)/UNIT) -1, round(s[3]/UNIT) - 1] !=1: #没有到边界，且路可走
                base_action[0] += UNIT
        elif action == 3:   # left
            if s[0] > UNIT and  maze[round((s[2]-UNIT)/UNIT) -1, round(s[3]/UNIT) -1] !=1: #没有到边界，且路可走
                base_action[0] -= UNIT

        self.canvas.move(self.rect, base_action[0], base_action[1])  # move agent

        s_ = self.canvas.coords(self.rect)  # next state

        #  下一步是不是碰墙
        if obstacleList.count(s_) > 0 :
            return s, 0 ,False

        # reward function
        if s_ == self.canvas.coords(self.oval):
            reward = 1
            done = True
            s_ = 'terminal'
        # 如果碰壁了
        #elif obstacleList.count(s_) > 0 :
        #    reward = -1
        #    done = True
        #    s_ = 'terminal'
        else:
            reward = 0
            done = False

        return s_, reward, done

    def render(self):
        time.sleep(0.1)
        self.update()


def update():
    for t in range(10):
        s = env.reset()
        while True:
            env.render()
            a = 1
            s, r, done = env.step(a)
            if done:
                break

if __name__ == '__main__':
    env = Maze()
    env.after(100, update)
    env.mainloop()