import numpy as np
from colorama import Fore, Back, Style

map = np.array([[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                [1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1],
                [1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1],
                [1,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1],
                [1,1,0,1,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,1],
                [1,1,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,1],
                [1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1],
                [1,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,1],
                [1,1,0,0,1,0,1,0,1,0,1,1,1,1,1,1,1,0,1,1],
                [1,1,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1],
                [1,1,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1],
                [1,1,0,0,1,0,0,0,1,1,1,1,1,1,1,1,1,0,1,1],
                [1,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,1,1],
                [1,1,0,0,1,0,0,0,0,0,0,0,0,1,1,0,1,0,1,1],
                [1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,0,1,0,1,1],
                [1,1,0,0,0,1,0,0,0,0,1,0,0,0,1,0,1,0,1,1],
                [1,1,0,1,0,1,0,1,0,0,1,1,1,0,1,0,1,0,1,1],
                [1,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,1,1],
                [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
                [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])

start = np.array([18, 1])
finish = np.array([2, 18])

for i in map:
    for j in i:
        if j == 1:
            print(Back.WHITE + " " + str(j), end="")
        else:
            print(Back.BLACK + " " + str(j), end="")
    print(Back.BLACK + "")

first_number = 2

def frontwave(xy, first_number):
    if xy[0] < 0 or xy[0] > 19:
        return 0
    if xy[1] < 0 or xy[1] > 19:
        return 0
    if map[xy[0]][xy[1]] == 1:
        return 0
    if map[xy[0]][xy[1]] != 0:
        return 0
    if map[xy[0]][xy[1]] == 0:
        map[xy[0]][xy[1]] = first_number
        frontwave(np.array([xy[0], xy[1] + 1]), first_number + 1)
        frontwave(np.array([xy[0], xy[1] - 1]), first_number + 1)
        frontwave(np.array([xy[0] + 1, xy[1]]), first_number + 1)
        frontwave(np.array([xy[0] - 1, xy[1]]), first_number + 1)

    
frontwave(finish, 2)

print("")

for i in map:
    for j in i:
        if j == 1:
            print(Back.WHITE + " " + str(j), end="")
        else:
            if j < 10:
                print(Back.BLACK + " " + str(j), end="")
            elif j < 100:
                print(Back.BLACK + "" + str(j), end="")
            else:
                print(Back.BLACK + "" + str(j), end="")
    print(Back.BLACK + "")

x = start[0]
y = start[1]
way = [[x, y]]
while map[x][y] != 2:
    if map[x + 1][y] < map[x][y] and map[x + 1][y] != 1:
        x = x + 1
        way.append([x, y])
        continue
    if map[x - 1][y] < map[x][y] and map[x - 1][y] != 1:
        x = x - 1
        way.append([x, y])
        continue
    if map[x][y + 1] < map[x][y] and map[x][y + 1] != 1:
        y = y + 1
        way.append([x, y])
        continue
    if map[x][y - 1] < map[x][y] and map[x][y - 1] != 1:
        y = y - 1
        way.append([x, y])
        continue

print(way)

for elem in way:
    elem[0] = -(elem[0]*2.5 + 1.25 - 25.57)
    elem[1] = elem[1]*2.5 + 1.25 - 25.72

print("")
print(way)