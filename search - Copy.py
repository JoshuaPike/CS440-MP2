# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains search functions.
"""
# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (alpha, beta, gamma) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,astar)
# You may need to slight change your previous search functions in MP1 since this is 3-d maze

from collections import deque
from heapq import heappop, heappush
from queue import Queue

def search(maze, searchMethod):
    return {
        "bfs": bfs,
    }.get(searchMethod, [])(maze)

def bfs(maze):
    # Write your code here
    """
    This function returns optimal path in a list, which contains start and objective.
    If no path found, return None. 
    """
    q = Queue(maxsize = 0)
    visited = {}
    goals = maze.getObjectives()

    start = maze.getStart()
    q.put([start, start])

    toRet = []

    while not q.empty():
        cur = q.get()
        if cur[0] in visited.keys():
            continue
        
        visited[cur[0]] = cur[1]

        if cur[0] in goals:
            print("A goal has been found\n")
            # goals.remove(cur[0])
            recurse = cur[0]
            
            while recurse != start:
                toRet.append(recurse)
                recurse = visited[recurse]
            toRet.append(start)
            toRet.reverse()
            # visited.clear()

            # while len(goals) != 0:
            #     pathAdd = []
            #     pathAdd = bfsHelper(maze, toRet[len(toRet) - 1], goals)
            #     toRet = toRet + pathAdd
            
            print(maze.isValidPath(toRet))
            return toRet

        for i in maze.getNeighbors(cur[0][0], cur[0][1]):
            if maze.isValidMove(i[0], i[1]):
                if i not in visited.keys():
                    q.put([i, cur[0]])

    return None