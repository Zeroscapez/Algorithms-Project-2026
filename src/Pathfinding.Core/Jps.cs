using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Pathfinding
{
    // Jump Point Search, built from Abdulkerim's version in JPS but moved into the harness so it can be
    // compared with A* fairly

    public class Jps : IPathfinder
    {
        private readonly MinHeap _open = new MinHeap(); // reused between searches, same as AStar

        // Set at the start of each search so the jump helpers don't need them passed in every time.
        private GridMap _map;
        private int _goalX, _goalY;

        public string Name => "JPS";

        public void Preprocess(GridMap map)
        {
            // Nothing to do, plain JPS works straight off the grid. (JPS+ is the one that preprocesses.)
        }

        public SearchResult FindPath(GridMap map, Cell start, Cell goal, SearchRecorder recorder = null)
        {
            var timer = Stopwatch.StartNew();
            var result = new SearchResult();

            if (!map.IsOpen(start) || !map.IsOpen(goal))
            {
                result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
                return result; // can't start or end inside a wall
            }

            _map = map;
            _goalX = goal.X;
            _goalY = goal.Y;

            // Same flat-array bookkeeping as AStar.
            int n = map.CellCount;
            var g = new double[n];
            var cameFrom = new int[n];
            var closed = new bool[n];
            for (int i = 0; i < n; i++)
            {
                g[i] = double.PositiveInfinity;
                cameFrom[i] = -1;
            }

            int startIndex = map.ToIndex(start);
            int goalIndex = map.ToIndex(goal);
            g[startIndex] = 0;
            _open.Clear();
            _open.Push(startIndex, AStar.Heuristic(start, goal), 0);
            recorder?.Opened(start);

            var successors = new List<int>(8);

            while (_open.Count > 0)
            {
                int current = _open.Pop();
                if (closed[current]) continue; // old duplicate entry
                closed[current] = true;
                result.NodesExpanded++;

                Cell currentCell = map.ToCell(current);
                recorder?.Expanded(currentCell);

                if (current == goalIndex)
                {
                    result.Found = true;
                    result.PathCost = g[goalIndex];
                    result.Path = BuildPath(map, cameFrom, goalIndex);
                    break;
                }

                FindSuccessors(currentCell, cameFrom[current], successors);

                foreach (int next in successors)
                {
                    if (closed[next]) continue;

                    // Jump points are always in a straight or diagonal line from here,
                    // so the octile distance is the exact cost of getting there.
                    Cell nextCell = map.ToCell(next);
                    double newG = g[current] + AStar.Heuristic(currentCell, nextCell);
                    if (newG < g[next])
                    {
                        g[next] = newG;
                        cameFrom[next] = current;
                        _open.Push(next, newG + AStar.Heuristic(nextCell, goal), newG);
                        recorder?.Opened(nextCell);
                    }
                }
            }

            _map = null;
            result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
            return result;
        }

        // Pruning: works out which directions are worth jumping in from this node, based on the direction
        // we came in from. Any direction not tried here can be reached at least as cheaply some other way.
        private void FindSuccessors(Cell c, int parentIndex, List<int> successors)
        {
            successors.Clear();
            int x = c.X, y = c.Y;

            if (parentIndex == -1)
            {
                // The start node has no direction yet, so try all 8.
                for (int dir = 0; dir < 8; dir++)
                    AddJump(successors, x, y, GridMap.DirX[dir], GridMap.DirY[dir]);
                return;
            }

            Cell parent = _map.ToCell(parentIndex);
            int dx = Math.Sign(x - parent.X);
            int dy = Math.Sign(y - parent.Y);

            if (dx != 0 && dy != 0)
            {
                // Came in diagonally: keep going diagonally, or along either of its two straight parts.
                // With no corner cutting there are no forced neighbors on a diagonal move.
                AddJump(successors, x, y, dx, 0);
                AddJump(successors, x, y, 0, dy);
                AddJump(successors, x, y, dx, dy);
            }
            else if (dx != 0)
            {
                // Came in horizontally: keep going. If the cell above/below is open but the one behind it
                // is a wall, the parent couldn't have reached it directly, so we have to turn here (forced).
                AddJump(successors, x, y, dx, 0);
                for (int side = -1; side <= 1; side += 2)
                {
                    if (_map.IsOpen(x, y + side) && !_map.IsOpen(x - dx, y + side))
                    {
                        AddJump(successors, x, y, 0, side);
                        AddJump(successors, x, y, dx, side);
                    }
                }
            }
            else
            {
                // Came in vertically: same thing with x and y swapped.
                AddJump(successors, x, y, 0, dy);
                for (int side = -1; side <= 1; side += 2)
                {
                    if (_map.IsOpen(x + side, y) && !_map.IsOpen(x + side, y - dy))
                    {
                        AddJump(successors, x, y, side, 0);
                        AddJump(successors, x, y, side, dy);
                    }
                }
            }
        }

        private void AddJump(List<int> successors, int x, int y, int dx, int dy)
        {
            int jumpPoint = Jump(x, y, dx, dy);
            if (jumpPoint != -1 && !successors.Contains(jumpPoint))
                successors.Add(jumpPoint);
        }


        private int Jump(int x, int y, int dx, int dy)
        {
            while (true)
            {
                if (!CanMove(x, y, dx, dy)) return -1;
                x += dx;
                y += dy;

                if (x == _goalX && y == _goalY) return y * _map.Width + x;

                if (dx != 0 && dy != 0)
                {
                    // Diagonal: this cell is a jump point if a straight jump from here finds anything.
                    if (Jump(x, y, dx, 0) != -1 || Jump(x, y, 0, dy) != -1)
                        return y * _map.Width + x;
                }
                else if (dx != 0)
                {
                    // Horizontal: stop if there's a forced neighbor above or below.
                    if ((_map.IsOpen(x, y - 1) && !_map.IsOpen(x - dx, y - 1)) ||
                        (_map.IsOpen(x, y + 1) && !_map.IsOpen(x - dx, y + 1)))
                        return y * _map.Width + x;
                }
                else
                {
                    // Vertical: stop if there's a forced neighbor left or right.
                    if ((_map.IsOpen(x - 1, y) && !_map.IsOpen(x - 1, y - dy)) ||
                        (_map.IsOpen(x + 1, y) && !_map.IsOpen(x + 1, y - dy)))
                        return y * _map.Width + x;
                }
            }
        }

        // Same rule as GridMap.CanStep: a diagonal needs both cells beside it open (no corner cutting).
        private bool CanMove(int x, int y, int dx, int dy)
        {
            if (!_map.IsOpen(x + dx, y + dy)) return false;
            if (dx != 0 && dy != 0)
                return _map.IsOpen(x + dx, y) && _map.IsOpen(x, y + dy);
            return true;
        }

        // Returns just the jump points, start and goal included. Each step between two of them is a
        // straight or diagonal line, so Unity (or anything else) can fill in the cells if it needs them.
        private static List<Cell> BuildPath(GridMap map, int[] cameFrom, int goalIndex)
        {
            var path = new List<Cell>();
            for (int i = goalIndex; i != -1; i = cameFrom[i])
                path.Add(map.ToCell(i));
            path.Reverse();
            return path;
        }
    }
}
