using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Pathfinding
{
    // Plain A* This is the baseline JPS and JPS+ get compared against.
    public class AStar : IPathfinder
    {
        private readonly MinHeap _open = new MinHeap(); // reused between searches so we don't keep reallocating it

        public string Name => "A*";

        public void Preprocess(GridMap map)
        {
            // Nothing to do, A* doesn't preprocess.
        }

        // Octile distance: the exact path length if there were no walls at all.
        // It never guesses too high, so A* still finds the optimal path.
        public static double Heuristic(Cell a, Cell b)
        {
            int dx = Math.Abs(a.X - b.X);
            int dy = Math.Abs(a.Y - b.Y);
            return Math.Max(dx, dy) + (GridMap.Sqrt2 - 1) * Math.Min(dx, dy);
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

            // Per-cell bookkeeping, as flat arrays indexed by y * Width + x (way faster than dictionaries).
            int n = map.CellCount;
            var g = new double[n];       // cheapest cost found so far to reach each cell
            var cameFrom = new int[n];   // which cell we got here from, for rebuilding the path
            var closed = new bool[n];    // the closed set: already expanded, done with it
            for (int i = 0; i < n; i++)
            {
                g[i] = double.PositiveInfinity;
                cameFrom[i] = -1;
            }

            int startIndex = map.ToIndex(start);
            int goalIndex = map.ToIndex(goal);
            g[startIndex] = 0;
            _open.Clear();
            _open.Push(startIndex, Heuristic(start, goal), 0);
            recorder?.Opened(start);

            while (_open.Count > 0)
            {
                int current = _open.Pop();
                if (closed[current]) continue; // old duplicate entry, we already found a cheaper way here
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

                // Look at all 8 neighbors.
                for (int dir = 0; dir < 8; dir++)
                {
                    if (!map.CanStep(currentCell.X, currentCell.Y, dir)) continue;

                    var nextCell = new Cell(currentCell.X + GridMap.DirX[dir], currentCell.Y + GridMap.DirY[dir]);
                    int next = map.ToIndex(nextCell);
                    if (closed[next]) continue;

                    double newG = g[current] + GridMap.StepCost(dir);
                    if (newG < g[next]) // found a cheaper way to get there
                    {
                        g[next] = newG;
                        cameFrom[next] = current;
                        _open.Push(next, newG + Heuristic(nextCell, goal), newG);
                        recorder?.Opened(nextCell);
                    }
                }
            }

            result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
            return result;
        }

        // Walk backwards from the goal following cameFrom, then flip it around.
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
