using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Pathfinding
{
    public class JumpPointSearchPlus : IPathfinder
    {
        private readonly MinHeap _open = new(); // reused between searches so we don't keep reallocating it
        public string Name => "JPS+";
        private int[] _jumpLookup; // flat table: [cellIndex * 8 + dir]; eight directions per cell
        private GridMap _cachedMap;
        public void Preprocess(GridMap map)
        {
            if (map == null)
                throw new ArgumentNullException(nameof(map));
            
            _cachedMap = map;
            int n = map.CellCount;
            _jumpLookup = new int[n * 8];

            // Precompute straight directions (0, 2, 4, 6)
            // 0 = down; 2 = right; 4 = up; 6 = left
            for (int y = 0; y < map.Height; y++)
            {
                for (int x = 0; x < map.Width; x++)
                {
                    if (!map.IsOpen(x, y))
                        continue;
                    
                    int cellIndex = map.ToIndex(new Cell(x, y));

                    for (int dir = 0; dir < 8; dir += 2)
                    {
                        _jumpLookup[cellIndex * 8 + dir] = ComputeStraightJump(map, x, y, dir);
                    }
                }
            }
            
            // Precompute diagonal directions (1, 3, 5, 7)
            // 1 = down-right; 3 = up-right; 5 = up-left; 7 = down-left
            for (int y = 0; y < map.Height; y++)
            {
                for (int x = 0; x < map.Width; x++)
                {
                    if(!map.IsOpen(x, y))
                        continue;

                    int cellIndex = map.ToIndex(new Cell(x, y));

                    for (int dir = 1; dir < 8; dir += 2)
                    {
                        _jumpLookup[cellIndex * 8 + dir] = ComputeDiagonalJump(map, x, y, dir);
                    }
                }
            }
        }

        /// <summary>
        /// Computes the preprocessed jump distance from the specified cell in a straight direction for Jump Point Search Plus.
        /// </summary>
        /// <param name="map">The grid map used to validate movement and detect forced neighbors.</param>
        /// <param name="startX">The horizontal coordinate of the starting cell.</param>
        /// <param name="startY">The vertical coordinate of the starting cell.</param>
        /// <param name="dir">The straight direction index to scan.</param>
        /// <return>A positive step count if a jump point is found in the given direction; otherwise, the negative number of traversable steps before movement is blocked.</return>
        private int ComputeStraightJump(GridMap map, int startX, int startY, int dir)
        {
            int dx = GridMap.DirX[dir];
            int dy = GridMap.DirY[dir];
            int currentX = startX;
            int currentY = startY;
            int steps = 0;
            
            while (map.CanStep(currentX, currentY, dir))
            {
                currentX += dx;
                currentY += dy;
                steps++;
                
                Cell cell = new(currentX, currentY);
                if (HasForcedNeighbor(map, cell, dx, dy))
                    return steps; // Found jump point
            }

            return -steps;
        }

        /// <summary>
        /// Computes the preprocessed jump distance from the specified cell in a diagonal direction for Jump Point Search Plus.
        /// </summary>
        /// <param name="map">The grid map used to validate movement and inspect neighboring cells.</param>
        /// <param name="startX">The horizontal coordinate of the starting cell.</param>
        /// <param name="startY">The vertical coordinate of the starting cell.</param>
        /// <param name="dir">The diagonal direction index to scan.</param>
        /// <return>A positive step count if a jump point is found in the given direction; otherwise, the negative number of traversable steps before movement is blocked.</return>
        private int ComputeDiagonalJump(GridMap map, int startX, int startY, int dir)
        {
            int dx = GridMap.DirX[dir];
            int dy = GridMap.DirY[dir];
            int dirH = Direction(dx, 0);
            int dirV = Direction(0, dy);    
            
            int currentX = startX;
            int currentY = startY;
            int steps = 0;
            
            while (map.CanStep(currentX, currentY, dir))
            {
                currentX += dx;
                currentY += dy;
                steps++;
                
                Cell cell = new(currentX, currentY);
                if (HasForcedNeighbor(map, cell, dx, dy))
                    return steps; // Found jump point

                int nextIndex = map.ToIndex(cell);
                if (_jumpLookup[nextIndex * 8 + dirH] > 0 || _jumpLookup[nextIndex * 8 + dirV] > 0)
                    return steps;
            }

            return -steps;
        }

        /// <summary>
        /// Determines whether the specified cell creates a forced neighbor while scanning in the given jump direction.
        /// </summary>
        /// <param name="map">The grid containing the cell and its neighboring positions.</param>
        /// <param name="cell">The current cell being evaluated.</param>
        /// <param name="dx">Horizontal component of the scan direction.</param>
        /// <param name="dy">Vertical component of the scan direction.</param>
        /// <return><c>true</c> if the cell has a forced neighbor for the given straight movement direction; otherwise, <c>false</c>.</return>
        private static bool HasForcedNeighbor(GridMap map, Cell cell, int dx, int dy)
        {
            if (dx != 0 && dy != 0) // Diagonal: no corner cuts.
            {
                return false;
            } 
            
            if (dx != 0) // Horizontal: wall beside the previous cell, opening beside this one.
            {
                return (!map.IsOpen(cell.X - dx, cell.Y - 1) && map.IsOpen(cell.X, cell.Y - 1)) ||
                       (!map.IsOpen(cell.X - dx, cell.Y + 1) && map.IsOpen(cell.X, cell.Y + 1));
            }

            // On a vertical ray, perform the corresponding checks to the left and right.
            return (!map.IsOpen(cell.X - 1, cell.Y - dy) && map.IsOpen(cell.X - 1, cell.Y)) ||
                   (!map.IsOpen(cell.X + 1, cell.Y - dy) && map.IsOpen(cell.X + 1, cell.Y));
        }
        
        /// <summary>
        /// Converts a unit direction into the direction index used by <see cref="GridMap"/>.
        /// </summary>
        /// <param name="dx">Horizontal component of the direction.</param>
        /// <param name="dy">Vertical component of the direction.</param>
        /// <returns>The matching index in <see cref="GridMap.DirX"/> and <see cref="GridMap.DirY"/>.</returns>
        /// <exception cref="ArgumentOutOfRangeException">The components are not a non-zero unit step.</exception>
        private static int Direction(int dx, int dy)
        {
            // Locate the unit vector in GridMap's shared direction arrays.
            for (int dir = 0; dir < 8; dir++)
                if (GridMap.DirX[dir] == dx && GridMap.DirY[dir] == dy)
                    return dir;

            // All JPS directions must be one of the eight non-zero unit steps.
            throw new ArgumentOutOfRangeException(nameof(dx), "Direction must be a non-zero unit step.");
        }

        public static double Heuristic(Cell a, Cell b)
        {
            int dx = Math.Abs(a.X - b.X);
            int dy = Math.Abs(a.Y - b.Y);
            return Math.Max(dx, dy) + (GridMap.Sqrt2 - 1) * Math.Min(dx, dy);
        }

        public SearchResult FindPath(GridMap map, Cell start, Cell goal, SearchRecorder recorder = null)
        {
            // A missing map is a programming error; invalid endpoints produce an ordinary no-path result.
            if (map == null)
                throw new ArgumentNullException(nameof(map));
            
            if (_jumpLookup == null || _cachedMap != map) 
                Preprocess(map);
            
            var timer = Stopwatch.StartNew();   // Start the timer
            var result = new SearchResult();    // Returning the results
            
            if (!map.IsOpen(start) || !map.IsOpen(goal))
            {
                result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
                return result; // can't start or end inside a wall
            }
            
            // The zero-length route needs no queue processing.
            if (start == goal)
            {
                result.Found = true;
                result.PathCost = 0;
                result.Path.Add(start);
                result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
                return result;
            }

            int n = map.CellCount;
            var g = new double[n];  // cheapest cost found so far to reach each cell
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
                
                int parentIndex = cameFrom[current];
                Cell? parent = parentIndex >= 0 ? map.ToCell(parentIndex) : null;

                foreach (int dir in GetPrunedDirections(map, currentCell, parent))
                {
                    int dx = GridMap.DirX[dir];
                    int dy = GridMap.DirY[dir];
                    int jumpDist = _jumpLookup[current * 8 + dir];

                    // Check if the goal lies along this ray before or at the jump point / obstacle
                    Cell? successor = GetGoalOrJumpPoint(currentCell, goal, dir, dx, dy, jumpDist);
                    if (!successor.HasValue) continue;

                    Cell target = successor.Value;
                    int targetIndex = map.ToIndex(target);
                    if (closed[targetIndex]) continue;

                    double stepCost = Distance(currentCell, target);
                    double newG = g[current] + stepCost;
                    if (newG < g[targetIndex])
                    {
                        g[targetIndex] = newG;
                        cameFrom[targetIndex] = current;
                        _open.Push(targetIndex, newG + Heuristic(target, goal), newG);
                        recorder?.Opened(target);
                    }
                }
            }
            
            result.RunTimeMs = timer.ElapsedMilliseconds;
            return result;
        }
        
        private static List<Cell> BuildPath(GridMap map, int[] cameFrom, int goalIndex)
        {
            var path = new List<Cell>();
            for (int i = goalIndex; i != -1; i = cameFrom[i])
                path.Add(map.ToCell(i));
            path.Reverse();
            return path;
        }
        
        /// <summary>
        /// Calculates the movement cost of a straight or diagonal segment between jump points.
        /// </summary>
        /// <param name="from">The segment's starting jump point.</param>
        /// <param name="to">The segment's ending jump point.</param>
        /// <returns>The segment cost, charging one per straight step and sqrt(2) per diagonal step.</returns>
        private static double Distance(Cell from, Cell to)
        {
            // Jump points lie on a straight or diagonal segment; count each type of step.
            int dx = Math.Abs(from.X - to.X);
            int dy = Math.Abs(from.Y - to.Y);
            int diagonalSteps = Math.Min(dx, dy);
            int straightSteps = Math.Max(dx, dy) - diagonalSteps;
            // Apply the same movement costs as the map: sqrt(2) diagonally and 1 straight.
            return diagonalSteps * GridMap.Sqrt2 + straightSteps;
        }

        /// <summary>
        /// Determines the movement directions that should be considered from the current cell after applying Jump Point Search pruning rules.
        /// </summary>
        /// <param name="map">The grid map used to test traversability and identify forced neighbor directions.</param>
        /// <param name="current">The cell from which successor directions are being selected.</param>
        /// <param name="parent">The previous cell on the current path, or null when the current cell is the search start.</param>
        /// <returns>The direction indices that remain valid after natural-neighbor and forced-neighbor pruning.</returns>
        private static IEnumerable<int> GetPrunedDirections(GridMap map, Cell current, Cell? parent)
        {
            if (!parent.HasValue)
            {
                for (int dir = 0; dir < 8; dir++)
                    yield return dir;
                yield break;
            }

            int dx = Math.Sign(current.X - parent.Value.X);
            int dy = Math.Sign(current.Y - parent.Value.Y);

            if (dx != 0 && dy != 0)
            {
                yield return Direction(dx, dy);
                yield return Direction(dx, 0);
                yield return Direction(0, dy);

                if (!map.IsOpen(current.X - dx, current.Y) && map.IsOpen(current.X - dx, current.Y + dy))
                    yield return Direction(-dx, dy);
                if (!map.IsOpen(current.X, current.Y - dy) && map.IsOpen(current.X + dx, current.Y - dy))
                    yield return Direction(dx, -dy);
            }
            else if (dx != 0)
            {
                yield return Direction(dx, 0);

                if (!map.IsOpen(current.X - dx, current.Y + 1) && map.IsOpen(current.X, current.Y + 1))
                    yield return Direction(0, 1);
                if (!map.IsOpen(current.X - dx, current.Y - 1) && map.IsOpen(current.X, current.Y - 1))
                    yield return Direction(0, -1);
                if (!map.IsOpen(current.X, current.Y + 1) && map.IsOpen(current.X + dx, current.Y + 1))
                    yield return Direction(dx, 1);
                if (!map.IsOpen(current.X, current.Y - 1) && map.IsOpen(current.X + dx, current.Y - 1))
                    yield return Direction(dx, -1);
            }
            else
            {
                yield return Direction(0, dy);

                if (!map.IsOpen(current.X + 1, current.Y - dy) && map.IsOpen(current.X + 1, current.Y))
                    yield return Direction(1, 0);
                if (!map.IsOpen(current.X - 1, current.Y - dy) && map.IsOpen(current.X - 1, current.Y))
                    yield return Direction(-1, 0);
                if (!map.IsOpen(current.X + 1, current.Y) && map.IsOpen(current.X + 1, current.Y + dy))
                    yield return Direction(1, dy);
                if (!map.IsOpen(current.X - 1, current.Y) && map.IsOpen(current.X - 1, current.Y + dy))
                    yield return Direction(-1, dy);
            }
        }
        
        private static Cell? GetGoalOrJumpPoint(Cell current, Cell goal, int dir, int dx, int dy, int jumpDist)
        {
            int diffX = goal.X - current.X;
            int diffY = goal.Y - current.Y;

            // Straight ray: check if goal is along the ray
            if (dx == 0 || dy == 0)
            {
                if (dx != 0 && diffY == 0 && Math.Sign(diffX) == dx)
                {
                    int goalDist = Math.Abs(diffX);
                    int maxReach = jumpDist > 0 ? jumpDist : -jumpDist;
                    if (goalDist <= maxReach)
                        return goal;
                }
                else if (dy != 0 && diffX == 0 && Math.Sign(diffY) == dy)
                {
                    int goalDist = Math.Abs(diffY);
                    int maxReach = jumpDist > 0 ? jumpDist : -jumpDist;
                    if (goalDist <= maxReach)
                        return goal;
                }
            }
            // Diagonal ray: check if goal is directly on the diagonal ray or in the diagonal sector
            else
            {
                if (Math.Sign(diffX) == dx && Math.Sign(diffY) == dy)
                {
                    int diagSteps = Math.Min(Math.Abs(diffX), Math.Abs(diffY));
                    int maxReach = jumpDist > 0 ? jumpDist : -jumpDist;

                    if (Math.Abs(diffX) == Math.Abs(diffY) && diagSteps <= maxReach)
                        return goal;

                    // Goal reached after diagonal + straight
                    if (diagSteps <= maxReach && diagSteps > 0)
                    {
                        // Return the diagonal corner towards the goal
                        if (jumpDist > 0 && jumpDist < diagSteps)
                            return new Cell(current.X + dx * jumpDist, current.Y + dy * jumpDist);

                        return new Cell(current.X + dx * diagSteps, current.Y + dy * diagSteps);
                    }
                }
            }

            // Standard jump point lookup (if positive jump distance exists)
            if (jumpDist > 0)
                return new Cell(current.X + dx * jumpDist, current.Y + dy * jumpDist);

            return null;
        }
    }
}