using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Pathfinding
{
    public class JpsPlus : IPathfinder
    {
        private readonly MinHeap _open = new(); // reused between searches, same as AStar and JPS
        public string Name => "JPS+";
        private int[] _jumpLookup; // flat table: [cellIndex * 8 + dir]; eight directions per cell
        private GridMap _cachedMap; // The cached map for the current search.
        
        // The directions for each cell in the map.
        private const int 
            Up = 0, 
            UpRight = 1, 
            Right = 2, 
            DownRight = 3,
            Down = 4, 
            DownLeft = 5, 
            Left = 6, 
            UpLeft = 7;

        /// <summary>
        /// Precomputes JPS+ lookup data for the specified grid map, caching jump distances
        /// for each open cell in all straight and diagonal directions.
        /// </summary>
        /// <param name="map">The grid map to preprocess.</param>
        /// <exception cref="ArgumentNullException">Thrown when <paramref name="map"/> is null.</exception>
        public void Preprocess(GridMap map)
        {
            _cachedMap = map ?? throw new ArgumentNullException(nameof(map));

            int n = map.CellCount; // Save the number of cells in the map.
            _jumpLookup = new int[n * 8]; // Save a lookup table for jump points.

            // Precompute straight directions (0, 2, 4, 6) for every cell in the map.
            for (int y = 0; y < map.Height; y++)
            {
                for (int x = 0; x < map.Width; x++)
                {
                    if (!map.IsOpen(x, y))
                        continue; // Skip cells that are not open.
                    
                    int cellIndex = map.ToIndex(new Cell(x, y)); // Save the index of the current cell.

                    for (int dir = 0; dir < 8; dir += 2) // Loop through every straight direction.
                    {
                        _jumpLookup[cellIndex * 8 + dir] = ComputeStraightJump(map, x, y, dir);
                    }
                }
            }
            
            // Precompute diagonal directions (1, 3, 5, 7) for every cell in the map.
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
        /// Computes the preprocessed jump distance from the specified cell in a straight direction for JPS+.
        /// </summary>
        /// <param name="map">The grid map used to validate movement and detect forced neighbors.</param>
        /// <param name="startX">The horizontal coordinate of the starting cell.</param>
        /// <param name="startY">The vertical coordinate of the starting cell.</param>
        /// <param name="dir">The straight direction index to scan.</param>
        /// <return>A positive step count if a jump point is found in the given direction; otherwise,
        /// the negative number of traversable steps before movement is blocked.</return>
        private int ComputeStraightJump(GridMap map, int startX, int startY, int dir)
        {
            int dx = GridMap.DirX[dir];
            int dy = GridMap.DirY[dir];
            int currentX = startX;
            int currentY = startY;
            int steps = 0;
            
            // Continue scanning until movement is blocked.
            while (map.CanStep(currentX, currentY, dir))
            {
                currentX += dx;
                currentY += dy;
                steps++;
                
                Cell cell = new(currentX, currentY);
                if (HasForcedNeighbor(map, cell, dx, dy))   // Check if the current cell has a forced neighbor in the specified direction.
                    return steps; // Found jump point
            }

            return -steps;  // Return the negative number of traversable steps before movement is blocked.
        }

        /// <summary>
        /// Computes the preprocessed jump distance from the specified cell in a diagonal direction for JPS+.
        /// </summary>
        /// <param name="map">The grid map used to validate movement and inspect neighboring cells.</param>
        /// <param name="startX">The horizontal coordinate of the starting cell.</param>
        /// <param name="startY">The vertical coordinate of the starting cell.</param>
        /// <param name="dir">The diagonal direction index to scan.</param>
        /// <return>A positive step count if a jump point is found in the given direction; otherwise,
        /// the negative number of traversable steps before movement is blocked.</return>
        private int ComputeDiagonalJump(GridMap map, int startX, int startY, int dir)
        {
            int dx = GridMap.DirX[dir];
            int dy = GridMap.DirY[dir];
            int dirH = Direction(dx, 0);    // get the horizontal direction.
            int dirV = Direction(0, dy);    // get the vertical direction.
            
            int currentX = startX;
            int currentY = startY;
            int steps = 0;
            
            while (map.CanStep(currentX, currentY, dir))
            {
                currentX += dx;
                currentY += dy;
                steps++;
                
                Cell cell = new(currentX, currentY);
                int nextIndex = map.ToIndex(cell);
                
                // Check the jump lookup table for straight jumps finding something.
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
        /// <return><c>true</c> if the cell has a forced neighbor for the given straight movement direction;
        /// otherwise, <c>false</c>.</return>
        private static bool HasForcedNeighbor(GridMap map, Cell cell, int dx, int dy)
        {
            if (dx != 0 && dy != 0) // Diagonal: No corner cutting. ComputeDiagonalJump() handles this.
            {
                return false;
            } 
            
            // Horizontal: perform the corresponding checks to the above and below.
            if (dx != 0)
            {
                return (!map.IsOpen(cell.X - dx, cell.Y - 1) && map.IsOpen(cell.X, cell.Y - 1)) ||
                       (!map.IsOpen(cell.X - dx, cell.Y + 1) && map.IsOpen(cell.X, cell.Y + 1));
            }

            // Vertical: perform the corresponding checks to the left and right.
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

        /// <summary>
        /// Searches for a shortest path and returns every cells jumped to towards the path.
        /// </summary>
        /// <param name="map">The grid to search.</param>
        /// <param name="start">The walkable starting cell.</param>
        /// <param name="goal">The walkable destination cell.</param>
        /// <param name="recorder">Optional recipient for open and expanded-cell events.</param>
        /// <returns>The search outcome, including the path and its cost when found.</returns>
        public SearchResult FindPath(GridMap map, Cell start, Cell goal, SearchRecorder recorder = null)
        {
            // A missing map is a programming error; invalid endpoints produce an ordinary no-path result.
            if (map == null)
                throw new ArgumentNullException(nameof(map));
            
            // If the jump lookup table is missing or the cached map is not the same as the provided map, preprocess the map.
            if (_jumpLookup == null || _cachedMap != map) 
                Preprocess(map);
            
            var timer = Stopwatch.StartNew();   // Start the timer
            var result = new SearchResult();    // Returning the results
            
            if (!map.IsOpen(start) || !map.IsOpen(goal))
            {
                result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
                return result; // can't start or end inside a wall
            }
            
            int n = map.CellCount;
            var g = new double[n];       // cheapest cost found so far to reach each cell
            var cameFrom = new int[n];   // which cell we got here from, for rebuilding the path
            var arrivedDir = new int[n]; // which direction we arrived from to reach this cell
            var closed = new bool[n];    // the closed set: already expanded, done with it
            for (int i = 0; i < n; i++)
            {
                g[i] = double.PositiveInfinity;
                cameFrom[i] = -1;
                arrivedDir[i] = -1;
            }
                        
            int startIndex = map.ToIndex(start); 
            int goalIndex = map.ToIndex(goal);
            g[startIndex] = 0;
            _open.Clear();
            _open.Push(startIndex, AStar.Heuristic(start, goal), 0);
            recorder?.Opened(start);
            
            while (_open.Count > 0)
            {
                int currentIndex = _open.Pop();
                if (closed[currentIndex]) continue; // old duplicate entry, we already found a cheaper way here
                closed[currentIndex] = true;
                result.NodesExpanded++;
                
                Cell currentCell = map.ToCell(currentIndex);
                recorder?.Expanded(currentCell);
                
                if (currentIndex == goalIndex)
                {
                    result.Found = true;
                    result.PathCost = g[goalIndex];
                    result.Path = BuildPath(map, cameFrom, goalIndex);
                    break;
                }
                
                // Where the goal is relative to the current cell.
                int gx = goal.X - currentCell.X;
                int gy = goal.Y - currentCell.Y;

                foreach (int dir in DirectionsToTry(arrivedDir[currentIndex]))
                {
                    int dx = GridMap.DirX[dir];
                    int dy = GridMap.DirY[dir];

                    if (_jumpLookup != null)
                    {
                        // Look up the precomputed jump distance and check when to stop in case the goal is on the way.
                        int d = _jumpLookup[currentIndex * 8 + dir];
                        int steps = StepsToStop(d, dx, dy, gx, gy);
                    
                        // Skip this direction if we can't take steps.
                        if (steps == 0)
                        {
                            continue;
                        }  
                    
                        var nextCell = new Cell(currentCell.X + dx * steps, currentCell.Y + dy * steps);
                        int nextIndex = map.ToIndex(nextCell);
                        if (closed[nextIndex])
                        {
                            continue;
                        }
                    
                        double newG = g[currentIndex] + steps * GridMap.StepCost(dir); 
                        if (newG < g[nextIndex])
                        {
                            g[nextIndex] = newG;
                            cameFrom[nextIndex] = currentIndex;
                            arrivedDir[nextIndex] = dir;
                            _open.Push(nextIndex, newG + AStar.Heuristic(nextCell, goal), newG);
                            recorder?.Opened(nextCell);
                        }
                    }
                    else
                    {
                        throw new InvalidOperationException("Jump lookup table was not initialized!");
                    }
                }
            }
            
            result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
            return result;
        }

        /// <summary>
        /// Gets the movement directions that should be evaluated next based on the direction used to enter the current node.
        /// </summary>
        /// <param name="arrivedDir">The direction index used to arrive at the current node, or a negative value for the start node.</param>
        /// <return>The ordered set of direction indices to consider for further JPS+ expansion.</return>
        private static int[] DirectionsToTry(int arrivedDir) =>
            DirsByArrival[arrivedDir < 0 ? 8 : arrivedDir];

        /// <summary>
        /// Pruning: Maps the direction used to enter a node to the set of movement directions that should be evaluated next by JPS+.
        /// </summary>
        private static readonly int[][] DirsByArrival =
        {
            new[] { Left, UpLeft, Up, UpRight, Right },                             // arrived Up
            new[] { Up, UpRight, Right },                                           // arrived UpRight
            new[] { Up, UpRight, Right, DownRight, Down },                          // arrived Right
            new[] { Right, DownRight, Down },                                       // arrived DownRight
            new[] { Right, DownRight, Down, DownLeft, Left },                       // arrived Down
            new[] { Down, DownLeft, Left },                                         // arrived DownLeft
            new[] { Down, DownLeft, Left, UpLeft, Up },                             // arrived Left
            new[] { Left, UpLeft, Up },                                             // arrived UpLeft
            new[] { Up, UpRight, Right, DownRight, Down, DownLeft, Left, UpLeft },  // start node
        };

        /// <summary>
        /// Determines how many steps to advance along a jump direction before stopping.
        /// </summary>
        /// <param name="d">The precomputed signed distance to the next jump point or wall in the current direction.</param>
        /// <param name="dx">Horizontal component of the movement direction.</param>
        /// <param name="dy">Vertical component of the movement direction.</param>
        /// <param name="gx">Horizontal offset from the current cell to the goal cell.</param>
        /// <param name="gy">Vertical offset from the current cell to the goal cell.</param>
        /// <return>The distance to the goal if it lies on the current ray within reach; otherwise, the positive jump distance, or zero when no jump point is reachable.</return>
        private static int StepsToStop(int d, int dx, int dy, int gx, int gy)
        {
            // Track the distance to the jump point or wall
            int reach = Math.Abs(d);

            // If the ray is straight
            if (dx == 0 || dy == 0)
            {
                // Check if the goal is straight in reach
                bool goalOnRay = dx == 0 
                    ? gx == 0 && Math.Sign(gy) == dy
                    : gy == 0 && Math.Sign(gx) == dx;
                
                int goalDist = Math.Abs(gx) + Math.Abs(gy);
                
                // If the goal is on the ray and within reach, return the distance to it.
                if (goalOnRay && goalDist <= reach)
                    return goalDist;
            }
            // If the ray is diagonal
            else
            {
                // Check if we are headed to the goal. We can then try to reach it.
                bool onGoalLevel = Math.Sign(gx) == dx && Math.Sign(gy) == dy;
                int goalLine = Math.Min(Math.Abs(gx), Math.Abs(gy));
                if (onGoalLevel && goalLine <= reach)
                    return goalLine;
            }

            // Return the distance to the closest point on the ray or 0.
            return d > 0 ? d : 0;
        }

        /// <summary>
        /// Returns just the jump points, start and goal included. Each step between two of them is a
        /// straight or diagonal line, so Unity (or anything else) can fill in the cells if it needs them.
        /// </summary>
        /// <param name="map">The grid map used to convert cell indices into cells.</param>
        /// <param name="cameFrom">The predecessor index array.</param>
        /// <param name="goalIndex">The index of the goal cell.</param>
        /// <returns>A list of cells: the start, the jump points in between, and the goal.</returns>
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