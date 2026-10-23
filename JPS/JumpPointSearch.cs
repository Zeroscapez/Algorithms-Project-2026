using System;
using System.Collections.Generic;

/// <summary>
/// Finds a shortest path on a two-dimensional, eight-directional grid using
/// Jump Point Search. A true grid cell is walkable; false represents an obstacle.
/// </summary>
public class JumpPointSearch
{
    private readonly bool[,] _grid;
    private readonly int _width;
    private readonly int _height;

    public JumpPointSearch(bool[,] grid)
    {
        // Require a grid so the search can safely inspect its dimensions and cells.
        ArgumentNullException.ThrowIfNull(grid);

        // Keep the supplied walkability map for future movement checks.
        _grid = grid;

        // Cache the grid bounds; X is the first array dimension and Y is the second.
        _width = grid.GetLength(0);
        _height = grid.GetLength(1);
    }

    /// <summary>
    /// A node in the A* open list. JPS stores only turning points as nodes;
    /// the complete sequence of grid cells is filled in after the search.
    /// </summary>
    public struct Node : IComparable<Node>
    {
        public int X, Y;
        public float G, H, F;

        public int CompareTo(Node other) => F.CompareTo(other.F);
    }

    /// <summary>
    /// Returns the path from start to goal, including both endpoints, or null
    /// if either endpoint is blocked/out of bounds or no path exists.
    /// Diagonal movement is allowed when at least one of the two adjacent
    /// orthogonal cells is walkable; this prevents squeezing through blocked corners.
    /// </summary>
    public List<Tuple<int, int>>? FindPath(Tuple<int, int> start, Tuple<int, int> goal)
    {
        // Throw a clear argument error instead of failing while reading coordinates.
        ArgumentNullException.ThrowIfNull(start);
        ArgumentNullException.ThrowIfNull(goal);

        // A route is impossible if either endpoint is blocked or outside the grid.
        if (!IsValid(start.Item1, start.Item2) || !IsValid(goal.Item1, goal.Item2))
            return null;

        // Convert the coordinates to keys used by the search's lookup collections.
        var startPosition = Tuple.Create(start.Item1, start.Item2);
        var goalPosition = Tuple.Create(goal.Item1, goal.Item2);

        // The one-cell path is already complete when start and goal are equal.
        if (startPosition.Equals(goalPosition))
            return new List<Tuple<int, int>> { startPosition };

        // Open contains jump points waiting to be explored, ordered by estimated
        // route cost. Closed tracks points already explored.
        var openList = new List<Node>();
        var closedSet = new HashSet<Tuple<int, int>>();

        // Record the cheapest known cost from start to each jump point.
        var gScores = new Dictionary<Tuple<int, int>, float>
        {
            [startPosition] = 0
        };

        // Record each jump point's predecessor so the full route can be rebuilt.
        var parents = new Dictionary<Tuple<int, int>, Tuple<int, int>>();

        // The start has no travel cost yet; its priority is just the estimated
        // remaining distance to the goal.
        float startH = Heuristic(start.Item1, start.Item2, goal.Item1, goal.Item2);
        openList.Add(new Node
        {
            X = start.Item1,
            Y = start.Item2,
            G = 0,
            H = startH,
            F = startH
        });

        while (openList.Count > 0)
        {
            // Sorting keeps this implementation simple; a priority queue can
            // replace it if the open list becomes a performance bottleneck.
            openList.Sort();
            Node current = openList[0];
            openList.RemoveAt(0);

            var currentPosition = Tuple.Create(current.X, current.Y);

            // An improved route may have added a newer copy to the open list.
            if (!gScores.TryGetValue(currentPosition, out float bestG) || current.G > bestG)
                continue;

            if (currentPosition.Equals(goalPosition))
                return ReconstructPath(goalPosition, parents);

            closedSet.Add(currentPosition);

            foreach (Tuple<int, int> successor in GetSuccessors(current, goalPosition, parents))
            {
                if (closedSet.Contains(successor))
                    continue;

                float tentativeG = current.G +
                    Distance(current.X, current.Y, successor.Item1, successor.Item2);

                if (gScores.TryGetValue(successor, out float existingG) &&
                    tentativeG >= existingG)
                {
                    continue;
                }

                float heuristic = Heuristic(successor.Item1, successor.Item2, goal.Item1, goal.Item2);
                gScores[successor] = tentativeG;
                parents[successor] = currentPosition;
                openList.Add(new Node
                {
                    X = successor.Item1,
                    Y = successor.Item2,
                    G = tentativeG,
                    H = heuristic,
                    F = tentativeG + heuristic
                });
            }
        }

        return null;
    }

    /// <summary>
    /// Finds the natural and forced directions from a jump point, then jumps
    /// in each direction until another jump point or an obstacle is reached.
    /// </summary>
    private List<Tuple<int, int>> GetSuccessors(
        Node current,
        Tuple<int, int> goal,
        Dictionary<Tuple<int, int>, Tuple<int, int>> parents)
    {
        // Successors are the next jump points reached along promising directions.
        var successors = new List<Tuple<int, int>>();
        var currentPosition = Tuple.Create(current.X, current.Y);

        // The start has no incoming direction, so every legal direction is
        // considered. At later nodes, parent-to-current determines pruning.
#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type.
        if (!parents.TryGetValue(currentPosition, out Tuple<int, int> parent))
        {
            // With no parent, this is the start node and there is no direction
            // to use for pruning, so try all eight neighboring directions.
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx != 0 || dy != 0)
                        AddJumpSuccessor(successors, current, dx, dy, goal);
                }
            }

            return successors;
        }
#pragma warning restore CS8600 // Converting null literal or possible null value to non-nullable type.

        // Reduce the parent-to-current offset to a unit direction. This tells
        // JPS which moves are natural and which moves could be forced by walls.
        int directionX = Math.Sign(current.X - parent.Item1);
        int directionY = Math.Sign(current.Y - parent.Item2);

        if (directionX != 0 && directionY != 0)
        {
            // On a diagonal arrival, continue diagonally or along either
            // straight component of that diagonal.
            AddJumpSuccessor(successors, current, directionX, directionY, goal);
            AddJumpSuccessor(successors, current, directionX, 0, goal);
            AddJumpSuccessor(successors, current, 0, directionY, goal);

            // If the cell behind one side is blocked but the diagonal escape
            // is open, include that otherwise-pruned forced direction.
            if (!IsValid(current.X - directionX, current.Y) &&
                IsValid(current.X - directionX, current.Y + directionY))
            {
                AddJumpSuccessor(successors, current, -directionX, directionY, goal);
            }

            // Check the equivalent forced turn for the other side of the path.
            if (!IsValid(current.X, current.Y - directionY) &&
                IsValid(current.X + directionX, current.Y - directionY))
            {
                AddJumpSuccessor(successors, current, directionX, -directionY, goal);
            }
        }
        else if (directionX != 0)
        {
            // On a horizontal run, continue forward. A wall immediately
            // above or below can force a diagonal move around its edge.
            AddJumpSuccessor(successors, current, directionX, 0, goal);

            if (!IsValid(current.X, current.Y + 1) &&
                IsValid(current.X + directionX, current.Y + 1))
            {
                AddJumpSuccessor(successors, current, directionX, 1, goal);
            }

            if (!IsValid(current.X, current.Y - 1) &&
                IsValid(current.X + directionX, current.Y - 1))
            {
                AddJumpSuccessor(successors, current, directionX, -1, goal);
            }
        }
        else
        {
            // On a vertical run, continue forward and apply the same forced
            // turn checks with X and Y exchanged.
            AddJumpSuccessor(successors, current, 0, directionY, goal);

            if (!IsValid(current.X + 1, current.Y) &&
                IsValid(current.X + 1, current.Y + directionY))
            {
                AddJumpSuccessor(successors, current, 1, directionY, goal);
            }

            if (!IsValid(current.X - 1, current.Y) &&
                IsValid(current.X - 1, current.Y + directionY))
            {
                AddJumpSuccessor(successors, current, -1, directionY, goal);
            }
        }

        // Return only directions that produced a reachable jump point.
        return successors;
    }

    private void AddJumpSuccessor(
        List<Tuple<int, int>> successors,
        Node current,
        int dx,
        int dy,
        Tuple<int, int> goal)
    {
        // Do not start a jump in a direction whose first move is blocked or
        // would pass through a disallowed diagonal corner.
        if (!CanMove(current.X, current.Y, dx, dy))
            return;

        // Search along this direction and keep only its next significant point.
        Tuple<int, int>? jumpPoint = Jump(current.X, current.Y, dx, dy, goal);

        // Jump rays may converge on the same point; add it only once.
        if (jumpPoint != null && !successors.Contains(jumpPoint))
            successors.Add(jumpPoint);
    }

    /// <summary>
    /// Moves in one direction until the goal, a forced neighbor, or a
    /// diagonal route's horizontal/vertical jump point is found.
    /// </summary>
    private Tuple<int, int>? Jump(int x, int y, int dx, int dy, Tuple<int, int> goal)
    {
        // Look one cell farther along the requested direction.
        int nextX = x + dx;
        int nextY = y + dy;

        // A blocked move ends this ray; there is no jump point in this direction.
        if (!CanMove(x, y, dx, dy))
            return null;

        // Stop as soon as the ray reaches the goal.
        if (nextX == goal.Item1 && nextY == goal.Item2)
            return Tuple.Create(nextX, nextY);

        // A forced neighbor means a route may need to turn at this cell.
        if (HasForcedNeighbor(nextX, nextY, dx, dy))
            return Tuple.Create(nextX, nextY);

        // For diagonal travel, also probe each straight component. If either
        // straight ray finds a jump point, this diagonal cell is significant.
        if (dx != 0 && dy != 0)
        {
            // A diagonal cell is a jump point if either component direction
            // can independently reach a jump point.
            if (Jump(nextX, nextY, dx, 0, goal) != null ||
                Jump(nextX, nextY, 0, dy, goal) != null)
            {
                return Tuple.Create(nextX, nextY);
            }
        }

        return Jump(nextX, nextY, dx, dy, goal);
    }

    /// <summary>
    /// Checks for an obstacle that makes a previously pruned direction
    /// necessary. The checks are made at the cell just entered by Jump.
    /// </summary>
    private bool HasForcedNeighbor(int x, int y, int dx, int dy)
    {
        if (dx != 0 && dy != 0)
        {
            return (!IsValid(x - dx, y) && IsValid(x - dx, y + dy)) ||
                   (!IsValid(x, y - dy) && IsValid(x + dx, y - dy));
        }

        if (dx != 0)
        {
            return (!IsValid(x, y + 1) && IsValid(x + dx, y + 1)) ||
                   (!IsValid(x, y - 1) && IsValid(x + dx, y - 1));
        }

        return (!IsValid(x + 1, y) && IsValid(x + 1, y + dy)) ||
               (!IsValid(x - 1, y) && IsValid(x - 1, y + dy));
    }

    /// <summary>
    /// A diagonal step is legal if its destination is open and at least one
    /// adjacent orthogonal step is open, so the search cannot cut a blocked corner.
    /// </summary>
    private bool CanMove(int x, int y, int dx, int dy)
    {
        int nextX = x + dx;
        int nextY = y + dy;
        if (!IsValid(nextX, nextY))
            return false;

        return dx == 0 || dy == 0 ||
               IsValid(x + dx, y) || IsValid(x, y + dy);
    }

    /// <summary>
    /// Returns true when the coordinate is inside the grid and its cell is walkable.
    /// </summary>
    private bool IsValid(int x, int y) =>
        x >= 0 && x < _width && y >= 0 && y < _height && _grid[x, y];

    /// <summary>
    /// Octile distance is the exact obstacle-free distance for 8-way movement
    /// where a diagonal step costs sqrt(2).
    /// </summary>
    private float Heuristic(int x1, int y1, int x2, int y2)
    {
        int deltaX = Math.Abs(x1 - x2);
        int deltaY = Math.Abs(y1 - y2);
        int diagonalSteps = Math.Min(deltaX, deltaY);
        int straightSteps = Math.Max(deltaX, deltaY) - diagonalSteps;
        return (float)(diagonalSteps * Math.Sqrt(2) + straightSteps);
    }

    /// <summary>
    /// Calculates the cost between two jump points, charging 1 for straight
    /// steps and sqrt(2) for diagonal steps.
    /// </summary>
    private float Distance(int x1, int y1, int x2, int y2)
    {
        int deltaX = Math.Abs(x1 - x2);
        int deltaY = Math.Abs(y1 - y2);
        int diagonalSteps = Math.Min(deltaX, deltaY);
        int straightSteps = Math.Max(deltaX, deltaY) - diagonalSteps;
        return (float)(diagonalSteps * Math.Sqrt(2) + straightSteps);
    }

    /// <summary>
    /// Follows jump-point parents back to the start, then expands each straight
    /// or diagonal segment so callers receive every traversed grid cell.
    /// </summary>
    private List<Tuple<int, int>> ReconstructPath(
        Tuple<int, int> goal,
        Dictionary<Tuple<int, int>, Tuple<int, int>> parents)
    {
        // Seed the reverse path with the goal, then follow parent links until
        // there is no predecessor (the start node has none).
        var jumpPoints = new List<Tuple<int, int>> { goal };
        Tuple<int, int> current = goal;

#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type.
        while (parents.TryGetValue(current, out Tuple<int, int> parent))
        {
            jumpPoints.Add(parent);
            current = parent;
        }
#pragma warning restore CS8600 // Converting null literal or possible null value to non-nullable type.

        // Parent links run goal-to-start; reverse them for start-to-goal order.
        jumpPoints.Reverse();

        // Begin the final path with the start jump point.
        var path = new List<Tuple<int, int>> { jumpPoints[0] };

        // Expand each pair of jump points into the individual grid cells
        // traversed between them.
        for (int index = 1; index < jumpPoints.Count; index++)
        {
            Tuple<int, int> previous = jumpPoints[index - 1];
            Tuple<int, int> next = jumpPoints[index];

            // Jump points on one segment are aligned horizontally, vertically,
            // or diagonally, so their difference gives a unit step direction.
            int dx = Math.Sign(next.Item1 - previous.Item1);
            int dy = Math.Sign(next.Item2 - previous.Item2);
            int x = previous.Item1;
            int y = previous.Item2;

            // Advance by one cell at a time, including the next jump point.
            while (x != next.Item1 || y != next.Item2)
            {
                x += dx;
                y += dy;
                path.Add(Tuple.Create(x, y));
            }
        }

        return path;
    }
}
