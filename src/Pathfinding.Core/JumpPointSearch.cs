using System;
using System.Collections.Generic;
using System.Diagnostics;
using Pathfinding;

/// <summary>
/// Finds shortest paths on eight-directional grids using Jump Point Search.
/// </summary>
public class JumpPointSearch : IPathfinder
{
    /// <summary>
    /// Gets the short name used to identify this pathfinding algorithm.
    /// </summary>
    public string Name => "JPS";

    /// <summary>
    /// Performs any map-specific setup required before searches begin.
    /// JPS does not need preprocessing, but validates that the map is supplied.
    /// </summary>
    /// <param name="map">The map that will be searched.</param>
    public void Preprocess(GridMap map)
    {
        // JPS has no map-derived tables to build, but validates the supplied map consistently.
        if (map == null)
            throw new ArgumentNullException(nameof(map));
    }

    /// <summary>
    /// Stores a jump point's location, cost from the start, and estimated total cost.
    /// </summary>
    private readonly struct Node
    {
        // Cell is the jump point; G is its known cost and F is its estimated total cost.
        public readonly Cell Cell;
        public readonly double G;
        public readonly double F;

        /// <summary>
        /// Creates a search node and calculates its estimated total cost.
        /// </summary>
        /// <param name="cell">The jump point represented by this node.</param>
        /// <param name="g">The known cost from the start to this cell.</param>
        /// <param name="h">The estimated remaining cost to the goal.</param>
        public Node(Cell cell, double g, double h)
        {
            // Store the known route cost and combine it with the heuristic for queue ordering.
            Cell = cell;
            G = g;
            F = g + h;
        }
    }

    /// <summary>
    /// Searches for a shortest path and returns every traversed cell.
    /// </summary>
    /// <param name="map">The grid to search.</param>
    /// <param name="start">The walkable starting cell.</param>
    /// <param name="goal">The walkable destination cell.</param>
    /// <param name="recorder">Optional recipient for open and expanded-cell events.</param>
    /// <returns>The search outcome, including the path and its cost when found.</returns>
    public SearchResult FindPath(
        GridMap map,
        Cell start,
        Cell goal,
        SearchRecorder recorder = null)
    {
        // A missing map is a programming error; invalid endpoints produce an ordinary no-path result.
        if (map == null)
            throw new ArgumentNullException(nameof(map));

        // Measure the whole call, including endpoint validation and path reconstruction.
        var timer = Stopwatch.StartNew();
        var result = new SearchResult();

        // The result defaults to not found, with no path and infinite cost.
        if (!map.IsOpen(start) || !map.IsOpen(goal))
        {
            result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
            return result;
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

        // Track the frontier, finalized points, best costs, and each point's predecessor.
        var open = new List<Node>();
        var closed = new HashSet<Cell>();
        var gScores = new Dictionary<Cell, double> { [start] = 0 };
        var parents = new Dictionary<Cell, Cell>();

        // Seed the search at the start and record its opening if an observer was supplied.
        open.Add(new Node(start, 0, Heuristic(start, goal)));
        recorder?.Opened(start);

        while (open.Count > 0)
        {
            // Select the frontier point with the smallest estimated total route cost.
            open.Sort((left, right) => left.F.CompareTo(right.F));
            Node current = open[0];
            open.RemoveAt(0);

            // Ignore old queue entries if a cheaper route to the same point was found later.
            if (!gScores.TryGetValue(current.Cell, out double bestG) || current.G > bestG)
                continue;

            // Mark this point finalized and record that it was expanded.
            closed.Add(current.Cell);
            result.NodesExpanded++;
            recorder?.Expanded(current.Cell);

            // Reconstruct the route once the destination is reached.
            if (current.Cell == goal)
            {
                result.Found = true;
                result.PathCost = current.G;
                result.Path = ReconstructPath(goal, parents);
                break;
            }

            // JPS expands only significant jump points, not every cell along each ray.
            foreach (Cell successor in GetSuccessors(map, current.Cell, goal, parents))
            {
                // A closed point was already fully processed.
                if (closed.Contains(successor))
                    continue;

                // Ignore routes that do not improve the best known cost.
                double tentativeG = current.G + Distance(current.Cell, successor);
                if (gScores.TryGetValue(successor, out double existingG) &&
                    tentativeG >= existingG)
                {
                    continue;
                }

                // Save the improved route and schedule the jump point for later expansion.
                gScores[successor] = tentativeG;
                parents[successor] = current.Cell;
                open.Add(new Node(successor, tentativeG, Heuristic(successor, goal)));
                recorder?.Opened(successor);
            }
        }

        // Save elapsed time for both successful searches and exhausted frontiers.
        result.RunTimeMs = timer.Elapsed.TotalMilliseconds;
        return result;
    }

    /// <summary>
    /// Selects the jump points reachable from the current point using JPS direction pruning.
    /// </summary>
    /// <param name="map">The grid that determines legal moves.</param>
    /// <param name="current">The jump point being expanded.</param>
    /// <param name="goal">The destination that stops a jump ray.</param>
    /// <param name="parents">Known predecessor links used to determine the incoming direction.</param>
    /// <returns>Unique jump points reachable in the directions retained by pruning.</returns>
    private static List<Cell> GetSuccessors(
        GridMap map,
        Cell current,
        Cell goal,
        Dictionary<Cell, Cell> parents)
    {
        // A parentless point is the start, so there is no incoming direction to prune against.
        var successors = new List<Cell>();

        if (!parents.TryGetValue(current, out Cell parent))
        {
            // Try all map directions at the start; the jump helper filters blocked moves.
            for (int dir = 0; dir < 8; dir++)
                AddJumpSuccessor(map, successors, current, GridMap.DirX[dir], GridMap.DirY[dir], goal);
            return successors;
        }

        // Reduce the previous segment to its unit incoming direction.
        int dx = Math.Sign(current.X - parent.X);
        int dy = Math.Sign(current.Y - parent.Y);

        if (dx != 0 && dy != 0)
        {
            // Diagonal arrivals naturally continue diagonally or along either straight component.
            AddJumpSuccessor(map, successors, current, dx, dy, goal);
            AddJumpSuccessor(map, successors, current, dx, 0, goal);
            AddJumpSuccessor(map, successors, current, 0, dy, goal);

            // A blocked trailing side with an open escape makes a diagonal turn necessary.
            if (!map.IsOpen(current.X - dx, current.Y) &&
                map.IsOpen(current.X - dx, current.Y + dy))
            {
                AddJumpSuccessor(map, successors, current, -dx, dy, goal);
            }

            // Check the equivalent forced turn on the other side of the diagonal.
            if (!map.IsOpen(current.X, current.Y - dy) &&
                map.IsOpen(current.X + dx, current.Y - dy))
            {
                AddJumpSuccessor(map, successors, current, dx, -dy, goal);
            }
        }
        else if (dx != 0)
        {
            // Horizontal travel continues forward and can be forced diagonally around either side.
            AddJumpSuccessor(map, successors, current, dx, 0, goal);

            // Under the map's strict corner rule, a blocked rear diagonal can force
            // an orthogonal turn even though the cell immediately beside the ray is open.
            if (!map.IsOpen(current.X - dx, current.Y + 1) &&
                map.IsOpen(current.X, current.Y + 1))
            {
                AddJumpSuccessor(map, successors, current, 0, 1, goal);
            }

            if (!map.IsOpen(current.X - dx, current.Y - 1) &&
                map.IsOpen(current.X, current.Y - 1))
            {
                AddJumpSuccessor(map, successors, current, 0, -1, goal);
            }

            // An obstacle above can force an upward diagonal.
            if (!map.IsOpen(current.X, current.Y + 1) &&
                map.IsOpen(current.X + dx, current.Y + 1))
            {
                AddJumpSuccessor(map, successors, current, dx, 1, goal);
            }

            // An obstacle below can force a downward diagonal.
            if (!map.IsOpen(current.X, current.Y - 1) &&
                map.IsOpen(current.X + dx, current.Y - 1))
            {
                AddJumpSuccessor(map, successors, current, dx, -1, goal);
            }
        }
        else
        {
            // Vertical travel continues forward and can be forced diagonally around either side.
            AddJumpSuccessor(map, successors, current, 0, dy, goal);

            // A blocked rear diagonal can likewise force a legal orthogonal turn.
            if (!map.IsOpen(current.X + 1, current.Y - dy) &&
                map.IsOpen(current.X + 1, current.Y))
            {
                AddJumpSuccessor(map, successors, current, 1, 0, goal);
                AddJumpSuccessor(map, successors, current, 1, dy, goal);
            }

            // Apply the same forced-turn check on the left side of the vertical ray.
            if (!map.IsOpen(current.X - 1, current.Y - dy) &&
                map.IsOpen(current.X - 1, current.Y))
            {
                AddJumpSuccessor(map, successors, current, -1, 0, goal);
                AddJumpSuccessor(map, successors, current, -1, dy, goal);
            }

            // A right-side obstacle can force a rightward diagonal.
            if (!map.IsOpen(current.X + 1, current.Y) &&
                map.IsOpen(current.X + 1, current.Y + dy))
            {
                AddJumpSuccessor(map, successors, current, 1, dy, goal);
            }

            // A left-side obstacle can force a leftward diagonal.
            if (!map.IsOpen(current.X - 1, current.Y) &&
                map.IsOpen(current.X - 1, current.Y + dy))
            {
                AddJumpSuccessor(map, successors, current, -1, dy, goal);
            }
        }

        // Return the unique significant points discovered in the retained directions.
        return successors;
    }

    /// <summary>
    /// Follows one candidate direction and adds its next jump point if one exists.
    /// </summary>
    /// <param name="map">The grid that determines whether each step is legal.</param>
    /// <param name="successors">Collection to receive a newly discovered jump point.</param>
    /// <param name="current">The cell from which this ray starts.</param>
    /// <param name="dx">Horizontal component of the unit direction.</param>
    /// <param name="dy">Vertical component of the unit direction.</param>
    /// <param name="goal">The destination cell.</param>
    private static void AddJumpSuccessor(
        GridMap map,
        List<Cell> successors,
        Cell current,
        int dx,
        int dy,
        Cell goal)
    {
        // Skip rays whose first move is blocked or violates GridMap's diagonal rules.
        if (!map.CanStep(current.X, current.Y, Direction(dx, dy)))
            return;

        // Follow the ray to its next jump point and add it only once.
        Cell? jumpPoint = Jump(map, current, dx, dy, goal);
        if (jumpPoint.HasValue && !successors.Contains(jumpPoint.Value))
            successors.Add(jumpPoint.Value);
    }

    /// <summary>
    /// Advances along a ray until it reaches the goal, a forced neighbor, or
    /// a diagonal point whose straight component contains a jump point.
    /// </summary>
    /// <param name="map">The grid that determines whether each step is legal.</param>
    /// <param name="current">The current cell along the ray.</param>
    /// <param name="dx">Horizontal component of the unit direction.</param>
    /// <param name="dy">Vertical component of the unit direction.</param>
    /// <param name="goal">The destination cell.</param>
    /// <returns>The next jump point, or <see langword="null"/> if the ray is blocked.</returns>
    private static Cell? Jump(GridMap map, Cell current, int dx, int dy, Cell goal)
    {
        // Stop this ray if its next step cannot be taken.
        int direction = Direction(dx, dy);
        if (!map.CanStep(current.X, current.Y, direction))
            return null;

        // The goal and forced-neighbor cells are significant stopping points.
        var next = new Cell(current.X + dx, current.Y + dy);
        if (next == goal || HasForcedNeighbor(map, next, dx, dy))
            return next;

        // A diagonal cell is significant when either straight component reaches a jump point.
        if (dx != 0 && dy != 0 &&
            (Jump(map, next, dx, 0, goal).HasValue ||
             Jump(map, next, 0, dy, goal).HasValue))
        {
            return next;
        }

        // Continue recursively when no stopping condition has been reached.
        return Jump(map, next, dx, dy, goal);
    }

    /// <summary>
    /// Checks whether an obstacle next to a cell creates a necessary, otherwise-pruned move.
    /// </summary>
    /// <param name="map">The grid containing the cell and its neighboring positions.</param>
    /// <param name="cell">The cell just entered by a jump ray.</param>
    /// <param name="dx">Horizontal component of the ray's direction.</param>
    /// <param name="dy">Vertical component of the ray's direction.</param>
    /// <returns><see langword="true"/> if a forced neighbor exists.</returns>
    private static bool HasForcedNeighbor(GridMap map, Cell cell, int dx, int dy)
    {
        if (dx != 0 && dy != 0)
        {
            // On a diagonal, a blocked trailing side may make an alternate diagonal necessary.
            return (!map.IsOpen(cell.X - dx, cell.Y) &&
                    map.IsOpen(cell.X - dx, cell.Y + dy)) ||
                   // Check the other trailing side for the matching forced escape.
                   (!map.IsOpen(cell.X, cell.Y - dy) &&
                    map.IsOpen(cell.X + dx, cell.Y - dy));
        }

        if (dx != 0)
        {
            // On a horizontal ray, inspect obstacles above and below the current cell.
            return (!map.IsOpen(cell.X - dx, cell.Y + 1) &&
                    map.IsOpen(cell.X, cell.Y + 1)) ||
                   // Repeat for the lower side, then account for forward diagonal alternatives.
                   (!map.IsOpen(cell.X - dx, cell.Y - 1) &&
                    map.IsOpen(cell.X, cell.Y - 1)) ||
                   (!map.IsOpen(cell.X, cell.Y + 1) &&
                    map.IsOpen(cell.X + dx, cell.Y + 1)) ||
                   (!map.IsOpen(cell.X, cell.Y - 1) &&
                    map.IsOpen(cell.X + dx, cell.Y - 1));
        }

        // On a vertical ray, perform the corresponding checks to the left and right.
        return (!map.IsOpen(cell.X + 1, cell.Y - dy) &&
                map.IsOpen(cell.X + 1, cell.Y)) ||
               (!map.IsOpen(cell.X - 1, cell.Y - dy) &&
                map.IsOpen(cell.X - 1, cell.Y)) ||
               (!map.IsOpen(cell.X + 1, cell.Y) &&
                map.IsOpen(cell.X + 1, cell.Y + dy)) ||
               (!map.IsOpen(cell.X - 1, cell.Y) &&
                map.IsOpen(cell.X - 1, cell.Y + dy));
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
    /// Estimates the remaining path cost using octile distance for eight-way movement.
    /// </summary>
    /// <param name="from">The cell where the estimate begins.</param>
    /// <param name="to">The target cell.</param>
    /// <returns>The obstacle-free cost estimate, with diagonals charged at sqrt(2).</returns>
    private static double Heuristic(Cell from, Cell to)
    {
        // Pair axis differences into diagonal steps first, then count leftover straight steps.
        int dx = Math.Abs(from.X - to.X);
        int dy = Math.Abs(from.Y - to.Y);
        int diagonalSteps = Math.Min(dx, dy);
        int straightSteps = Math.Max(dx, dy) - diagonalSteps;
        // Octile distance ignores obstacles and therefore does not overestimate the route.
        return diagonalSteps * GridMap.Sqrt2 + straightSteps;
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
    /// Follows jump-point predecessors and expands each segment into individual grid cells.
    /// </summary>
    /// <param name="goal">The final jump point in the route.</param>
    /// <param name="parents">Predecessor links created during the search.</param>
    /// <returns>The complete path ordered from start to goal, including both endpoints.</returns>
    private static List<Cell> ReconstructPath(Cell goal, Dictionary<Cell, Cell> parents)
    {
        // Follow parent links backward from the goal to collect the search's jump points.
        var jumpPoints = new List<Cell> { goal };
        Cell current = goal;
        while (parents.TryGetValue(current, out Cell parent))
        {
            jumpPoints.Add(parent);
            current = parent;
        }

        // Reverse the points into travel order and initialize the expanded route at the start.
        jumpPoints.Reverse();
        var path = new List<Cell> { jumpPoints[0] };

        // Expand each pair of jump points into the individual cells traversed between them.
        for (int index = 1; index < jumpPoints.Count; index++)
        {
            Cell previous = jumpPoints[index - 1];
            Cell next = jumpPoints[index];
            int dx = Math.Sign(next.X - previous.X);
            int dy = Math.Sign(next.Y - previous.Y);
            int x = previous.X;
            int y = previous.Y;

            // Walk one cell at a time so the returned path contains no skipped cells.
            while (x != next.X || y != next.Y)
            {
                x += dx;
                y += dy;
                path.Add(new Cell(x, y));
            }
        }

        return path;
    }
}
