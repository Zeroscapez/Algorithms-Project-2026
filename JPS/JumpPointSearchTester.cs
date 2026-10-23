using System;
using System.Collections.Generic;

/// <summary>
/// Runs sanity checks for JumpPointSearch and reports a process exit code.
/// </summary>
public class JumpPointSearchTester
{
    /// <summary>
    /// Runs all test scenarios and returns zero only when every scenario passes.
    /// </summary>
    public static int Main()
    {
        // Count failures so the program can report every problem after both tests run.
        int failures = 0;

        // Test 1: Every grid cell is walkable, so the search has no walls to navigate around.
        failures += Run("Open map", CreateOpenMap(), (1, 1), (5, 3));

        // Test 2: Walls create forced-neighbor paths that require diagonal/original JPS logic.
        failures += Run("Map with forced-neighbor wall pattern", CreateForcedNeighborMap(), (1, 1), (5, 3));

        // Test 3: Starting and ending at the same walkable cell returns that cell.
        failures += Run("Start equals goal", CreateOpenMap(), (2, 2), (2, 2));

        // Test 4-7: Invalid endpoints and a wall barrier must not produce a path.
        failures += RunExpectedNoPath("Blocked start", CreateMapWithBlockedCell((1, 1)), (1, 1), (5, 3));
        failures += RunExpectedNoPath("Blocked goal", CreateMapWithBlockedCell((5, 3)), (1, 1), (5, 3));
        failures += RunExpectedNoPath("Out-of-bounds start", CreateOpenMap(), (-1, 1), (5, 3));
        failures += RunExpectedNoPath("Unreachable goal", CreateSeparatedMap(), (1, 2), (3, 2));

        // Null arguments are programmer errors and should be reported explicitly.
        failures += RunExpectedArgumentNull("Null start", null, Tuple.Create(1, 1));
        failures += RunExpectedArgumentNull("Null goal", Tuple.Create(1, 1), null);

        // Tiny and one-dimensional maps exercise boundaries with almost no alternate routes.
        failures += Run("Single-cell grid", CreateOpenGrid(1, 1), (0, 0), (0, 0));
        failures += Run("Single-row corridor", CreateOpenGrid(1, 7), (0, 0), (0, 6));
        failures += Run("Single-column corridor", CreateOpenGrid(7, 1), (0, 0), (6, 0));

        // Diagonal endpoints cannot connect when both orthogonal corner cells are blocked.
        failures += RunExpectedNoPath("Blocked diagonal corner", CreateDiagonalCornerTrap(), (0, 0), (1, 1));

        // A zero-sized map has no valid coordinates, even when the endpoints match.
        failures += RunExpectedNoPath("Empty grid", new bool[0, 3], (0, 0), (0, 0));

        // Report success only when none of the test helpers recorded a failure.
        if (failures == 0)
        {
            Console.WriteLine("All JumpPointSearch tests passed.");
            return 0;
        }

        Console.WriteLine($"{failures} JumpPointSearch test(s) failed.");
        return 1;
    }

    /// <summary>
    /// Runs a case that should produce a valid path between the supplied endpoints.
    /// </summary>
    private static int Run(string name, bool[,] grid, (int X, int Y) start, (int X, int Y) goal)
    {
        // Run each scenario independently, catching failures without stopping later tests.
        try
        {
            // Create the searcher with the test grid and ask it to find the path.
            var search = new JumpPointSearch(grid);
#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type.
            List<Tuple<int, int>> path = search.FindPath(
                Tuple.Create(start.X, start.Y),
                Tuple.Create(goal.X, goal.Y));
#pragma warning restore CS8600 // Converting null literal or possible null value to non-nullable type.

            if (path == null)
            {
                throw new Exception("Expected a path, but none was found.");
            }

            // Validate the result instead of only checking whether a list was returned.
            AssertValidPath(name, path, grid, start, goal);
            Console.WriteLine($"PASS: {name} ({path.Count} points)");
            return 0;
        }
        // Convert an exception into a reported test failure so remaining cases run.
        catch (Exception exception)
        {
            Console.WriteLine($"FAIL: {name} - {exception.Message}");
            return 1;
        }
    }

    /// <summary>
    /// Runs a case whose endpoints should not be connected by a valid path.
    /// </summary>
    private static int RunExpectedNoPath(
        string name,
        bool[,] grid,
        (int X, int Y) start,
        (int X, int Y) goal)
    {
        // A successful search result is a failure for this negative test.
        try
        {
            var search = new JumpPointSearch(grid);
            List<Tuple<int, int>>? path = search.FindPath(
                Tuple.Create(start.X, start.Y),
                Tuple.Create(goal.X, goal.Y));

            if (path != null)
                throw new Exception($"Expected no path, but received {path.Count} points.");

            Console.WriteLine($"PASS: {name} (no path)");
            return 0;
        }
        // Report unexpected exceptions as failures instead of treating them as no path.
        catch (Exception exception)
        {
            Console.WriteLine($"FAIL: {name} - {exception.Message}");
            return 1;
        }
    }

    /// <summary>
    /// Verifies that FindPath throws when either coordinate argument is null.
    /// </summary>
    private static int RunExpectedArgumentNull(
        string name,
        Tuple<int, int>? start,
        Tuple<int, int>? goal)
    {
        // If FindPath returns normally, the expected argument validation is missing.
        try
        {
            var search = new JumpPointSearch(CreateOpenMap());
            search.FindPath(start!, goal!);
            throw new Exception("Expected an ArgumentNullException.");
        }
        // This is the expected result for a null start or goal argument.
        catch (ArgumentNullException)
        {
            Console.WriteLine($"PASS: {name} (ArgumentNullException)");
            return 0;
        }
        // Any other exception indicates a failure in the null-argument check.
        catch (Exception exception)
        {
            Console.WriteLine($"FAIL: {name} - {exception.Message}");
            return 1;
        }
    }

    /// <summary>
    /// Creates a fully walkable 7-by-5 grid for tests without obstacles.
    /// </summary>
    private static bool[,] CreateOpenMap()
    {
        // The grid is 7 cells wide and 5 cells high.
        return CreateOpenGrid(7, 5);
    }

    /// <summary>
    /// Creates a fully walkable grid with the requested width and height.
    /// </summary>
    private static bool[,] CreateOpenGrid(int width, int height)
    {
        // A new bool array defaults to false, so mark every cell walkable.
        bool[,] grid = new bool[width, height];
        for (int x = 0; x < grid.GetLength(0); x++)
        {
            // Visit each row position for the current column.
            for (int y = 0; y < grid.GetLength(1); y++)
            {
                // true means the cell can be walked through.
                grid[x, y] = true;
            }
        }

        return grid;
    }

    /// <summary>
    /// Creates a two-cell diagonal setup where both corner-cutting routes are blocked.
    /// </summary>
    private static bool[,] CreateDiagonalCornerTrap()
    {
        // Only the diagonal endpoints are open; the two side cells remain blocked.
        bool[,] grid = new bool[2, 2];
        grid[0, 0] = true;
        grid[1, 1] = true;
        return grid;
    }

    /// <summary>
    /// Creates an open grid with one specified cell marked as blocked.
    /// </summary>
    private static bool[,] CreateMapWithBlockedCell((int X, int Y) blockedCell)
    {
        // Reuse the open-grid setup, then block the requested coordinate.
        bool[,] grid = CreateOpenMap();
        grid[blockedCell.X, blockedCell.Y] = false;
        return grid;
    }

    /// <summary>
    /// Creates an open grid split into two unreachable regions by a wall.
    /// </summary>
    private static bool[,] CreateSeparatedMap()
    {
        // A full-height wall separates the left and right sides of the grid.
        bool[,] grid = CreateOpenMap();
        // Block the same column in every row so no route can cross the barrier.
        for (int y = 0; y < grid.GetLength(1); y++)
            grid[2, y] = false;

        return grid;
    }

    /// <summary>
    /// Creates an open grid with obstacles that exercise forced-neighbor handling.
    /// </summary>
    private static bool[,] CreateForcedNeighborMap()
    {
        // Start with an open grid, then add walls that block direct movement.
        bool[,] grid = CreateOpenMap();

        // A wall beside an otherwise straight path can force a move into a
        // neighboring row. These false cells represent walls in the grid.
        grid[1, 2] = false;
        grid[3, 0] = false;
        grid[3, 1] = false;

        return grid;
    }

    /// <summary>
    /// Throws when a path is empty, misses an endpoint, crosses a wall,
    /// leaves the grid, skips cells, or cuts a blocked diagonal corner.
    /// </summary>
    private static void AssertValidPath(
        string name,
        List<Tuple<int, int>> path,
        bool[,] grid,
        (int X, int Y) start,
        (int X, int Y) goal)
    {
        // A valid path must contain at least one position.
        if (path.Count == 0)
        {
            throw new Exception("The returned path is empty.");
        }

        // Check that the path starts at the caller's requested start coordinate.
        if (path[0].Item1 != start.X || path[0].Item2 != start.Y)
        {
            throw new Exception("The path does not begin at the start position.");
        }

        // Check that the final position is the requested goal.
        Tuple<int, int> last = path[path.Count - 1];
        if (last.Item1 != goal.X || last.Item2 != goal.Y)
        {
            throw new Exception("The path does not end at the goal position.");
        }

        // Inspect every returned point and the connection to the previous point.
        for (int index = 0; index < path.Count; index++)
        {
            // Validate one returned coordinate at a time.
            Tuple<int, int> point = path[index];

            // Reject coordinates outside the grid dimensions.
            if (point.Item1 < 0 || point.Item1 >= grid.GetLength(0) ||
                point.Item2 < 0 || point.Item2 >= grid.GetLength(1))
            {
                throw new Exception($"The path leaves the map at {point}.");
            }

            // Reject wall cells in the returned path; false means blocked.
            if (!grid[point.Item1, point.Item2])
            {
                throw new Exception($"The path crosses a wall at {point}.");
            }

            // Consecutive points must be orthogonally or diagonally adjacent.
            if (index > 0)
            {
                // Compare each step to its predecessor to ensure movement is adjacent.
                Tuple<int, int> previous = path[index - 1];
                int deltaX = Math.Abs(point.Item1 - previous.Item1);
                int deltaY = Math.Abs(point.Item2 - previous.Item2);

                // This also rejects duplicate points and jumps that skipped a map cell.
                if (deltaX > 1 || deltaY > 1 || (deltaX == 0 && deltaY == 0))
                {
                    throw new Exception(
                        $"The path has a discontinuity between {previous} and {point}.");
                }

                // Diagonal movement is allowed only if at least one adjacent
                // orthogonal cell is open; reject corner-cutting paths.
                if (deltaX == 1 && deltaY == 1 &&
                    !grid[point.Item1, previous.Item2] &&
                    !grid[previous.Item1, point.Item2])
                {
                    throw new Exception(
                        $"The path cuts a blocked corner between {previous} and {point}.");
                }
            }
        }
    }
}
