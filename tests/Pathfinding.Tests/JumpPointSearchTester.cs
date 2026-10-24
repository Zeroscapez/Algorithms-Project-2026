using System;
using System.Collections.Generic;
using Pathfinding;

/// <summary>
/// Runs sanity checks for JumpPointSearch and reports a process exit code.
/// </summary>
public class JumpPointSearchTester
{
    /// <summary>
    /// Runs the JPS sanity scenarios and returns a process status code.
    /// </summary>
    /// <returns>Zero if every scenario passes; otherwise, one.</returns>
    public static int Main()
    {
        // Each helper returns one failure for its scenario, allowing the rest to keep running.
        int failures = 0;

        // Check ordinary open movement, obstacle-driven pruning, and the zero-length route.
        failures += Run("Open map", CreateOpenGrid(7, 5), new Cell(1, 1), new Cell(5, 3));
        failures += Run("Forced-neighbor wall pattern", CreateForcedNeighborMap(), new Cell(1, 1), new Cell(5, 3));
        failures += Run("Start equals goal", CreateOpenGrid(7, 5), new Cell(2, 2), new Cell(2, 2));

        // Invalid endpoints and a complete wall must result in no path.
        failures += RunExpectedNoPath("Blocked start", CreateMapWithBlockedCell(new Cell(1, 1)), new Cell(1, 1), new Cell(5, 3));
        failures += RunExpectedNoPath("Blocked goal", CreateMapWithBlockedCell(new Cell(5, 3)), new Cell(1, 1), new Cell(5, 3));
        failures += RunExpectedNoPath("Out-of-bounds start", CreateOpenGrid(7, 5), new Cell(-1, 1), new Cell(5, 3));
        failures += RunExpectedNoPath("Unreachable goal", CreateSeparatedMap(), new Cell(1, 2), new Cell(3, 2));

        // Degenerate grids, blocked diagonal corners, empty maps, and null input cover edge cases.
        failures += Run("Single-cell grid", CreateOpenGrid(1, 1), new Cell(0, 0), new Cell(0, 0));
        failures += Run("Single-row corridor", CreateOpenGrid(7, 1), new Cell(0, 0), new Cell(6, 0));
        failures += Run("Single-column corridor", CreateOpenGrid(1, 7), new Cell(0, 0), new Cell(0, 6));
        failures += RunExpectedNoPath("Blocked diagonal corner", CreateDiagonalCornerTrap(), new Cell(0, 0), new Cell(1, 1));
        failures += RunExpectedNoPath("Empty grid", new GridMap(0, 3, Array.Empty<bool>()), new Cell(0, 0), new Cell(0, 0));
        failures += RunExpectedNullMap();

        // Return success only when every scenario reported no failures.
        if (failures == 0)
        {
            Console.WriteLine("All JumpPointSearch tests passed.");
            return 0;
        }

        Console.WriteLine($"{failures} JumpPointSearch test(s) failed.");
        return 1;
    }

    /// <summary>
    /// Runs a scenario that must find a path and validates every returned step.
    /// </summary>
    /// <param name="name">Human-readable scenario name printed in the test report.</param>
    /// <param name="map">The map to search.</param>
    /// <param name="start">The path's expected starting cell.</param>
    /// <param name="goal">The path's expected destination cell.</param>
    /// <returns>Zero when the scenario passes; otherwise, one.</returns>
    private static int Run(string name, GridMap map, Cell start, Cell goal)
    {
        try
        {
            // Set up the algorithm and execute it against the requested map and endpoints.
            var search = new JumpPointSearch();
            search.Preprocess(map);
            SearchResult result = search.FindPath(map, start, goal);

            // A successful search must also return a legal full-cell path.
            if (!result.Found)
                throw new Exception("Expected a path, but none was found.");

            AssertValidPath(result.Path, map, start, goal);
            Console.WriteLine($"PASS: {name} ({result.Path.Count} cells)");
            return 0;
        }
        catch (Exception exception)
        {
            // Treat thrown errors as a failed scenario while preserving the test run.
            Console.WriteLine($"FAIL: {name} - {exception.Message}");
            return 1;
        }
    }

    /// <summary>
    /// Runs a scenario where the endpoints are expected to have no connecting path.
    /// </summary>
    /// <param name="name">Human-readable scenario name printed in the test report.</param>
    /// <param name="map">The map to search.</param>
    /// <param name="start">The starting cell.</param>
    /// <param name="goal">The destination cell.</param>
    /// <returns>Zero when no path is found as expected; otherwise, one.</returns>
    private static int RunExpectedNoPath(string name, GridMap map, Cell start, Cell goal)
    {
        try
        {
            // Run the search without preprocessing requirements for this scenario.
            var search = new JumpPointSearch();
            SearchResult result = search.FindPath(map, start, goal);

            // A found route contradicts the expected disconnected or invalid case.
            if (result.Found)
                throw new Exception($"Expected no path, but received {result.Path.Count} cells.");

            Console.WriteLine($"PASS: {name} (no path)");
            return 0;
        }
        catch (Exception exception)
        {
            // Surface unexpected exceptions as failures rather than passing as "no path."
            Console.WriteLine($"FAIL: {name} - {exception.Message}");
            return 1;
        }
    }

    /// <summary>
    /// Confirms that searching with a null map throws an argument exception.
    /// </summary>
    /// <returns>Zero when the expected exception is thrown; otherwise, one.</returns>
    private static int RunExpectedNullMap()
    {
        try
        {
            // Deliberately pass null to verify the public method's argument guard.
            new JumpPointSearch().FindPath(null!, new Cell(0, 0), new Cell(1, 1));
            throw new Exception("Expected an ArgumentNullException.");
        }
        catch (ArgumentNullException)
        {
            // This is the expected result when the map argument is null.
            Console.WriteLine("PASS: Null map (ArgumentNullException)");
            return 0;
        }
        catch (Exception exception)
        {
            // Any other exception indicates the null-input contract was not met.
            Console.WriteLine($"FAIL: Null map - {exception.Message}");
            return 1;
        }
    }

    /// <summary>
    /// Creates a fully walkable map with the requested dimensions.
    /// </summary>
    /// <param name="width">Number of columns in the map.</param>
    /// <param name="height">Number of rows in the map.</param>
    /// <returns>A <see cref="GridMap"/> in which every cell is open.</returns>
    private static GridMap CreateOpenGrid(int width, int height)
    {
        // GridMap stores cells in a flat row-major array, so mark each slot walkable.
        var open = new bool[width * height];
        Array.Fill(open, true);
        return new GridMap(width, height, open);
    }

    /// <summary>
    /// Creates a 2-by-2 map whose open diagonal endpoints cannot connect without crossing a corner.
    /// </summary>
    /// <returns>A map with only cells (0, 0) and (1, 1) open.</returns>
    private static GridMap CreateDiagonalCornerTrap()
    {
        // Only the diagonal endpoints are open; the two orthogonal corner cells are walls.
        var open = new bool[4];
        open[0] = true;
        open[3] = true;
        return new GridMap(2, 2, open);
    }

    /// <summary>
    /// Creates the standard 7-by-5 open map with one cell blocked.
    /// </summary>
    /// <param name="blockedCell">The coordinate to mark as blocked.</param>
    /// <returns>A map open everywhere except at <paramref name="blockedCell"/>.</returns>
    private static GridMap CreateMapWithBlockedCell(Cell blockedCell)
    {
        // Start with an open map, then convert the requested (x, y) coordinate to its flat index.
        var open = new bool[7 * 5];
        Array.Fill(open, true);
        open[blockedCell.Y * 7 + blockedCell.X] = false;
        return new GridMap(7, 5, open);
    }

    /// <summary>
    /// Creates an open 7-by-5 map split by a full-height wall.
    /// </summary>
    /// <returns>A map whose blocked column prevents travel between its sides.</returns>
    private static GridMap CreateSeparatedMap()
    {
        // Begin with open terrain and block column 2 from top to bottom.
        var open = new bool[7 * 5];
        Array.Fill(open, true);

        // The solid column divides the map into disconnected left and right regions.
        for (int y = 0; y < 5; y++)
            open[y * 7 + 2] = false;
        return new GridMap(7, 5, open);
    }

    /// <summary>
    /// Creates an open 7-by-5 map with obstacles that exercise forced-neighbor handling.
    /// </summary>
    /// <returns>The obstacle map used by the forced-neighbor scenario.</returns>
    private static GridMap CreateForcedNeighborMap()
    {
        // Begin with open terrain, then place walls that create forced turns along candidate rays.
        var open = new bool[7 * 5];
        Array.Fill(open, true);
        // The walls at (1,2), (3,0), and (3,1) exercise the forced-neighbor cases.
        open[2 * 7 + 1] = false;
        open[0 * 7 + 3] = false;
        open[1 * 7 + 3] = false;
        return new GridMap(7, 5, open);
    }

    /// <summary>
    /// Verifies that a path includes its endpoints and uses only legal adjacent steps.
    /// </summary>
    /// <param name="path">The path returned by JPS.</param>
    /// <param name="map">The map against which each step is checked.</param>
    /// <param name="start">The expected first cell.</param>
    /// <param name="goal">The expected last cell.</param>
    /// <exception cref="Exception">A path endpoint or movement step is invalid.</exception>
    private static void AssertValidPath(List<Cell> path, GridMap map, Cell start, Cell goal)
    {
        // Confirm the result is non-empty and begins and ends at the requested endpoints.
        if (path.Count == 0 || path[0] != start || path[^1] != goal)
            throw new Exception("The path is empty or does not include both endpoints.");

        // Check each adjacent pair independently using GridMap's movement rules.
        for (int index = 1; index < path.Count; index++)
        {
            Cell previous = path[index - 1];
            Cell current = path[index];
            int dx = current.X - previous.X;
            int dy = current.Y - previous.Y;
            int dir = -1;

            // Convert the coordinate difference into GridMap's direction index.
            for (int candidate = 0; candidate < 8; candidate++)
                if (GridMap.DirX[candidate] == dx && GridMap.DirY[candidate] == dy)
                {
                    dir = candidate;
                    break;
                }

            // Reject non-adjacent steps and moves blocked by obstacles or corner constraints.
            if (dir < 0 || !map.CanStep(previous.X, previous.Y, dir))
                throw new Exception($"Invalid step from {previous} to {current}.");
        }
    }
}
