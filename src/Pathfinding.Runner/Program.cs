using Pathfinding;

// The test harness: runs every algorithm on every map and prints the metrics.
// First a few tiny built-in maps, then every .map file in the maps folder (or a folder you pass in).

// Algorithms to test. When JPS and JPS+ exist, just uncomment them here.
var pathfinders = new List<IPathfinder>
{
    new AStar(),
    // new Jps(),
    // new JpsPlus(),
};

const int Runs = 10;        // each search gets timed this many times and averaged
const int PairsPerMap = 5;  // random start/goal pairs per map file

// ---- Collect the maps ----
var maps = new List<(string Name, GridMap Map, List<(Cell Start, Cell Goal)> Queries)>
{
    ("Open room", GridMap.FromStrings(
        "..........",
        "..........",
        "..........",
        "..........",
        ".........."),
        new() { (new Cell(0, 0), new Cell(9, 4)) }),

    ("Wall in the middle", GridMap.FromStrings(
        "..........",
        "....@.....",
        "....@.....",
        "....@.....",
        ".........."),
        new() { (new Cell(1, 2), new Cell(8, 2)) }),

    ("No path", GridMap.FromStrings(
        "...@...",
        "...@...",
        "...@..."),
        new() { (new Cell(0, 0), new Cell(6, 2)) }),
};

string folder = args.Length > 0 ? args[0] : "maps";
if (Directory.Exists(folder))
{
    foreach (string file in Directory.GetFiles(folder, "*.map").OrderBy(f => f))
    {
        var map = GridMap.Load(file);
        maps.Add((Path.GetFileName(file), map, RandomQueries(map, PairsPerMap)));
    }
}
else
{
    Console.WriteLine($"Heads up: no '{folder}' folder here, so only the built-in maps will run.\n");
}

// ---- Run everything ----
Console.WriteLine($"{"Map",-22} {"Algorithm",-9} {"Start",-9} {"Goal",-9} {"Found",-6} {"Cost",9} {"Expanded",9} {"Avg ms",9}");

foreach (var (name, map, queries) in maps)
{
    foreach (var finder in pathfinders)
    {
        // Preprocessing gets timed on its own (only matters for JPS+).
        var preTimer = System.Diagnostics.Stopwatch.StartNew();
        finder.Preprocess(map);
        double preMs = preTimer.Elapsed.TotalMilliseconds;
        if (preMs > 0.5) // A* and JPS do nothing here, so only bother printing when it took real time
            Console.WriteLine($"{name,-22} {finder.Name,-9} preprocessing took {preMs:F3} ms");

        foreach (var (start, goal) in queries)
        {
            finder.FindPath(map, start, goal); // warm-up, the first run is always slow while .NET gets going

            double totalMs = 0;
            SearchResult r = null;
            for (int i = 0; i < Runs; i++)
            {
                r = finder.FindPath(map, start, goal);
                totalMs += r.RunTimeMs;
            }

            string cost = r.Found ? r.PathCost.ToString("F3") : "-";
            Console.WriteLine($"{name,-22} {finder.Name,-9} {start,-9} {goal,-9} {r.Found,-6} {cost,9} {r.NodesExpanded,9} {totalMs / Runs,9:F4}");
        }
    }
}

// Random start/goal pairs on open cells. Fixed seed, so you get the same pairs every run
// (and every algorithm gets the exact same pairs).
static List<(Cell, Cell)> RandomQueries(GridMap map, int count)
{
    var openCells = new List<Cell>();
    for (int y = 0; y < map.Height; y++)
        for (int x = 0; x < map.Width; x++)
            if (map.IsOpen(x, y)) openCells.Add(new Cell(x, y));

    var queries = new List<(Cell, Cell)>();
    if (openCells.Count < 2) return queries;

    var random = new Random(42);
    for (int i = 0; i < count; i++)
        queries.Add((openCells[random.Next(openCells.Count)], openCells[random.Next(openCells.Count)]));
    return queries;
}
