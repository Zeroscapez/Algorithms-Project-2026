using Pathfinding;

//run A* on a few maps and print the metrics.
// Add your own maps to this list.
var tests = new List<(string Name, GridMap Map, Cell Start, Cell Goal)>
{
    ("Open room", GridMap.FromStrings(
        "..........",
        "..........",
        "..........",
        "..........",
        ".........."),
        new Cell(0, 0), new Cell(9, 4)),

    ("Wall in the middle", GridMap.FromStrings(
        "..........",
        "....@.....",
        "....@.....",
        "....@.....",
        ".........."),
        new Cell(1, 2), new Cell(8, 2)),

    ("Small maze", GridMap.FromStrings(
        ".@........",
        ".@.@@@@@@.",
        ".@.@......",
        ".@.@.@@@@@",
        "...@......"),
        new Cell(0, 0), new Cell(9, 4)),

    ("No path", GridMap.FromStrings(
        "...@...",
        "...@...",
        "...@..."),
        new Cell(0, 0), new Cell(6, 2)),
};

// Run each search a few times and average the time .
const int Runs = 10;

Console.WriteLine($"{"Map",-20} {"Found",-6} {"Cost",8} {"Expanded",9} {"Avg ms",9}");
foreach (var (name, map, start, goal) in tests)
{
    AStar.FindPath(map, start, goal);

    var results = new List<SearchResult>();
    for (int i = 0; i < Runs; i++)
        results.Add(AStar.FindPath(map, start, goal));

    var r = results[0];
    double avgMs = results.Average(x => x.RunTimeMs);
    string cost = r.Found ? r.PathCost.ToString("F3") : "-";
    Console.WriteLine($"{name,-20} {r.Found,-6} {cost,8} {r.NodesExpanded,9} {avgMs,9:F4}");
}
