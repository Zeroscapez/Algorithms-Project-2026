using Pathfinding;
using Xunit;

// Tests every algorithm has to pass. Each algorithm gets a tiny class at the bottom of this file.
// When JPS is ready, add:   public class JpsTests : PathfinderTests { protected override IPathfinder Create() => new Jps(); }
public abstract class PathfinderTests
{
    protected abstract IPathfinder Create();

    // Runs the search and checks the answer actually makes sense:
    // starts and ends in the right spot, only takes legal steps, and the cost adds up.
    private SearchResult Search(GridMap map, Cell start, Cell goal)
    {
        var finder = Create();
        finder.Preprocess(map);
        var r = finder.FindPath(map, start, goal);

        if (r.Found)
        {
            Assert.Equal(start, r.Path[0]);
            Assert.Equal(goal, r.Path[^1]);
            Assert.Equal(r.PathCost, CostOfPath(map, r.Path), 6);
        }
        else
        {
            Assert.Empty(r.Path);
            Assert.True(double.IsPositiveInfinity(r.PathCost));
        }
        return r;
    }

    // Walks the path one cell at a time (filling in the gaps between JPS jump points) and makes sure
    // every single step is legal. Returns the total cost.
    private static double CostOfPath(GridMap map, List<Cell> path)
    {
        double total = 0;
        for (int i = 1; i < path.Count; i++)
        {
            Cell a = path[i - 1], b = path[i];
            int distX = Math.Abs(b.X - a.X), distY = Math.Abs(b.Y - a.Y);
            Assert.True(distX == 0 || distY == 0 || distX == distY, $"{a} -> {b} isn't a straight or diagonal line");

            int dx = Math.Sign(b.X - a.X), dy = Math.Sign(b.Y - a.Y);

            for (Cell c = a; c != b; c = new Cell(c.X + dx, c.Y + dy))
            {
                var next = new Cell(c.X + dx, c.Y + dy);
                var step = map.GetNeighbors(c).Where(n => n.Next == next).ToList();
                Assert.True(step.Count == 1, $"Illegal step {c} -> {next} (wall or corner cut)");
                total += step[0].Cost;
            }
        }
        return total;
    }

    [Fact]
    public void StartIsGoal()
    {
        var r = Search(GridMap.FromStrings("..."), new Cell(1, 0), new Cell(1, 0));
        Assert.True(r.Found);
        Assert.Equal(0, r.PathCost);
    }

    [Fact]
    public void StraightLine()
    {
        var r = Search(GridMap.FromStrings("....."), new Cell(0, 0), new Cell(4, 0));
        Assert.Equal(4, r.PathCost, 9);
    }

    [Fact]
    public void UsesDiagonalsWhenShorter()
    {
        var map = GridMap.FromStrings("....", "....", "....", "....");
        Assert.Equal(3 * Math.Sqrt(2), Search(map, new Cell(0, 0), new Cell(3, 3)).PathCost, 9);
    }

    [Fact]
    public void GoesAroundAWall()
    {
        var map = GridMap.FromStrings(
            ".....",
            "..@..",
            "..@..",
            "..@..",
            ".....");
        // Over the top: diagonal up, 4 straight steps around the end of the wall (no corner cutting), diagonal down.
        Assert.Equal(4 + 2 * Math.Sqrt(2), Search(map, new Cell(0, 2), new Cell(4, 2)).PathCost, 9);
    }

    [Fact]
    public void FindsItsWayThroughAMaze()
    {
        var map = GridMap.FromStrings(
            ".@........",
            ".@.@@@@@@.",
            ".@.@......",
            ".@.@.@@@@@",
            "...@......");
        var r = Search(map, new Cell(0, 0), new Cell(9, 4));
        Assert.True(r.Found);
        Assert.Equal(DijkstraCost(map, new Cell(0, 0), new Cell(9, 4)), r.PathCost, 6);
    }

    [Fact]
    public void NoPathWhenWalledOff()
    {
        var r = Search(GridMap.FromStrings(".@.", ".@.", ".@."), new Cell(0, 0), new Cell(2, 0));
        Assert.False(r.Found);
    }

    [Fact]
    public void NoPathWhenGoalIsAWall()
    {
        Assert.False(Search(GridMap.FromStrings("..@"), new Cell(0, 0), new Cell(2, 0)).Found);
    }

    // The big one: random maps, random queries, and every answer has to match a dead-simple Dijkstra.
    // If JPS has a bug, this is where it'll show up.
    [Theory]
    [InlineData(0.0)]
    [InlineData(0.1)]
    [InlineData(0.25)]
    [InlineData(0.4)]
    public void MatchesDijkstraOnRandomMaps(double wallChance)
    {
        var rng = new Random(123);
        var open = new bool[30 * 20];
        for (int i = 0; i < open.Length; i++) open[i] = rng.NextDouble() >= wallChance;
        var map = new GridMap(30, 20, open);

        for (int q = 0; q < 40; q++)
        {
            var start = new Cell(rng.Next(30), rng.Next(20));
            var goal = new Cell(rng.Next(30), rng.Next(20));
            var r = Search(map, start, goal);
            double expected = DijkstraCost(map, start, goal);
            Assert.Equal(!double.IsInfinity(expected), r.Found);
            if (r.Found) Assert.Equal(expected, r.PathCost, 6);
        }
    }

    [Fact]
    public void CountsNodesAndTime()
    {
        var r = Search(GridMap.FromStrings("......", "......", "......"), new Cell(0, 0), new Cell(5, 2));
        Assert.True(r.NodesExpanded > 0);
        Assert.True(r.RunTimeMs >= 0);
    }

    // The recorder (for the Unity visual sim later) shouldn't change the answer at all.
    [Fact]
    public void RecorderDoesNotChangeTheResult()
    {
        var map = GridMap.FromStrings(
            ".....",
            "..@..",
            "..@..",
            ".....");
        var finder = Create();
        finder.Preprocess(map);
        var plain = finder.FindPath(map, new Cell(0, 1), new Cell(4, 1));
        var recorder = new SearchRecorder();
        var recorded = finder.FindPath(map, new Cell(0, 1), new Cell(4, 1), recorder);

        Assert.Equal(plain.PathCost, recorded.PathCost);
        Assert.Equal(plain.NodesExpanded, recorded.NodesExpanded);
        Assert.Equal(plain.NodesExpanded, recorder.Events.Count(e => e.Type == SearchEventType.Expanded));
    }

    // Super basic Dijkstra (no heuristic, no tricks) to check answers against.
    private static double DijkstraCost(GridMap map, Cell start, Cell goal)
    {
        if (!map.IsOpen(start) || !map.IsOpen(goal)) return double.PositiveInfinity;
        var dist = new Dictionary<Cell, double> { [start] = 0 };
        var queue = new PriorityQueue<Cell, double>();
        queue.Enqueue(start, 0);
        while (queue.TryDequeue(out var c, out double d))
        {
            if (d > dist[c]) continue;
            if (c == goal) return d;
            foreach (var (next, cost) in map.GetNeighbors(c))
            {
                if (dist.TryGetValue(next, out double old) && old <= d + cost) continue;
                dist[next] = d + cost;
                queue.Enqueue(next, d + cost);
            }
        }
        return double.PositiveInfinity;
    }
}

public class AStarTests : PathfinderTests
{
    protected override IPathfinder Create() => new AStar();
}
