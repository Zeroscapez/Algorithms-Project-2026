using Pathfinding;
using Xunit;

public class GridMapTests
{
    [Fact]
    public void ReadsOpenAndBlockedCells()
    {
        var map = GridMap.FromStrings(".@", "..");
        Assert.Equal(2, map.Width);
        Assert.Equal(2, map.Height);
        Assert.True(map.IsOpen(0, 0));
        Assert.False(map.IsOpen(1, 0));
        Assert.False(map.IsOpen(5, 5)); // off the map = not open
    }

    [Fact]
    public void MiddleCellHasEightNeighbors()
    {
        var map = GridMap.FromStrings("...", "...", "...");
        Assert.Equal(8, map.GetNeighbors(new Cell(1, 1)).Count);
    }

    [Fact]
    public void CornerCellHasThreeNeighbors()
    {
        var map = GridMap.FromStrings("...", "...", "...");
        Assert.Equal(3, map.GetNeighbors(new Cell(0, 0)).Count);
    }

    [Fact]
    public void DiagonalsCostSqrt2()
    {
        var map = GridMap.FromStrings("..", "..");
        var step = map.GetNeighbors(new Cell(0, 0)).Single(n => n.Next == new Cell(1, 1));
        Assert.Equal(Math.Sqrt(2), step.Cost, 9);
    }

    [Fact]
    public void NoCornerCutting()
    {
        var map = GridMap.FromStrings(
            "..",
            "@.");
        // (0,0) -> (1,1) would squeeze past the wall at (0,1), not allowed
        Assert.DoesNotContain(map.GetNeighbors(new Cell(0, 0)), n => n.Next == new Cell(1, 1));
    }

    [Fact]
    public void LoadsMovingAiFormat()
    {
        var map = GridMap.Parse("type octile\nheight 3\nwidth 4\nmap\n.@T.\nG..S\n....\n");
        Assert.Equal(4, map.Width);
        Assert.Equal(3, map.Height);
        Assert.True(map.IsOpen(0, 0));
        Assert.False(map.IsOpen(1, 0)); // @
        Assert.False(map.IsOpen(2, 0)); // T = tree
        Assert.True(map.IsOpen(0, 1));  // G = ground
        Assert.True(map.IsOpen(3, 1));  // S = swamp
    }

    [Fact]
    public void LoadsPlainRowsWithWindowsLineEndings()
    {
        var map = GridMap.Parse("...\r\n.@.\r\n...\r\n");
        Assert.Equal(3, map.Width);
        Assert.False(map.IsOpen(1, 1));
    }

    [Fact]
    public void LoadsFromFile()
    {
        string path = Path.GetTempFileName();
        File.WriteAllText(path, "type octile\nheight 1\nwidth 3\nmap\n.@.\n");
        try
        {
            Assert.False(GridMap.Load(path).IsOpen(1, 0));
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void RejectsRowsOfDifferentLengths()
    {
        Assert.Throws<FormatException>(() => GridMap.FromStrings("...", ".."));
    }
}

public class MinHeapTests
{
    [Fact]
    public void PopsSmallestFFirst_TiesGoToBiggerG()
    {
        var heap = new MinHeap();
        heap.Push(1, 5, 1);
        heap.Push(2, 3, 0);
        heap.Push(3, 5, 4); // same f as item 1 but bigger g, so it should win the tie
        heap.Push(4, 1, 0);
        var order = new List<int>();
        while (heap.Count > 0) order.Add(heap.Pop());
        Assert.Equal(new[] { 4, 2, 3, 1 }, order);
    }

    [Fact]
    public void LotsOfRandomPushesComeOutSorted()
    {
        var rng = new Random(1);
        var heap = new MinHeap();
        var fs = new Dictionary<int, double>();
        for (int i = 0; i < 1000; i++)
        {
            fs[i] = rng.Next(100);
            heap.Push(i, fs[i], 0);
        }
        double last = -1;
        while (heap.Count > 0)
        {
            double f = fs[heap.Pop()];
            Assert.True(f >= last);
            last = f;
        }
    }
}
