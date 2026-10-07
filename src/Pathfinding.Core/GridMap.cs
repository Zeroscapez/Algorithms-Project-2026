using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace Pathfinding
{
    // The grid . Each cell is either open or blocked.
    // Rules: you can move 8 ways, straight costs 1, diagonal costs sqrt(2),
    // and no corner cutting - a diagonal only works if both cells next to it are open too.
    public class GridMap
    {
        public static readonly double Sqrt2 = Math.Sqrt(2);

        // The 8 directions, so we don't have to write them out everywhere.
        public static readonly int[] DirX = { 0, 1, 1, 1, 0, -1, -1, -1 };
        public static readonly int[] DirY = { -1, -1, 0, 1, 1, 1, 0, -1 };

        private readonly bool[] _open; // one flat array, cell (x, y) lives at y * Width + x

        public int Width { get; }
        public int Height { get; }
        public int CellCount => Width * Height;

        public GridMap(int width, int height, bool[] open)
        {
            if (open.Length != width * height)
                throw new ArgumentException("open needs exactly width * height entries");
            Width = width;
            Height = height;
            _open = open;
        }

        // Makes a map out of text rows. '.' is open (so are 'G' and 'S', Moving AI uses those for ground/swamp).
        // Anything else - '@', 'T' for trees, whatever - is a wall.
        public static GridMap FromStrings(params string[] rows)
        {
            int width = rows[0].Length;
            var open = new bool[width * rows.Length];
            for (int y = 0; y < rows.Length; y++)
            {
                if (rows[y].Length != width)
                    throw new FormatException($"Row {y} is {rows[y].Length} wide but row 0 is {width} wide.");
                for (int x = 0; x < width; x++)
                    open[y * width + x] = rows[y][x] == '.' || rows[y][x] == 'G' || rows[y][x] == 'S';
            }
            return new GridMap(width, rows.Length, open);
        }

        // Reads the contents of a map file. Works with Moving AI .map files (it just skips their header,
        // which ends with a line that says "map") and with plain files that are only rows.
        // In Unity you can hand this a TextAsset's .text.
        public static GridMap Parse(string text)
        {
            var lines = text.Split('\n')
                .Select(line => line.TrimEnd('\r', ' ', '\t'))
                .Where(line => line.Length > 0)
                .ToList();

            int mapLine = lines.FindIndex(line => line.Equals("map", StringComparison.OrdinalIgnoreCase));
            var rows = mapLine >= 0 ? lines.Skip(mapLine + 1).ToArray() : lines.ToArray();

            if (rows.Length == 0)
                throw new FormatException("Didn't find any map rows in there.");
            return FromStrings(rows);
        }

        public static GridMap Load(string path) => Parse(File.ReadAllText(path));

        public bool InBounds(int x, int y) => x >= 0 && y >= 0 && x < Width && y < Height;
        public bool IsOpen(int x, int y) => InBounds(x, y) && _open[y * Width + x];
        public bool IsOpen(Cell c) => IsOpen(c.X, c.Y);

        public int ToIndex(Cell c) => c.Y * Width + c.X;
        public Cell ToCell(int index) => new Cell(index % Width, index / Width);

        // Can we take one step from (x, y) in direction dir (0-7)? This is the hot path for searches,
        // so it doesn't allocate anything.
        public bool CanStep(int x, int y, int dir)
        {
            int dx = DirX[dir], dy = DirY[dir];
            if (!IsOpen(x + dx, y + dy)) return false;
            if (dx != 0 && dy != 0) // diagonal: no squeezing past a wall
                return IsOpen(x + dx, y) && IsOpen(x, y + dy);
            return true;
        }

        public static double StepCost(int dir) => (DirX[dir] != 0 && DirY[dir] != 0) ? Sqrt2 : 1.0;


        public List<(Cell Next, double Cost)> GetNeighbors(Cell c)
        {
            var result = new List<(Cell, double)>(8);
            for (int dir = 0; dir < 8; dir++)
                if (CanStep(c.X, c.Y, dir))
                    result.Add((new Cell(c.X + DirX[dir], c.Y + DirY[dir]), StepCost(dir)));
            return result;
        }
    }
}
