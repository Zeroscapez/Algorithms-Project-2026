using System;

namespace Pathfinding
{
    // The open list. .NET has PriorityQueue but Unity doesn't, so here's a small one of our own.
    // It holds cell indexes sorted by f (lowest first). If two have the same f, the one with the bigger g
    // comes out first - that's the one closer to the goal, and it cuts down on wasted expansions.
    // JPS and JPS+ should use this too, so the comparison between algorithms stays fair.
    public class MinHeap
    {
        private int[] _items = new int[256];
        private double[] _f = new double[256];
        private double[] _g = new double[256];

        public int Count { get; private set; }

        public void Clear() => Count = 0;

        public void Push(int item, double f, double g)
        {
            if (Count == _items.Length)
            {
                Array.Resize(ref _items, Count * 2);
                Array.Resize(ref _f, Count * 2);
                Array.Resize(ref _g, Count * 2);
            }

            // Drop it at the bottom and bubble it up.
            int i = Count++;
            while (i > 0)
            {
                int parent = (i - 1) / 2;
                if (!ComesFirst(f, g, _f[parent], _g[parent])) break;
                Move(parent, i);
                i = parent;
            }
            Set(i, item, f, g);
        }

        public int Pop()
        {
            int top = _items[0];
            Count--;
            if (Count == 0) return top;

            // Take the last item, put it at the top, and sink it down.
            int item = _items[Count];
            double f = _f[Count], g = _g[Count];
            int i = 0;
            while (true)
            {
                int child = 2 * i + 1;
                if (child >= Count) break;
                if (child + 1 < Count && ComesFirst(_f[child + 1], _g[child + 1], _f[child], _g[child])) child++;
                if (!ComesFirst(_f[child], _g[child], f, g)) break;
                Move(child, i);
                i = child;
            }
            Set(i, item, f, g);
            return top;
        }

        private static bool ComesFirst(double fa, double ga, double fb, double gb) => fa < fb || (fa == fb && ga > gb);

        private void Move(int from, int to) => Set(to, _items[from], _f[from], _g[from]);

        private void Set(int i, int item, double f, double g)
        {
            _items[i] = item;
            _f[i] = f;
            _g[i] = g;
        }
    }
}
