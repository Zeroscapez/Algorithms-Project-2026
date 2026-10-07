using System.Collections.Generic;

namespace Pathfinding
{
    // What a search hands back
    public class SearchResult
    {
        public bool Found;

        // Start to goal, both included. Empty if there's no path.
        // JPS/JPS+ can just return the jump points here - the steps between them are straight or diagonal lines.
        public List<Cell> Path = new List<Cell>();

        public double PathCost = double.PositiveInfinity; // stays infinity if there's no path
        public int NodesExpanded;                          // how many cells got popped off the open list and expanded
        public double RunTimeMs;
    }
}
