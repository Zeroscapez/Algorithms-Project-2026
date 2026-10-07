using System.Collections.Generic;

namespace Pathfinding
{
    public enum SearchEventType
    {
        Opened,   // cell got added to the open list (the "frontier")
        Expanded, // cell got popped off the open list and expanded
    }

    public readonly struct SearchEvent
    {
        public readonly SearchEventType Type;
        public readonly Cell Cell;

        public SearchEvent(SearchEventType type, Cell cell)
        {
            Type = type;
            Cell = cell;
        }
    }

    // How the Unity visual sim gets its data without slowing anything down:
    //   1. the search runs at full speed and just jots down what it did in here,
    //   2. Unity replays the list afterwards at whatever speed looks good.
    // So the animation never touches the search itself. And when you're benchmarking you pass null,
    // which costs one "is it null?" check per step - basically nothing.
    public class SearchRecorder
    {
        public readonly List<SearchEvent> Events = new List<SearchEvent>();

        public void Clear() => Events.Clear();
        public void Opened(Cell c) => Events.Add(new SearchEvent(SearchEventType.Opened, c));
        public void Expanded(Cell c) => Events.Add(new SearchEvent(SearchEventType.Expanded, c));
    }
}
