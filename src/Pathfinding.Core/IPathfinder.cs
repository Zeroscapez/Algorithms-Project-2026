namespace Pathfinding
{
    // Every algorithm (A* now, JPS and JPS+ later) implements this, so the runner, the tests
    // and the Unity visualizer can treat them all the same way.
    //
    // To add JPS: make a class Jps : IPathfinder, then add it to the list in Program.cs
    // and add a JpsTests class next to AStarTests (one line, see PathfinderTests.cs).
    public interface IPathfinder
    {
        // Short name for printing, like "A*", "JPS", "JPS+".
        string Name { get; }

        // Called once per map before any searches on it. A* and JPS don't need anything here,
        // but JPS+ builds its jump-distance table in here. The runner times this separately.
        void Preprocess(GridMap map);

        // Find the cheapest path from start to goal.
        // recorder is optional - leave it null when benchmarking, pass one in from Unity to get
        // the step-by-step events for the animation (see SearchRecorder.cs).
        SearchResult FindPath(GridMap map, Cell start, Cell goal, SearchRecorder recorder = null);
    }
}
