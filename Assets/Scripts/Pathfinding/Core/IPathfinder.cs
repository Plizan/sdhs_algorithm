using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    public delegate int Heuristic(GridPos a, GridPos b);

    public delegate void StepCallback(bool[] visited, GridPos? current);

    public interface IPathfinder
    {
        PathResult FindPath(GridGraph graph, GridPos start, GridPos goal, Heuristic heuristic = null);
        
        IEnumerable<PathfindingStep> FindPathSteps(GridGraph graph, GridPos start, GridPos goal, Heuristic heuristic = null);
    }
}
