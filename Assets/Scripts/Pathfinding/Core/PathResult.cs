using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    public sealed class PathResult
    {
        public bool Success { get; }
        public int PathCost { get; }
        public int VisitedCount { get; }
        public IReadOnlyList<GridPos> Path { get; }
        public bool[] VisitedMap { get; }
        public int PathLength => Path.Count;

        public PathResult(bool success, int pathCost, int visitedCount, List<GridPos> path, bool[] visitedMap)
        {
            Success = success;
            PathCost = pathCost;
            VisitedCount = visitedCount;
            Path = path;
            VisitedMap = visitedMap;
        }
    }
}
