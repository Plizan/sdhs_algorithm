namespace Sdhs.Pathfinding.Core
{
    public sealed class PathfindingStep
    {
        public bool[] Visited { get; }
        public GridPos? CurrentPos { get; }
        public int[] Prev { get; }
        public int VisitedCount { get; }
        public bool IsComplete { get; }
        public bool PathFound { get; }

        public PathfindingStep(bool[] visited, GridPos? currentPos, int[] prev, int visitedCount, bool isComplete, bool pathFound)
        {
            Visited = visited;
            CurrentPos = currentPos;
            Prev = prev;
            VisitedCount = visitedCount;
            IsComplete = isComplete;
            PathFound = pathFound;
        }
    }
}
