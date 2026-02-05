using System;
using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    public sealed class AStarPathfinder : IPathfinder
    {
        public IEnumerable<PathfindingStep> FindPathSteps(GridGraph graph, GridPos start, GridPos goal, Heuristic heuristic = null)
        {
            var visited = new bool[graph.CellCount];
            var prev = new int[graph.CellCount];
            var gScore = new int[graph.CellCount];
            Array.Fill(prev, -1);
            Array.Fill(gScore, int.MaxValue);

            if (!graph.IsWalkable(start) || !graph.IsWalkable(goal))
            {
                yield return new PathfindingStep(visited, null, prev, 0, true, false);
                yield break;
            }

            int startIndex = graph.ToIndex(start);
            int goalIndex = graph.ToIndex(goal);
            var heap = new MinHeap<int>();
            var h = heuristic ?? Heuristics.Manhattan;

            gScore[startIndex] = 0;
            heap.Enqueue(startIndex, h(start, goal));
            int visitedCount = 0;

            while (heap.Count > 0)
            {
                var (currentIndex, priority) = heap.Dequeue();
                int currentBest = gScore[currentIndex] + h(graph.FromIndex(currentIndex), goal);
                if (priority != currentBest)
                    continue;
                if (visited[currentIndex])
                    continue;

                visited[currentIndex] = true;
                visitedCount++;
                var current = graph.FromIndex(currentIndex);

                yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, false, false);

                if (currentIndex == goalIndex)
                {
                    yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, true, true);
                    yield break;
                }

                foreach (var neighbor in graph.GetNeighbors(current))
                {
                    int neighborIndex = graph.ToIndex(neighbor);
                    if (visited[neighborIndex])
                        continue;

                    int tentative = gScore[currentIndex] + graph.Cost(neighbor);
                    if (tentative < gScore[neighborIndex])
                    {
                        gScore[neighborIndex] = tentative;
                        prev[neighborIndex] = currentIndex;
                        int fScore = tentative + h(neighbor, goal);
                        heap.Enqueue(neighborIndex, fScore);
                    }
                }
            }

            yield return new PathfindingStep((bool[])visited.Clone(), null, (int[])prev.Clone(), visitedCount, true, false);
        }

        public PathResult FindPath(GridGraph graph, GridPos start, GridPos goal, Heuristic heuristic = null)
        {
            var visited = new bool[graph.CellCount];
            var prev = new int[graph.CellCount];
            var gScore = new int[graph.CellCount];
            Array.Fill(prev, -1);
            Array.Fill(gScore, int.MaxValue);

            if (!graph.IsWalkable(start) || !graph.IsWalkable(goal))
                return new PathResult(false, 0, 0, new List<GridPos>(), visited);

            int startIndex = graph.ToIndex(start);
            int goalIndex = graph.ToIndex(goal);
            var heap = new MinHeap<int>();
            var h = heuristic ?? Heuristics.Manhattan;

            gScore[startIndex] = 0;
            heap.Enqueue(startIndex, h(start, goal));
            int visitedCount = 0;

            while (heap.Count > 0)
            {
                var (currentIndex, priority) = heap.Dequeue();
                int currentBest = gScore[currentIndex] + h(graph.FromIndex(currentIndex), goal);
                if (priority != currentBest)
                    continue;
                if (visited[currentIndex])
                    continue;

                visited[currentIndex] = true;
                visitedCount++;

                if (currentIndex == goalIndex)
                    break;

                var current = graph.FromIndex(currentIndex);
                foreach (var neighbor in graph.GetNeighbors(current))
                {
                    int neighborIndex = graph.ToIndex(neighbor);
                    if (visited[neighborIndex])
                        continue;

                    int tentative = gScore[currentIndex] + graph.Cost(neighbor);
                    if (tentative < gScore[neighborIndex])
                    {
                        gScore[neighborIndex] = tentative;
                        prev[neighborIndex] = currentIndex;
                        int fScore = tentative + h(neighbor, goal);
                        heap.Enqueue(neighborIndex, fScore);
                    }
                }
            }

            var path = ReconstructPath(graph, startIndex, goalIndex, prev);
            int cost = path.Count > 0 ? gScore[goalIndex] : 0;
            return new PathResult(path.Count > 0, cost, visitedCount, path, visited);
        }

        private static List<GridPos> ReconstructPath(GridGraph graph, int startIndex, int goalIndex, int[] prev)
        {
            if (startIndex == goalIndex)
                return new List<GridPos> { graph.FromIndex(startIndex) };

            if (prev[goalIndex] == -1)
                return new List<GridPos>();

            var path = new List<GridPos>();
            int current = goalIndex;
            while (current != -1)
            {
                path.Add(graph.FromIndex(current));
                current = prev[current];
            }

            path.Reverse();
            return path;
        }
    }
}
