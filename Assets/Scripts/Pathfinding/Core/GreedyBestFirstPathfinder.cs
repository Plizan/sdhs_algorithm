using System;
using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    /// <summary>
    /// 그리디 베스트 퍼스트 서치 - 휴리스틱만 보고 목표에 가까운 곳으로 이동 (최적 경로 보장 안 됨)
    /// </summary>
    public sealed class GreedyBestFirstPathfinder : IPathfinder
    {
        public IEnumerable<PathfindingStep> FindPathSteps(GridGraph graph, GridPos start, GridPos goal, Heuristic heuristic = null)
        {
            var visited = new bool[graph.CellCount];
            var prev = new int[graph.CellCount];
            Array.Fill(prev, -1);

            if (!graph.IsWalkable(start) || !graph.IsWalkable(goal))
            {
                yield return new PathfindingStep(visited, null, prev, 0, true, false);
                yield break;
            }

            int startIndex = graph.ToIndex(start);
            int goalIndex = graph.ToIndex(goal);
            var heap = new MinHeap<int>();
            var h = heuristic ?? Heuristics.Manhattan;

            heap.Enqueue(startIndex, h(start, goal));
            int visitedCount = 0;

            while (heap.Count > 0)
            {
                var (currentIndex, _) = heap.Dequeue();
                
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

                    if (prev[neighborIndex] == -1)
                    {
                        prev[neighborIndex] = currentIndex;
                        int hScore = h(neighbor, goal);
                        heap.Enqueue(neighborIndex, hScore);
                    }
                }
            }

            yield return new PathfindingStep((bool[])visited.Clone(), null, (int[])prev.Clone(), visitedCount, true, false);
        }

        public PathResult FindPath(GridGraph graph, GridPos start, GridPos goal, Heuristic heuristic = null)
        {
            var visited = new bool[graph.CellCount];
            var prev = new int[graph.CellCount];
            Array.Fill(prev, -1);

            if (!graph.IsWalkable(start) || !graph.IsWalkable(goal))
                return new PathResult(false, 0, 0, new List<GridPos>(), visited);

            int startIndex = graph.ToIndex(start);
            int goalIndex = graph.ToIndex(goal);
            var heap = new MinHeap<int>();
            var h = heuristic ?? Heuristics.Manhattan;

            heap.Enqueue(startIndex, h(start, goal));
            int visitedCount = 0;

            while (heap.Count > 0)
            {
                var (currentIndex, _) = heap.Dequeue();
                
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

                    if (prev[neighborIndex] == -1)
                    {
                        prev[neighborIndex] = currentIndex;
                        int hScore = h(neighbor, goal);
                        heap.Enqueue(neighborIndex, hScore);
                    }
                }
            }

            var path = ReconstructPath(graph, startIndex, goalIndex, prev);
            
            int cost = 0;
            if (path.Count > 1)
            {
                for (int i = 1; i < path.Count; i++)
                    cost += graph.Cost(path[i]);
            }
            
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
