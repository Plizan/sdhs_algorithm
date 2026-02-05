using System;
using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    /// <summary>
    /// 목표 방향으로만 쭉 가는 알고리즘 - 목표 방향을 정확히 알 때 최고 효율
    /// 막히면 우회하지 않고 실패함 (가장 단순하고 빠름)
    /// </summary>
    public sealed class DirectionalGreedyPathfinder : IPathfinder
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

            var path = new List<GridPos>();
            var current = start;
            int visitedCount = 0;
            int maxSteps = graph.CellCount;
            int steps = 0;

            visited[graph.ToIndex(current)] = true;
            path.Add(current);
            visitedCount++;

            yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, false, false);

            while (current != goal && steps < maxSteps)
            {
                steps++;

                // 목표 방향으로 한 칸 이동 시도
                GridPos next = GetNextStep(current, goal);
                
                if (!graph.IsWalkable(next))
                {
                    // 목표 방향으로 갈 수 없으면 실패
                    yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, true, false);
                    yield break;
                }

                current = next;
                int currentIndex = graph.ToIndex(current);
                
                if (!visited[currentIndex])
                {
                    visited[currentIndex] = true;
                    visitedCount++;
                }
                
                path.Add(current);

                yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, false, false);

                if (current == goal)
                {
                    for (int i = 1; i < path.Count; i++)
                    {
                        prev[graph.ToIndex(path[i])] = graph.ToIndex(path[i - 1]);
                    }
                    yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, true, true);
                    yield break;
                }
            }

            yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, true, false);
        }

        public PathResult FindPath(GridGraph graph, GridPos start, GridPos goal, Heuristic heuristic = null)
        {
            PathfindingStep lastStep = null;
            foreach (var step in FindPathSteps(graph, start, goal, heuristic))
            {
                lastStep = step;
                if (step.IsComplete)
                    break;
            }

            if (lastStep == null || !lastStep.PathFound)
                return new PathResult(false, 0, lastStep?.VisitedCount ?? 0, new List<GridPos>(), lastStep?.Visited ?? new bool[graph.CellCount]);

            var path = ReconstructPath(graph, graph.ToIndex(start), graph.ToIndex(goal), lastStep.Prev);
            int cost = path.Count > 0 ? path.Count - 1 : 0;
            return new PathResult(true, cost, lastStep.VisitedCount, path, lastStep.Visited);
        }

        private static List<GridPos> ReconstructPath(GridGraph graph, int startIndex, int goalIndex, int[] prev)
        {
            if (prev[goalIndex] == -1)
                return new List<GridPos>();

            var path = new List<GridPos>();
            int current = goalIndex;
            while (current != -1 && current != startIndex)
            {
                path.Add(graph.FromIndex(current));
                current = prev[current];
            }
            
            if (current == startIndex)
                path.Add(graph.FromIndex(startIndex));

            path.Reverse();
            return path;
        }

        /// <summary>
        /// 목표를 향한 다음 스텝을 결정합니다.
        /// X 차이가 크면 X 방향, Y 차이가 크면 Y 방향으로 이동합니다.
        /// </summary>
        private static GridPos GetNextStep(GridPos current, GridPos goal)
        {
            int dx = goal.X - current.X;
            int dy = goal.Y - current.Y;

            if (Math.Abs(dx) >= Math.Abs(dy))
            {
                // X 방향이 더 멀면 X로 이동
                return dx > 0 
                    ? new GridPos(current.X + 1, current.Y) 
                    : new GridPos(current.X - 1, current.Y);
            }
            else
            {
                // Y 방향이 더 멀면 Y로 이동
                return dy > 0 
                    ? new GridPos(current.X, current.Y + 1) 
                    : new GridPos(current.X, current.Y - 1);
            }
        }
    }
}
