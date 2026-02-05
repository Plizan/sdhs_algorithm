using System;
using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    public sealed class WallFollowerPathfinder : IPathfinder
    {
        private enum Direction { Up, Right, Down, Left }

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
            var direction = GetInitialDirection(start, goal);
            int visitedCount = 0;
            int maxSteps = graph.CellCount * 4;
            int steps = 0;

            visited[graph.ToIndex(current)] = true;
            path.Add(current);
            visitedCount++;

            yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, false, false);

            while (current != goal && steps < maxSteps)
            {
                steps++;
                
                GridPos? next = null;
                int rotations = 0;

                // 네 방향 모두 시도
                while (rotations < 4 && !next.HasValue)
                {
                    var testPos = Move(current, direction);
                    
                    if (graph.IsWalkable(testPos) && !visited[graph.ToIndex(testPos)])
                    {
                        next = testPos;
                        break;
                    }
                    
                    direction = TurnRight(direction);
                    rotations++;
                }

                // 사방이 막혔으면 백트래킹
                if (!next.HasValue)
                {
                    if (path.Count <= 1)
                    {
                        // 더 이상 돌아갈 곳이 없음
                        yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, true, false);
                        yield break;
                    }
                    
                    // 이전 위치로 백트래킹
                    path.RemoveAt(path.Count - 1);
                    current = path[path.Count - 1];
                    
                    // 방향을 목표 쪽으로 재설정
                    direction = GetInitialDirection(current, goal);
                    
                    yield return new PathfindingStep((bool[])visited.Clone(), current, (int[])prev.Clone(), visitedCount, false, false);
                    continue;
                }

                current = next.Value;
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

        private static Direction GetInitialDirection(GridPos start, GridPos goal)
        {
            int dx = goal.X - start.X;
            int dy = goal.Y - start.Y;

            if (Math.Abs(dx) > Math.Abs(dy))
                return dx > 0 ? Direction.Right : Direction.Left;
            else
                return dy > 0 ? Direction.Down : Direction.Up;
        }

        private static Direction TurnRight(Direction dir) => dir switch
        {
            Direction.Up => Direction.Right,
            Direction.Right => Direction.Down,
            Direction.Down => Direction.Left,
            Direction.Left => Direction.Up,
            _ => dir
        };

        private static GridPos Move(GridPos pos, Direction dir) => dir switch
        {
            Direction.Up => new GridPos(pos.X, pos.Y - 1),
            Direction.Down => new GridPos(pos.X, pos.Y + 1),
            Direction.Left => new GridPos(pos.X - 1, pos.Y),
            Direction.Right => new GridPos(pos.X + 1, pos.Y),
            _ => pos
        };
    }
}
