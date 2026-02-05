using System;
using Sdhs.Pathfinding.Core;
using UnityEngine;

namespace Sdhs.Pathfinding.Unity
{
    public enum AlgorithmType
    {
        BFS,
        Dijkstra,
        AStar
    }

    [DisallowMultipleComponent]
    public sealed class PathfindingDemoController : MonoBehaviour
    {
        [Header("Grid")]
        [SerializeField] private int width = 24;
        [SerializeField] private int height = 24;
        [Range(0f, 0.6f)]
        [SerializeField] private float obstacleChance = 0.2f;
        [SerializeField] private bool useWeightedCosts = true;
        [SerializeField] private int minCost = 1;
        [SerializeField] private int maxCost = 5;

        [Header("Start / Goal")]
        [SerializeField] private Vector2Int start = new(1, 1);
        [SerializeField] private Vector2Int goal = new(20, 10);

        [Header("Algorithm")]
        [SerializeField] private AlgorithmType algorithm = AlgorithmType.AStar;

        [Header("Animation")]
        [SerializeField] private bool animateSteps = true;
        [SerializeField] private float stepDelay = 0.05f;

        [Header("Random")]
        [SerializeField] private int randomSeed = 1234;
        [SerializeField] private bool randomizeSeedOnStart = true;

        [Header("Rendering")]
        [SerializeField] private GridTextureRenderer rendererTarget;

        private GridGraph _graph;
        private PathResult _lastResult;
        private int _seedUsed;
        private Coroutine _animationCoroutine;
        private bool _isAnimating;

        public PathResult LastResult => _lastResult;
        public AlgorithmType CurrentAlgorithm => algorithm;
        public int SeedUsed => _seedUsed;
        public int Width => width;
        public int Height => height;
        public bool IsAnimating => _isAnimating;

        private void Awake()
        {
            if (rendererTarget == null)
                rendererTarget = GetComponent<GridTextureRenderer>();
        }

        private void Start()
        {
            GenerateAndSolve();
        }

        private void Update()
        {
            if (_isAnimating)
                return;

            if (Input.GetKeyDown(KeyCode.R))
                GenerateAndSolve();
            if (Input.GetKeyDown(KeyCode.Space))
                Solve();
            if (Input.GetKeyDown(KeyCode.Tab))
                CycleAlgorithm();
            if (Input.GetKeyDown(KeyCode.A))
                ToggleAnimation();
        }

        public void GenerateAndSolve()
        {
            Generate();
            Solve();
        }

        public void Generate()
        {
            _seedUsed = randomizeSeedOnStart ? Environment.TickCount : randomSeed;
            var rng = new System.Random(_seedUsed);

            _graph = new GridGraph(width, height, defaultCost: 1);

            var startPos = ToGridPos(start);
            var goalPos = ToGridPos(goal);

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    var pos = new GridPos(x, y);
                    if (pos == startPos || pos == goalPos)
                        continue;

                    if (rng.NextDouble() < obstacleChance)
                        _graph.SetBlocked(pos, true);
                    else if (useWeightedCosts)
                        _graph.SetCost(pos, rng.Next(minCost, maxCost + 1));
                }
            }
        }

        public void Solve()
        {
            if (_graph == null)
                return;

            if (_animationCoroutine != null)
                StopCoroutine(_animationCoroutine);

            if (animateSteps)
            {
                _animationCoroutine = StartCoroutine(SolveAnimated());
            }
            else
            {
                SolveInstant();
            }
        }

        private void SolveInstant()
        {
            var startPos = ToGridPos(start);
            var goalPos = ToGridPos(goal);

            var pathfinder = CreatePathfinder(algorithm);
            var heuristic = algorithm == AlgorithmType.AStar ? (Heuristic)Heuristics.Manhattan : null;
            _lastResult = pathfinder.FindPath(_graph, startPos, goalPos, heuristic);

            if (rendererTarget != null)
                rendererTarget.Render(_graph, _lastResult, startPos, goalPos, maxCost);
        }

        private System.Collections.IEnumerator SolveAnimated()
        {
            _isAnimating = true;
            var startPos = ToGridPos(start);
            var goalPos = ToGridPos(goal);

            var pathfinder = CreatePathfinder(algorithm);
            var heuristic = algorithm == AlgorithmType.AStar ? (Heuristic)Heuristics.Manhattan : null;

            PathfindingStep lastStep = null;

            foreach (var step in pathfinder.FindPathSteps(_graph, startPos, goalPos, heuristic))
            {
                lastStep = step;

                var partialPath = step.IsComplete && step.PathFound
                    ? ReconstructPathFromStep(_graph, ToIndex(startPos), ToIndex(goalPos), step.Prev)
                    : new System.Collections.Generic.List<GridPos>();

                var partialResult = new PathResult(
                    step.PathFound,
                    0,
                    step.VisitedCount,
                    partialPath,
                    step.Visited
                );

                if (rendererTarget != null)
                    rendererTarget.Render(_graph, partialResult, startPos, goalPos, maxCost);

                yield return new WaitForSeconds(stepDelay);
            }

            if (lastStep != null && lastStep.IsComplete)
            {
                var finalPath = lastStep.PathFound
                    ? ReconstructPathFromStep(_graph, ToIndex(startPos), ToIndex(goalPos), lastStep.Prev)
                    : new System.Collections.Generic.List<GridPos>();

                int cost = 0;
                if (finalPath.Count > 1)
                {
                    for (int i = 1; i < finalPath.Count; i++)
                        cost += _graph.Cost(finalPath[i]);
                }

                _lastResult = new PathResult(lastStep.PathFound, cost, lastStep.VisitedCount, finalPath, lastStep.Visited);

                if (rendererTarget != null)
                    rendererTarget.Render(_graph, _lastResult, startPos, goalPos, maxCost);
            }

            _isAnimating = false;
            _animationCoroutine = null;
        }

        private System.Collections.Generic.List<GridPos> ReconstructPathFromStep(GridGraph graph, int startIndex, int goalIndex, int[] prev)
        {
            if (startIndex == goalIndex)
                return new System.Collections.Generic.List<GridPos> { graph.FromIndex(startIndex) };

            if (prev[goalIndex] == -1)
                return new System.Collections.Generic.List<GridPos>();

            var path = new System.Collections.Generic.List<GridPos>();
            int current = goalIndex;
            while (current != -1)
            {
                path.Add(graph.FromIndex(current));
                current = prev[current];
            }

            path.Reverse();
            return path;
        }

        private int ToIndex(GridPos pos) => _graph.ToIndex(pos);

        public string GetUseCaseText()
        {
            return algorithm switch
            {
                AlgorithmType.BFS => "Use when all edges have equal cost; shortest by steps but ignores weights.",
                AlgorithmType.Dijkstra => "Use for weighted maps; guarantees shortest cost but explores more.",
                AlgorithmType.AStar => "Use for weighted maps with a good heuristic; faster, still optimal.",
                _ => string.Empty
            };
        }

        public string GetAlgorithmLabel()
        {
            return algorithm switch
            {
                AlgorithmType.BFS => "BFS",
                AlgorithmType.Dijkstra => "Dijkstra",
                AlgorithmType.AStar => "A*",
                _ => "Unknown"
            };
        }

        public Vector2Int GetStart() => start;
        public Vector2Int GetGoal() => goal;

        private static GridPos ToGridPos(Vector2Int pos) => new GridPos(pos.x, pos.y);

        private static IPathfinder CreatePathfinder(AlgorithmType type)
        {
            return type switch
            {
                AlgorithmType.BFS => new BFSPathfinder(),
                AlgorithmType.Dijkstra => new DijkstraPathfinder(),
                AlgorithmType.AStar => new AStarPathfinder(),
                _ => new BFSPathfinder()
            };
        }

        private void CycleAlgorithm()
        {
            algorithm = (AlgorithmType)(((int)algorithm + 1) % Enum.GetValues(typeof(AlgorithmType)).Length);
            Solve();
        }

        private void ToggleAnimation()
        {
            animateSteps = !animateSteps;
        }
    }
}
