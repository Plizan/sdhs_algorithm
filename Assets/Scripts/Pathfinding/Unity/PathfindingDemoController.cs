using System;
using Sdhs.Pathfinding.Core;
using UnityEngine;

namespace Sdhs.Pathfinding.Unity
{
    public enum AlgorithmType
    {
        BFS,
        Dijkstra,
        AStar,
        GreedyBestFirst,
        DirectionalGreedy,
        WallFollower
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
        private float _lastSearchTimeMs;

        public PathResult LastResult => _lastResult;
        public AlgorithmType CurrentAlgorithm => algorithm;
        public int SeedUsed => _seedUsed;
        public int Width => width;
        public int Height => height;
        public bool IsAnimating => _isAnimating;
        public float LastSearchTimeMs => _lastSearchTimeMs;

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

            // 시작/도착 지점 랜덤 생성
            start = new Vector2Int(rng.Next(0, width), rng.Next(0, height));
            goal = new Vector2Int(rng.Next(0, width), rng.Next(0, height));
            
            // 시작과 도착이 너무 가까우면 다시 생성
            while (Vector2Int.Distance(start, goal) < Mathf.Min(width, height) * 0.3f)
            {
                goal = new Vector2Int(rng.Next(0, width), rng.Next(0, height));
            }

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
            var heuristic = (algorithm == AlgorithmType.AStar || algorithm == AlgorithmType.GreedyBestFirst) 
                ? (Heuristic)Heuristics.Manhattan : null;
            
            var startTime = System.DateTime.Now;
            _lastResult = pathfinder.FindPath(_graph, startPos, goalPos, heuristic);
            var endTime = System.DateTime.Now;
            _lastSearchTimeMs = (float)(endTime - startTime).TotalMilliseconds;

            if (rendererTarget != null)
                rendererTarget.Render(_graph, _lastResult, startPos, goalPos, maxCost);
        }

        private System.Collections.IEnumerator SolveAnimated()
        {
            _isAnimating = true;
            var startPos = ToGridPos(start);
            var goalPos = ToGridPos(goal);

            var pathfinder = CreatePathfinder(algorithm);
            var heuristic = (algorithm == AlgorithmType.AStar || algorithm == AlgorithmType.GreedyBestFirst) 
                ? (Heuristic)Heuristics.Manhattan : null;

            var startTime = DateTime.Now;
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

            var endTime = DateTime.Now;
            _lastSearchTimeMs = (float)(endTime - startTime).TotalMilliseconds;

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
                AlgorithmType.BFS => "균등 비용 맵에서 최고 효율; 최단 경로 보장",
                AlgorithmType.Dijkstra => "가중치 맵; 비용 고려하므로 BFS보다 느림",
                AlgorithmType.AStar => "가중치 맵 + 휴리스틱; Dijkstra보다 빠르고 최적",
                AlgorithmType.GreedyBestFirst => "거리만 봄; 빠르지만 최단 경로 보장 안 함",
                AlgorithmType.DirectionalGreedy => "목표 방향으로만 직진; 막히면 실패 (가장 빠르지만 위험)",
                AlgorithmType.WallFollower => "벽 따라가기 + 백트래킹; 비효율적",
                _ => string.Empty
            };
        }

        public string GetAlgorithmLabel()
        {
            return algorithm switch
            {
                AlgorithmType.BFS => "BFS (너비 우선)",
                AlgorithmType.Dijkstra => "Dijkstra (다익스트라)",
                AlgorithmType.AStar => "A* (에이스타)",
                AlgorithmType.GreedyBestFirst => "Greedy (그리디)",
                AlgorithmType.DirectionalGreedy => "Dir.Greedy (방향 탐욕)",
                AlgorithmType.WallFollower => "Wall Follower (벽 따라가기)",
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
                AlgorithmType.GreedyBestFirst => new GreedyBestFirstPathfinder(),
                AlgorithmType.DirectionalGreedy => new DirectionalGreedyPathfinder(),
                AlgorithmType.WallFollower => new WallFollowerPathfinder(),
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
