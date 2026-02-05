using System;
using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    public sealed class GridGraph
    {
        private static readonly GridPos[] NeighborOffsets =
        {
            new GridPos(1, 0),
            new GridPos(-1, 0),
            new GridPos(0, 1),
            new GridPos(0, -1),
        };

        private readonly bool[] _blocked;
        private readonly int[] _cost;

        public int Width { get; }
        public int Height { get; }
        public int CellCount => _blocked.Length;

        public GridGraph(int width, int height, int defaultCost = 1)
        {
            if (width <= 0)
                throw new ArgumentOutOfRangeException(nameof(width), "Width must be positive.");
            if (height <= 0)
                throw new ArgumentOutOfRangeException(nameof(height), "Height must be positive.");
            if (defaultCost < 1)
                throw new ArgumentOutOfRangeException(nameof(defaultCost), "Default cost must be >= 1.");

            Width = width;
            Height = height;
            _blocked = new bool[width * height];
            _cost = new int[width * height];
            Array.Fill(_cost, defaultCost);
        }

        public bool InBounds(GridPos pos) => (uint)pos.X < (uint)Width && (uint)pos.Y < (uint)Height;

        public int ToIndex(GridPos pos) => pos.Y * Width + pos.X;

        public GridPos FromIndex(int index) => new GridPos(index % Width, index / Width);

        public bool IsBlocked(GridPos pos) => InBounds(pos) && _blocked[ToIndex(pos)];

        public bool IsWalkable(GridPos pos) => InBounds(pos) && !_blocked[ToIndex(pos)];

        public int Cost(GridPos pos) => _cost[ToIndex(pos)];

        public void SetBlocked(GridPos pos, bool blocked)
        {
            if (!InBounds(pos))
                return;

            _blocked[ToIndex(pos)] = blocked;
        }

        public void SetCost(GridPos pos, int cost)
        {
            if (!InBounds(pos))
                return;

            _cost[ToIndex(pos)] = Math.Max(1, cost);
        }

        public IEnumerable<GridPos> GetNeighbors(GridPos pos)
        {
            if (!InBounds(pos))
                yield break;

            for (int i = 0; i < NeighborOffsets.Length; i++)
            {
                var offset = NeighborOffsets[i];
                var next = new GridPos(pos.X + offset.X, pos.Y + offset.Y);
                if (IsWalkable(next))
                    yield return next;
            }
        }
    }
}
