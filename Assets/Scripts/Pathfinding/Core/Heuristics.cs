using System;

namespace Sdhs.Pathfinding.Core
{
    public static class Heuristics
    {
        public static int Manhattan(GridPos a, GridPos b)
        {
            return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
        }

        public static int Euclidean(GridPos a, GridPos b)
        {
            int dx = a.X - b.X;
            int dy = a.Y - b.Y;
            return (int)Math.Round(Math.Sqrt(dx * dx + dy * dy));
        }
    }
}
