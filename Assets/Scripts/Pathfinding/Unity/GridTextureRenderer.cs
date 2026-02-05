using Sdhs.Pathfinding.Core;
using UnityEngine;

namespace Sdhs.Pathfinding.Unity
{
    [RequireComponent(typeof(SpriteRenderer))]
    public sealed class GridTextureRenderer : MonoBehaviour
    {
        [SerializeField] private float cellSize = 0.5f;
        [SerializeField] private bool showCostTint = true;
        [SerializeField] private Color baseColor = new(0.2f, 0.2f, 0.2f, 1f);
        [SerializeField] private Color blockedColor = new(0.05f, 0.05f, 0.05f, 1f);
        [SerializeField] private Color visitedColor = new(0.2f, 0.6f, 0.9f, 1f);
        [SerializeField] private Color pathColor = new(1f, 0.9f, 0.2f, 1f);
        [SerializeField] private Color startColor = new(0.2f, 0.9f, 0.2f, 1f);
        [SerializeField] private Color goalColor = new(0.9f, 0.2f, 0.2f, 1f);
        [SerializeField] private Color costTintColor = new(0.9f, 0.6f, 0.2f, 1f);

        private Texture2D _texture;
        private SpriteRenderer _spriteRenderer;
        private int _width;
        private int _height;

        private void Awake()
        {
            _spriteRenderer = GetComponent<SpriteRenderer>();
        }

        public void Render(GridGraph graph, PathResult result, GridPos start, GridPos goal, int maxCost)
        {
            EnsureTexture(graph.Width, graph.Height);

            var colors = new Color[_width * _height];
            var visited = result?.VisitedMap ?? new bool[graph.CellCount];
            var pathMap = new bool[graph.CellCount];

            if (result != null)
            {
                foreach (var pos in result.Path)
                {
                    pathMap[graph.ToIndex(pos)] = true;
                }
            }

            for (int y = 0; y < _height; y++)
            {
                for (int x = 0; x < _width; x++)
                {
                    var pos = new GridPos(x, y);
                    int index = graph.ToIndex(pos);

                    Color color = graph.IsBlocked(pos) ? blockedColor : baseColor;
                    if (!graph.IsBlocked(pos) && showCostTint)
                    {
                        float t = Mathf.InverseLerp(1f, Mathf.Max(1, maxCost), graph.Cost(pos));
                        color = Color.Lerp(color, costTintColor, t * 0.5f);
                    }

                    if (visited[index])
                        color = visitedColor;
                    if (pathMap[index])
                        color = pathColor;
                    if (pos == start)
                        color = startColor;
                    if (pos == goal)
                        color = goalColor;

                    // Texture2D는 왼쪽 아래가 (0,0)이므로 Y를 반전시킵니다
                    int textureY = _height - 1 - y;
                    int textureIndex = textureY * _width + x;
                    colors[textureIndex] = color;
                }
            }

            _texture.SetPixels(colors);
            _texture.Apply(false, false);
        }

        private void EnsureTexture(int width, int height)
        {
            if (_texture != null && _width == width && _height == height)
                return;

            _width = width;
            _height = height;

            _texture = new Texture2D(width, height, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp
            };

            var sprite = Sprite.Create(_texture, new Rect(0, 0, width, height), new Vector2(0.5f, 0.5f), 1f);
            _spriteRenderer.sprite = sprite;
            transform.localScale = new Vector3(cellSize, cellSize, 1f);
        }
    }
}
