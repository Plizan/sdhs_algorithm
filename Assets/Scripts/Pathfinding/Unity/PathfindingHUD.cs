using Sdhs.Pathfinding.Core;
using UnityEngine;

namespace Sdhs.Pathfinding.Unity
{
    public sealed class PathfindingHUD : MonoBehaviour
    {
        [SerializeField] private PathfindingDemoController controller;
        [SerializeField] private bool showControls = true;

        private GUIStyle _labelStyle;

        private void Awake()
        {
            if (controller == null)
                controller = FindFirstObjectByType<PathfindingDemoController>();
        }

        private void OnGUI()
        {
            if (controller == null)
                return;

            _labelStyle ??= new GUIStyle(GUI.skin.label)
            {
                fontSize = 14,
                richText = false
            };

            PathResult result = controller.LastResult;
            string status = result == null
                ? "No result"
                : result.Success
                    ? $"✓ Path found: Length={result.PathLength} | Cost={result.PathCost} | Visited={result.VisitedCount}"
                    : $"✗ No path found | Visited={result.VisitedCount}";

            var start = controller.GetStart();
            var goal = controller.GetGoal();

            string text =
                $"Algorithm: {controller.GetAlgorithmLabel()}\n" +
                $"Use case: {controller.GetUseCaseText()}\n" +
                $"Start: ({start.x},{start.y}) Goal: ({goal.x},{goal.y})\n" +
                $"{status}\n" +
                $"Search time: {controller.LastSearchTimeMs:F2}ms";

            if (showControls)
            {
                text += $"\n\nControls: R = regenerate, Space = solve, Tab = switch algorithm, A = toggle animation [{(controller.IsAnimating ? "animating..." : "ready")}]";
            }

            GUILayout.BeginArea(new Rect(10, 10, 520, 160));
            GUILayout.Label(text, _labelStyle);
            GUILayout.EndArea();
        }
    }
}
