using System;
using System.Collections.Generic;

namespace Sdhs.Pathfinding.Core
{
    public sealed class MinHeap<T>
    {
        private readonly List<(T Item, int Priority)> _items = new();

        public int Count => _items.Count;

        public void Enqueue(T item, int priority)
        {
            _items.Add((item, priority));
            SiftUp(_items.Count - 1);
        }

        public (T Item, int Priority) Dequeue()
        {
            if (_items.Count == 0)
                throw new InvalidOperationException("Heap is empty.");

            var root = _items[0];
            var last = _items[^1];
            _items.RemoveAt(_items.Count - 1);

            if (_items.Count > 0)
            {
                _items[0] = last;
                SiftDown(0);
            }

            return root;
        }

        private void SiftUp(int index)
        {
            while (index > 0)
            {
                int parent = (index - 1) / 2;
                if (_items[index].Priority >= _items[parent].Priority)
                    break;

                (_items[index], _items[parent]) = (_items[parent], _items[index]);
                index = parent;
            }
        }

        private void SiftDown(int index)
        {
            int lastIndex = _items.Count - 1;
            while (true)
            {
                int left = index * 2 + 1;
                int right = index * 2 + 2;
                if (left > lastIndex)
                    break;

                int smallest = left;
                if (right <= lastIndex && _items[right].Priority < _items[left].Priority)
                    smallest = right;

                if (_items[index].Priority <= _items[smallest].Priority)
                    break;

                (_items[index], _items[smallest]) = (_items[smallest], _items[index]);
                index = smallest;
            }
        }
    }
}
