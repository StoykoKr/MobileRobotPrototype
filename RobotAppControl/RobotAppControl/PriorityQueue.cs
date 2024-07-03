using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public class PriorityQueue<T>
    {
        private List<(T item, float priority)> elements = new List<(T item, float priority)>();

        public int Count => elements.Count;

        public void Enqueue(T item, float priority)
        {
            elements.Add((item, priority));
            elements.Sort((x, y) => x.priority.CompareTo(y.priority));
        }

        public T Dequeue()
        {
            var bestItem = elements[0];
            elements.RemoveAt(0);
            return bestItem.item;
        }

        public bool Contains(T item)
        {
            return elements.Exists(x => EqualityComparer<T>.Default.Equals(x.item, item));
        }

        public void UpdatePriority(T item, float priority)
        {
            var index = elements.FindIndex(x => EqualityComparer<T>.Default.Equals(x.item, item));
            if (index != -1)
            {
                elements[index] = (item, priority);
                elements.Sort((x, y) => x.priority.CompareTo(y.priority));
            }
        }
    }
}
