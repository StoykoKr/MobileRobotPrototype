using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public class Node : IComparable<Node>
    {
        public int X { get; }
        public int Y { get; }
        public float G { get; set; }
        public float H { get; set; }
        public Node? Parent { get; set; }

        public Node(int x, int y)
        {
            X = x;
            Y = y;
            G = float.MaxValue;
            H = 0;
            Parent = null;
        }

        public float F => G + H;

        public override bool Equals(object obj)
        {
            return obj is Node node && X == node.X && Y == node.Y;
        }

        public override int GetHashCode()
        {
            return X.GetHashCode() ^ Y.GetHashCode();
        }

        public int CompareTo(Node other)
        {
            return F.CompareTo(other.F);
        }
    }

    public class AStar
    {
        private readonly Grid _grid;

        public AStar(Grid grid)
        {
            _grid = grid;
        }

        private static float Heuristic(Node a, Node b)
        {
            return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
        }

        private IEnumerable<Node> GetNeighbors(Node node)
        {
            var neighbors = new List<Node>();

            var directions = new List<(int, int)>
        {
            (0, 1), (1, 0), (0, -1), (-1, 0)
        };

            foreach (var (dx, dy) in directions)
            {
                int newX = node.X + dx;
                int newY = node.Y + dy;

                if (newX >= 0 && newX < _grid.Width && newY >= 0 && newY < _grid.Height && _grid.Walkable[newX, newY])
                {
                    neighbors.Add(new Node(newX, newY));
                }
            }

            return neighbors;
        }

        public List<Node> FindPath(Node start, Node goal)
        {
            var openSet = new SortedSet<Node>(Comparer<Node>.Create((a, b) => a.F.CompareTo(b.F)));
            var closedSet = new HashSet<Node>();

            start.G = 0;
            start.H = Heuristic(start, goal);

            openSet.Add(start);

            while (openSet.Count > 0)
            {
                var current = openSet.Min;
                openSet.Remove(current);

                if (current.Equals(goal))
                {
                    var path = new List<Node>();
                    while (current != null)
                    {
                        path.Add(current);
                        current = current.Parent;
                    }
                    path.Reverse();
                    return path;
                }

                closedSet.Add(current);

                foreach (var neighbor in GetNeighbors(current))
                {
                    if (closedSet.Contains(neighbor))
                    {
                        continue;
                    }

                    float tentativeG = current.G + 1;

                    if (!openSet.Contains(neighbor) || tentativeG < neighbor.G)
                    {
                        neighbor.Parent = current;
                        neighbor.G = tentativeG;
                        neighbor.H = Heuristic(neighbor, goal);

                        if (!openSet.Contains(neighbor))
                        {
                            openSet.Add(neighbor);
                        }
                    }
                }
            }

            return null; // No path found
        }
    }
}
