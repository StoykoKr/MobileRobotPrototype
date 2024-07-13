using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public class ThetaStar
    {
        private readonly Grid _grid;

        public ThetaStar(Grid grid)
        {
            _grid = grid;
        }

        public List<Node> FindPath(Node start, Node goal) /// This method is supposed to find a good path between two given Nodes and more or less it does indeed work.
        {
            var openSet = new SortedSet<Node>(Comparer<Node>.Create((a, b) =>
            {
                int fComparison = a.F.CompareTo(b.F);
                if (fComparison == 0)
                {
                    int hComparison = a.H.CompareTo(b.H);
                    if (hComparison == 0)
                    {
                        return a.GetHashCode().CompareTo(b.GetHashCode());
                    }
                    return hComparison;
                }
                return fComparison;
            }));

            var closedSet = new HashSet<Node>();

            start.G = 0;
            start.H = Heuristic(start, goal);
            openSet.Add(start);

            while (openSet.Count > 0)
            {
                var current = openSet.Min;
                openSet.Remove(current);

                if (current.X == goal.X && current.Y == goal.Y)
                    return ReconstructPath(current);

                closedSet.Add(current);

                foreach (var neighbor in GetNeighbors(current))
                {
                    if (closedSet.Contains(neighbor) || !_grid.Walkable[neighbor.X, neighbor.Y])
                        continue;
                    // About the tentativeG. During the calculation using the _grid.GetCost leads to faster results but I fear we are not using the real distance.
                    float tentativeG = current.Parent != null && LineOfSight(current.Parent, neighbor) ?
                                       current.Parent.G + (_grid.GetCost(current.Parent.X, current.Parent.Y) + _grid.GetCost(neighbor.X, neighbor.Y) + _grid.GetCost(current.X, current.Y)) :// + _grid.GetCost(current.Parent.X, current.Parent.Y) :         //Heuristic(current.Parent, neighbor) :
                                       current.G + _grid.GetCost(neighbor.X, neighbor.Y) + _grid.GetCost(current.X, current.Y);//_grid.GetCost(current.X, current.Y) + _grid.GetCost(neighbor.X, neighbor.Y);// Heuristic(current, neighbor);

                    if (!openSet.Contains(neighbor) || tentativeG < neighbor.G)
                    {
                        neighbor.G = tentativeG;
                        neighbor.H = Heuristic(neighbor, goal);
                        neighbor.Parent = current.Parent != null && LineOfSight(current.Parent, neighbor) ? current.Parent : current;

                        if (openSet.Contains(neighbor))
                        {
                            openSet.Remove(neighbor); // Remove to reinsert with updated G value
                        }
                        openSet.Add(neighbor);
                    }
                }
            }

            return new List<Node>(); // Return empty list if no path found
        }


        private List<Node> ReconstructPath(Node node)
        {
            var path = new List<Node>();
            while (node != null)
            {
                path.Add(node);
                node = node.Parent;
            }
            path.Reverse();
            return path;
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

        private bool LineOfSight(Node from, Node to)
        {
            int x0 = from.X, y0 = from.Y, x1 = to.X, y1 = to.Y;
            int dx = Math.Abs(x1 - x0), dy = Math.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;

            while (true)
            {
                try
                {
                    if (!_grid.Walkable[x0, y0])
                        return false;
                    if (x0 == x1 && y0 == y1)
                        return true;

                }
                catch (Exception)
                {

                    throw;
                }

                if(dx > 0)
                {
                    x0 += sx;
                    dx--;
                }
                if(dy > 0)
                {
                    y0 += sy;
                    dy--;
                }
            }
        }

        private float Heuristic(Node a, Node b)
        {
            float euclidean = (float)Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
            // float manhattan = Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
            // return (float)Math.Round(euclidean * 0.4f + manhattan * 0.6f);
            return euclidean + _grid.GetCost(a.X,a.Y);
        }
    }


}
