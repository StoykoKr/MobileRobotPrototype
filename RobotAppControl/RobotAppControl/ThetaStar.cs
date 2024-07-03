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

        public List<Node> FindPath(Node start, Node goal)
        {
            var openSet = new SortedSet<Node>(Comparer<Node>.Create((a, b) => a.F.CompareTo(b.F) == 0 ? a.H.CompareTo(b.H) : a.F.CompareTo(b.F)));
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
                    if (closedSet.Contains(neighbor) || !IsPassable(neighbor))
                        continue;

                    float tentativeG = current.Parent != null && LineOfSight(current.Parent, neighbor) ? current.Parent.G + Heuristic(current.Parent, neighbor) : current.G + Heuristic(current, neighbor);

                    if (tentativeG < neighbor.G)
                    {
                        neighbor.G = tentativeG;
                        neighbor.H = Heuristic(neighbor, goal);
                        neighbor.Parent = current.Parent != null && LineOfSight(current.Parent, neighbor) ? current.Parent : current;

                        if (!openSet.Contains(neighbor))
                            openSet.Add(neighbor);
                    }
                }
            }

            return new List<Node>(); // Return empty list if no path found
        }
        public List<Node> NewFindPath(Node start, Node goal)
        {
            var openSet = new SortedList<float, List<Node>>();
            var openSetDict = new Dictionary<(int, int), Node>();
            var closedSet = new HashSet<Node>();

            start.G = 0;
            start.H = Heuristic(start, goal);
            AddToOpenSet(openSet, start);

            while (openSet.Count > 0)
            {
                var current = ExtractMinFromOpenSet(openSet);

                if (current.X == goal.X && current.Y == goal.Y)
                    return ReconstructPath(current);

                closedSet.Add(current);

                foreach (var neighbor in GetNeighbors(current))
                {
                    if (closedSet.Contains(neighbor) || !IsPassable(neighbor))
                        continue;

                    float tentativeG = current.Parent != null && LineOfSight(current.Parent, neighbor) ? current.Parent.G + Heuristic(current.Parent, neighbor) : current.G + Heuristic(current, neighbor);

                    if (!openSetDict.ContainsKey((neighbor.X, neighbor.Y)) || tentativeG < neighbor.G)
                    {
                        neighbor.G = tentativeG;
                        neighbor.H = Heuristic(neighbor, goal);
                        neighbor.Parent = current.Parent != null && LineOfSight(current.Parent, neighbor) ? current.Parent : current;

                        if (!openSetDict.ContainsKey((neighbor.X, neighbor.Y)))
                        {
                            AddToOpenSet(openSet, neighbor);
                            openSetDict[(neighbor.X, neighbor.Y)] = neighbor;
                        }
                    }
                }
            }

            return new List<Node>(); // Return empty list if no path found
        }

        private void AddToOpenSet(SortedList<float, List<Node>> openSet, Node node)
        {
            if (!openSet.ContainsKey(node.F))
                openSet[node.F] = new List<Node>();

            openSet[node.F].Add(node);
        }

        private Node ExtractMinFromOpenSet(SortedList<float, List<Node>> openSet)
        {
            var firstKey = openSet.Keys[0];
            var node = openSet[firstKey][0];
            openSet[firstKey].RemoveAt(0);
            if (openSet[firstKey].Count == 0)
                openSet.Remove(firstKey);
            return node;
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

            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0)
                        continue;

                    int newX = node.X + dx;
                    int newY = node.Y + dy;

                    if (newX >= 0 && newX < _grid.Width && newY >= 0 && newY < _grid.Height && _grid.Walkable[newX,newY])
                    {
                        neighbors.Add(new Node(newX, newY));
                    }
                }
            }

            return neighbors;
        }

        private bool IsPassable(Node node)
        {
            return _grid.Walkable[node.X,node.Y];
        }

        private bool LineOfSight(Node from, Node to)
        {
            int x0 = from.X, y0 = from.Y, x1 = to.X, y1 = to.Y;
            int dx = Math.Abs(x1 - x0), dy = Math.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;

            while (true)
            {
                if (!_grid.Walkable[x0, y0])
                    return false;
                if (x0 == x1 && y0 == y1)
                    return true;

                int e2 = err * 2;
                if (e2 > -dy)
                {
                    if (x0 == x1)
                    {
                  //      return true;
                    }
                    err -= dy;
                    x0 += sx;
                }
                if (e2 < dx)
                {
                    if (y0 == y1)
                    {
                    //    return true;
                    }
                    err += dx;
                    y0 += sy;
                }
            }
            return true;
        }

      /*  private float Heuristic(Node a, Node b)
        {
            return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
        }*/
         private float Heuristic(Node a, Node b)
        {
            float one = (float)Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
            float two = Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
            // return (float)Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));

            /*  if (one < two)
              {
                  return two ;
              }
              else return one;*/

            //  return (one + two)/2;
           // return one;
            //return two;
            return (float)Math.Round(one * 0.4f + two * 0.6f);
           
        }
    }

}
