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
}
   