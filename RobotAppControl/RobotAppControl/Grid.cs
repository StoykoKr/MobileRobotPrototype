using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public class Grid
    {
        public int Width { get; }
        public int Height { get; }
        public bool[,] Walkable { get; }

        public Grid(int width, int height)
        {
            Width = width;
            Height = height;
            Walkable = new bool[width, height];
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    Walkable[x, y] = true; // Default all cells to walkable
                }
            }
        }

        public void SetWalkable(int x, int y, bool walkable)
        {
            Walkable[x, y] = walkable;
        }
    }
}

