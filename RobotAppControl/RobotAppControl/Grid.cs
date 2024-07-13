using System;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
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
        private Tuple<bool,float>[,] GridInfo; 
        
        public Grid(int width, int height)
        {
            Width = width;
            Height = height;
            Walkable = new bool[width, height];
            GridInfo = new Tuple<bool, float>[width,height];
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    Walkable[x, y] = true;
                    GridInfo[x,y] = Tuple.Create<bool,float>(true,1);
                    // Default all cells to walkable
                }
            }
        }
        public float GetCost(int x, int y)
        {
            if ((x >= 0 && x < Width)&&(y >= 0 && y < Height))
            {
                return GridInfo[x,y].Item2;
            }
            else
                return 0;
        }

        public void SetCost(int x, int y, float newCost)
        {
            if ((x >= 0 && x < Width) && (y >= 0 && y < Height))
            {
                var oldTuple = GridInfo[x, y];
                GridInfo[x, y] = Tuple.Create(oldTuple.Item1,newCost);
            }
        }
        public bool? IsWalkable(int x,int y)
        {

            if ((x >= 0 && x < Width) && (y >= 0 && y < Height))
            {
                return GridInfo[x, y].Item1;
            }
            else
            return null;
        }
        public void SetWalkable(int x, int y, bool walkable)
        {
            Walkable[x, y] = walkable;
            var oldTuple = GridInfo[x, y];
            GridInfo[x, y] = Tuple.Create(walkable, oldTuple.Item2);
        }
    }
}

