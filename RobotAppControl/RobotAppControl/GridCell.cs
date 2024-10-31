using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    internal class GridCell
    {
        public int StartingX { get; set; }
        public int StartingY { get; set; }
        public int EndingX { get; set; }
        public int EndingY { get; set; }
        public double GetCost(Grid totalGrid)
        {
            int counter = 0;
            double sum = 0;
            for (int i = StartingX; i <= EndingX; i++)
            {
                for (int j = StartingY; j <= EndingY; j++)
                {
                    sum += totalGrid.GetCost(i, j);
                    counter++;
                }
            }


            return sum / counter;
        }
    }
}
