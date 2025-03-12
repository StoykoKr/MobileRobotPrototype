using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    internal class GridCell
    {
        public int IsPossible { get; set; } = 0; 
        public int IsProvenEmpty { get; set; } = 0;

        public void IncreaseIsProvenEmpty()
        {
            IsProvenEmpty++;
            IsProvenEmpty++;
            if (IsProvenEmpty > 10) {
                IsProvenEmpty = 10;
            }
        }
        public void ResetBelief()
        {
            IsProvenEmpty = 0;
        }
    }
}
