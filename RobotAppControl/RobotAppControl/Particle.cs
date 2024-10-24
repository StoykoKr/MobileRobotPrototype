using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    internal class Particle
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Theta { get; set; } // Orientation in radians
        public double Weight { get; set; }

        public Particle(double x, double y, double theta)
        {
            X = x;
            Y = y;
            Theta = theta;
            Weight = 1;
        }
        public Particle Clone()
        {
            return new Particle(X, Y, Theta) { Weight = this.Weight };
        }
    }
}
