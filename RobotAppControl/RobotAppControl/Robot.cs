using System;
using System.CodeDom.Compiler;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public class Robot
    {
        private readonly Grid _grid;
        private readonly CustomBitmap _bitmap;
        public double _currentX;
        public double _currentY;
        private double theta;
        private Form1 _form;
        public double LeftWheelVelocity { get; set; }
        public double RightWheelVelocity { get; set; }
        public double currentLeftWheelVelocity { get; set; }
        public double currentRightWheelVelocity { get; set; }
        public double WheelBase { get; set; }

        public double getThetaVisual()
        {
            double temp = theta - 90;
            if (temp < 0)
                temp = temp + 360;
            else if (temp > 360)
                temp = temp - 360;
           
            return temp;//temp;
        }
        public double getThetaActual()
        {
            //  double temp = theta - 90;
            //   if (temp < 0) temp += 360;
            return theta;//temp;
        }
        public void setThetaVisual(double theta)
        {
            double temp = theta + 270;
            if (temp < 0)
                this.theta = temp + 360;
            else if (temp > 360)
                this.theta = temp - 360;
            else
                this.theta = temp;
        }
        public void setThetaActual(double theta)
        {
            if (theta < 0)
                this.theta = theta + 360;
            else if (theta > 360)
                this.theta = theta - 360;
            else
                this.theta = theta;
        }
        public Robot(Grid grid, CustomBitmap bitmap, int startX, int startY, Form1 form, double Wheelbase)
        {
            _grid = grid;
            _bitmap = bitmap;
            _currentX = startX;
            _currentY = startY;
            _form = form;
            LeftWheelVelocity = 0;
            RightWheelVelocity = 0;
            WheelBase = Wheelbase;
        }

        public void MoveTo(int x, int y)  // This for now just marks the path we are planning to take.
        {
            /*while (_currentX != x || _currentY != y)
            {
                if (_currentX < x) _currentX++;
                else if (_currentX > x) _currentX--;

                if (_currentY < y) _currentY++;
                else if (_currentY > y) _currentY--;

                _bitmap.SetPixel(_currentX, _currentY, Color.Red);
                
            }
            */

            Graphics g = Graphics.FromImage(_bitmap.Bitmap);
            g.DrawLine(new Pen(Brushes.Red), (int)_currentX, (int)_currentY, x, y);
            //  g.DrawLine(new Pen(Brushes.Blue), _currentX, _currentY, x, _currentY);;
            //   g.DrawLine(new Pen(Brushes.Green), x, _currentY, x, y);
            _currentX = x;
            _currentY = y;



            _form.Invoke(_form.myDelagate);
        }

        public void ExecutePath(List<Node> path)
        {
            var startX = _currentX;
            var startY = _currentY;
            foreach (var node in path)
            {
                MoveTo(node.X, node.Y);

            }
            _currentX = startX;
            _currentY = startY;
        }
    }
}
