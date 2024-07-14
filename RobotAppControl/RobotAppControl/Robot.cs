using System;
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
        private int _currentX;
        private int _currentY;
        private Form1 _form;

        public Robot(Grid grid, CustomBitmap bitmap, int startX, int startY, Form1 form)
        {
            _grid = grid;
            _bitmap = bitmap;
            _currentX = startX;
            _currentY = startY;
            _form = form;
          
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
            {
                Graphics g = Graphics.FromImage(_bitmap.Bitmap);
                g.DrawLine(new Pen(Brushes.Red),_currentX,_currentY,x,y);
              //  g.DrawLine(new Pen(Brushes.Blue), _currentX, _currentY, x, _currentY);;
             //   g.DrawLine(new Pen(Brushes.Green), x, _currentY, x, y);
                _currentX = x;
               _currentY = y; 

            }

                _form.Invoke(_form.myDelagate);
        }

        public void ExecutePath(List<Node> path)
        {
            foreach (var node in path)
            {
                MoveTo(node.X, node.Y);

            }
        }
    }
}
