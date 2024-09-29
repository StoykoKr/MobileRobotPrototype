using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public class JsonMessageClass
    {
        public float direction { get; set; }
        public float movement { get; set; }
        public float leftSensor { get; set; }
        public float midSensor { get; set; }
        public float rightSensor { get; set; }
    }
}
