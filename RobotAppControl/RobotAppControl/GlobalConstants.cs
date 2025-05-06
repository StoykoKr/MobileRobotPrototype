using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public static class GlobalConstants
    {
        public static readonly double MidDegrees = Math.Atan2(-4, 33) * (180.0 / Math.PI);  
        public static readonly double LeftDegrees = Math.Atan2(23, 26) * (180.0 / Math.PI);  
        public static readonly double RightDegrees = Math.Atan2(-27, 25) * (180.0 / Math.PI);

        public static readonly double DegreeOffsetMid = 0;
        public static readonly double DegreeOffsetLeft = -60;
        public static readonly double DegreeOffsetRight = 52;


        public static readonly (double, double) MidSensorOffsets = (-4,33);
        public static readonly (double, double) LeftSensorOffsets = (23, 26);
        public static readonly (double, double) RightSensorOffsets = (-27, 25);

        public static readonly double SensorDispersion = 15; 


    }
}
