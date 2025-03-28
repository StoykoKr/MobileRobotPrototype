using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public static class GlobalConstants
    {
        public static readonly double MidDegrees = Math.Atan2(14, 30) * (180.0 / Math.PI);  
        public static readonly double LeftDegrees = Math.Atan2(-22, 8) * (180.0 / Math.PI);  
        public static readonly double RightDegrees = Math.Atan2(25, 7) * (180.0 / Math.PI);

        public static readonly double DegreeOffsetMid = 0;
        public static readonly double DegreeOffsetLeft = -90;
        public static readonly double DegreeOffsetRight = 90;


        public static readonly (double, double) MidSensorOffsets = (14,-30);
        public static readonly (double, double) LeftSensorOffsets = (-22, -8);
        public static readonly (double, double) RightSensorOffsets = (25, -7);

        public static readonly double SensorDispersion = 30; //assumed... how much the sensor wave thing expands


    }
}
