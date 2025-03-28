using System;
using System.Collections.Generic;
using System.Drawing.Imaging;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using System.Collections;
using System.Security.Policy;
using System.Windows.Forms;

namespace RobotAppControl
{
    internal class Interpreter
    {
        private ConcurrentQueue<(string key, object value)> stringsToBeInterpreted;
        private CustomBitmap bitmap;
        private bool stopInterpreting = false;
        private double currentX;
        private double currentY;
        private double currentRotation;
        private int counter = 0;
        private Form1 formControl;
        public double movedTotalRecieved = 0;
        private List<string> magDataList;
        private double currentDegrees = 0; // This represents the offset in degrees of the sensors compared to the middle point. It is replaced by the 3 values below. ONLY use with offset values not incoming sensor data
        //private double MidDegrees = Math.Atan2(14, 30) * (180.0 / Math.PI);  //mid  change Math.Atan2(x,y) with new values if sensors are moved
        //private double LeftDegrees = Math.Atan2(-22, 8) * (180.0 / Math.PI);  //left
        //private double RightDegrees = Math.Atan2(25, 7) * (180.0 / Math.PI);  //right
        //private int degreeOffsetMid = 0;
        //private int degreeOffsetLeft = -90;
        //private int degreeOffsetRight = 90;
        private KalmanFilter kalmanLeft;
        private KalmanFilter kalmanRight;
        private KalmanFilter kalmanMid;
        private double midValue = 0;
        private double leftValue = 0;
        private double rightValue = 0;
        private double midValuePrevious = 0;
        private double leftValuePrevious = 0;
        private double rightValuePrevious = 0;
        public List<string> inputLog = new List<string>();
        private GridCell[,] gridForMapping;
        private double wheelBase = 0;
        public Interpreter(ref ConcurrentQueue<(string key, object value)> strings, ref CustomBitmap actualMap, ref double curX, ref double curY, ref double curRotation, Form1 formReference, double wheelBase)
        {
            magDataList = new List<string>();
            stringsToBeInterpreted = strings;
            bitmap = actualMap;
            gridForMapping = new GridCell[bitmap.Width, bitmap.Height];
            for (int i = 0; i < bitmap.Width; i++)
            {
                for (int j = 0; j < bitmap.Height; j++)
                {
                    gridForMapping[i, j] = new GridCell();
                }
            }
            this.wheelBase = wheelBase;
            currentX = curX;
            currentY = curY;
            currentRotation = curRotation;
            formControl = formReference;
            kalmanRight = new KalmanFilter(0.7f, 15, 7.5f, 4, 3, 10);
            kalmanLeft = new KalmanFilter(0.7f, 15, 7.5f, 4, 3, 10);
            kalmanMid = new KalmanFilter(0.7f, 15, 7.5f, 4, 3, 10);
        }
        private void UpdateImg()  // Invokes the UI thread to show the results of the newly painted map.
        {
            formControl.Invoke(formControl.myDelagate);
        }

        private bool IsInBounds(int x, int y) => x >= 0 && x < bitmap.Width && y >= 0 && y < bitmap.Height;

        private List<(int, int)> pointsThatNeedVisualChange = new List<(int, int)>();
        private void MarkPossibleArea(int Ax, int Ay, double distance, int angleMin, int angleMax)
        {
            HashSet<(int, int)> arcBoundaryPoints = new();
            HashSet<(int, int)> interiorPoints = new();
            var tempAngle = 0;
            for (int i = 0; i <= 30; i++)
            {
                tempAngle = angleMin + i;
                if (tempAngle >= 360)
                {
                    tempAngle -= 360;
                }
                double radians = tempAngle * Math.PI / 180.0;
                int targetX = (int)Math.Round(Ax + Math.Abs(distance) * Math.Cos(radians));
                int targetY = (int)Math.Round(Ay+ Math.Abs(distance) * Math.Sin(radians));

                if (IsInBounds(targetX, targetY))
                    arcBoundaryPoints.Add((targetX, targetY));

                foreach (var point in BresenhamLine(Ax, Ay, targetX, targetY))
                {
                    if (point != (targetX, targetY) && IsInBounds(point.Item1, point.Item2))
                        interiorPoints.Add(point);
                }
            }

            Dictionary<Tuple<int, int>, int> AffectedCells = new Dictionary<Tuple<int, int>, int>();
            foreach (var (x, y) in arcBoundaryPoints)
            {
              
                HandleAdjacentPixels(x, y, 2, AffectedCells);

            }

            foreach (var item in AffectedCells.Keys)
            {
                if (gridForMapping[item.Item1, item.Item2].IsPossible <= 15)
                {
                    gridForMapping[item.Item1, item.Item2].IsPossible += 5;
                }

               
                if (gridForMapping[item.Item1, item.Item2].IsPossible >= 8 + gridForMapping[item.Item1, item.Item2].IsProvenEmpty)
                {
                    pointsThatNeedVisualChange.Add((item.Item1, item.Item2));
                    gridForMapping[item.Item1, item.Item2].ResetBelief();

                    if(distance < 20)
                    {
                        gridForMapping[item.Item1, item.Item2].WasSpottedFromTooClose = true;
                    }
                }
            }

            foreach (var (x, y) in interiorPoints)
            {
               
                if (gridForMapping[x, y].IsPossible > 0)
                    gridForMapping[x, y].IsPossible -=4;

                if (gridForMapping[x, y].IsPossible < 8 && gridForMapping[x, y].IsPossible >=3)
                {
                    pointsThatNeedVisualChange.Add((x, y));
                }
                if (gridForMapping[x, y].IsPossible <= 2)
                {
                    gridForMapping[x, y].IncreaseIsProvenEmpty();
                }

            }
            inputLog.Add($"Arc logic over. Added {pointsThatNeedVisualChange.Count} points for change");
        }
        private void HandleAdjacentPixels(int iCentr, int jCentr, int spread, Dictionary<Tuple<int, int>, int> affectedCells)
        {

            for (int i = iCentr - spread; i <= iCentr + spread; i++)
            {
                for (int j = jCentr - spread; j <= jCentr + spread; j++)
                {
                    if (IsInBounds(i, j))
                    {
                        if (affectedCells.ContainsKey(Tuple.Create(i, j)))
                            {
                                affectedCells[Tuple.Create(i, j)] += 1;
                            }
                            else
                            {
                                affectedCells.Add(Tuple.Create(i, j), 1);
                            }
                        
                    }
                    
                }
            }
        }
        public static List<(int, int)> BresenhamLine(int x0, int y0, int x1, int y1)  // should be an algorithm to cheaply draw a line from one point to another in a 2d grid. If working should implement in other parts of the code too.
        {
            List<(int, int)> points = new();
            int dx = Math.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int dy = -Math.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy, e2;

            while (true)
            {
                points.Add((x0, y0));
                if (x0 == x1 && y0 == y1) break;
                e2 = 2 * err;
                if (e2 >= dy) { err += dy; x0 += sx; }
                if (e2 <= dx) { err += dx; y0 += sy; }
            }

            return points;
        }

        public List<(double, double, double, double)> sensorRotationLog = new List<(double, double, double, double)>();
        public List<(double, double, double, double)> sensormovelog = new List<(double, double, double, double)>();
        double prevMagnetometerReading = -1;

        private void ClearNearbyPoints(int x, int y, double rotation, int width, int heighth)
        {
            double theta = rotation * Math.PI/180; // Orientation in radians (30 degrees)
            double margin = 1;         // Expansion margin

            var expandedCorners = GetExpandedRectangle(x, y, width, heighth, theta, margin);

            FillRotatedRectangle(bitmap, expandedCorners, Color.Yellow);

        }
        private void MakeMap(JsonMessageClass whatWeKnow) // Paints the map with the information we got. 
        {
            if (bitmap != null)
            {
                try
                {
                    (double, double, double, double) log = new(0, 0, 0, 0);
                    (double, double, double, double) log1 = new(0, 0, 0, 0);
                    int tempY = bitmap.Height / 2;
                    int tempX = bitmap.Width / 2;
                    currentRotation = whatWeKnow.direction;    // ADJUST HERE BY 90 IN A DIRECTION NOT SURE WHICH // Added change idk what now

                    if(prevMagnetometerReading < 0)
                    {
                        prevMagnetometerReading = currentRotation;
                    }

                    double dL = whatWeKnow.rightMovement * 0.1;  // Right wheel movement
                    double dR = whatWeKnow.leftMovement * 0.1;   // Left wheel movement

                    double d = (dL + dR) / 2.0;  // Linear displacement

                    // Get the current rotation from the magnetometer (already in degrees)
                    double newTheta = currentRotation * Math.PI / 180;  // Convert to radians
                    double deltaTheta = newTheta - (prevMagnetometerReading * Math.PI / 180);  // Compute actual heading change
                    prevMagnetometerReading = currentRotation;  // Store current rotation for next iteration

                    if (Math.Abs(deltaTheta) < 1e-6)  // If rotation is negligible, move straight
                    {
                        currentX += d * Math.Cos(newTheta);
                        currentY += d * Math.Sin(newTheta);
                    }
                    else
                    {
                        currentX -= (d / deltaTheta) * (Math.Sin(newTheta) - Math.Sin(newTheta - deltaTheta));
                        currentY += (d / deltaTheta) * (Math.Cos(newTheta - deltaTheta) - Math.Cos(newTheta));
                    }





                    int x = tempX - (int)Math.Round(currentX);
                        int y = tempY - (int)Math.Round(currentY);
                        //int offsedX = 0;
                        //int offsedY = 0;
                        Color newColor = Color.Green;
                        //bitmap.SetPixel(
                        //     x,
                        //     y, newColor);
                    gridForMapping[x, y].RobotCenterPassedHere = true;


                        int centralPixelX;
                        int centralPixelY;
                        //if (!formControl.currentlyMappingSimulation)
                        //{

                        //    offsedY = -30;
                        //    offsedX = 14;
                        //    //degreeOffsetLeft = 90;
                        //    //degreeOffsetRight = -90;
                        //}

                        Dictionary<Tuple<int, int>, int> AffectedCells = new Dictionary<Tuple<int, int>, int>();

                        // currentRotation += 180;

                       // newColor = Color.White;
                          newColor = Color.Red;
                        currentDegrees = GlobalConstants.MidDegrees;
                        double tempValueToCalcNewRotation = 0;
                       
                            tempValueToCalcNewRotation = 360 - currentRotation;  // ADD SOMETHING LIKE THIS TO MCL

                        currentRotation = tempValueToCalcNewRotation;


                         ClearNearbyPoints(x,y,currentRotation,20,30);


                        if (whatWeKnow.midSensor < 330)
                        {

                            midValue = kalmanMid.Output(whatWeKnow.midSensor);
                            if (Math.Abs(midValue - midValuePrevious) < 5)
                            {
                                var tempMin = GlobalConstants.DegreeOffsetMid + currentRotation - 15;
                                var tempMax = GlobalConstants.DegreeOffsetMid + currentRotation + 15;
                                while (tempMin < 0)
                                {
                                    tempMin += 360;
                                }
                                while (tempMin > 360)
                                {
                                    tempMin -= 360;
                                }
                                while (tempMax < 0)
                                {
                                    tempMax += 360;
                                }
                                while (tempMax > 360)
                                {
                                    tempMax -= 360;
                                }


                            //inputLog.Add($"Mid Arc with:{x}| {y} | {midValue} | {(int)tempMin} | {(int)tempMax}");
                            MarkPossibleArea(x + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.MidSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.MidSensorOffsets.Item2, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                                             y + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.MidSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.MidSensorOffsets.Item2, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)),
                                midValue, (int)tempMin, (int)tempMax);
                            log.Item2 = currentDegrees + currentRotation;
                            //      bitmap.SetPixel(
                            //x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                            //         y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)), Color.Orange);

                            //centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(midValue * Math.Cos((degreeOffsetMid + currentRotation) * Math.PI / 180));
                            //centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(midValue * Math.Sin((degreeOffsetMid + currentRotation) * Math.PI / 180));

                            //          bitmap.SetPixel(
                            //             centralPixelX,
                            //             centralPixelY, newColor);


                            //HandleAdjacentPixels(centralPixelX, centralPixelY, 1, AffectedCells);


                        }
                            inputLog.Add($"mid:{Math.Round(whatWeKnow.midSensor, 2)} {Math.Round(midValue, 2)} {Math.Round(midValuePrevious, 2)} {GlobalConstants.DegreeOffsetMid} {currentRotation}");
                            midValuePrevious = midValue;
                        }

                        if (whatWeKnow.leftSensor < 330)
                        {
                            if (!formControl.currentlyMappingSimulation)
                            {

                                //offsedY = -8;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                                //offsedX = -22;
                            }
                               newColor = Color.Yellow;

                            leftValue = kalmanLeft.Output(whatWeKnow.leftSensor);
                            if (Math.Abs(leftValue - leftValuePrevious) < 5)
                            {
                                currentDegrees = GlobalConstants.LeftDegrees;


                                var tempMin = GlobalConstants.DegreeOffsetLeft + currentRotation - 15;
                                var tempMax = GlobalConstants.DegreeOffsetLeft + currentRotation + 15;
                                while (tempMin < 0)
                                {
                                    tempMin += 360;
                                }
                                while (tempMin > 360)
                                {
                                    tempMin -= 360;
                                }
                                while (tempMax < 0)
                                {
                                    tempMax += 360;
                                }
                                while (tempMax > 360)
                                {
                                    tempMax -= 360;
                                }

                            //inputLog.Add($"Left Arc with:{x}| {y} | {leftValue} | {(int)tempMin} | {(int)tempMax}");
                            MarkPossibleArea(x + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.LeftSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.LeftSensorOffsets.Item2, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                                y + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.LeftSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.LeftSensorOffsets.Item2, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)),
                                leftValue, (int)tempMin, (int)tempMax);
                            log.Item1 = currentDegrees + currentRotation;

                            //      bitmap.SetPixel(
                            //x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                            //         y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)), Color.Cyan);

                            //      centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(leftValue * Math.Cos((degreeOffsetLeft + currentRotation) * Math.PI / 180));
                            //          centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(leftValue * Math.Sin((degreeOffsetLeft + currentRotation) * Math.PI / 180));
                            //          bitmap.SetPixel(
                            //       centralPixelX,
                            //       centralPixelY, newColor);


                            //   HandleAdjacentPixels(centralPixelX, centralPixelY, 1, AffectedCells);



                        }
                            inputLog.Add($"left:{Math.Round(whatWeKnow.leftSensor, 2)} {Math.Round(leftValue, 2)} {Math.Round(leftValuePrevious, 2)} {GlobalConstants.DegreeOffsetLeft} {currentRotation}");
                            leftValuePrevious = leftValue;
                        }

                        if (whatWeKnow.rightSensor < 330)
                        {
                            if (!formControl.currentlyMappingSimulation)
                            {
                                //offsedY = -7;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                                //offsedX = 25;
                            }
                                newColor = Color.Blue;

                            rightValue = kalmanRight.Output(whatWeKnow.rightSensor);
                            if (Math.Abs(rightValue - rightValuePrevious) < 5)
                            {
                                currentDegrees = GlobalConstants.RightDegrees;


                                var tempMin = GlobalConstants.DegreeOffsetRight + currentRotation - 15;
                                var tempMax = GlobalConstants.DegreeOffsetRight + currentRotation + 15;
                                while (tempMin < 0)
                                {
                                    tempMin += 360;
                                }
                                while (tempMin > 360)
                                {
                                    tempMin -= 360;
                                }
                                while (tempMax < 0)
                                {
                                    tempMax += 360;
                                }
                                while (tempMax > 360)
                                {
                                    tempMax -= 360;
                                }
                                inputLog.Add($"Right Arc with:{x}| {y} | {rightValue} | {(int)tempMin} | {(int)tempMax}");

                            MarkPossibleArea(x + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.RightSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.RightSensorOffsets.Item2, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                                y + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.RightSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.RightSensorOffsets.Item2, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)),
                                rightValue, (int)tempMin, (int)tempMax);

                            //log.Item3 = currentDegrees + currentRotation;
                       //     bitmap.SetPixel(
                       //x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                       //         y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)), Color.Gold);

                       //     centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(rightValue * Math.Cos((degreeOffsetRight + currentRotation) * Math.PI / 180));
                       //         centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(rightValue * Math.Sin((degreeOffsetRight + currentRotation) * Math.PI / 180));



                       //         bitmap.SetPixel(
                       //      centralPixelX,
                       //      centralPixelY, newColor);
                                //   HandleAdjacentPixels(centralPixelX, centralPixelY, 1, AffectedCells);
                            }
                            inputLog.Add($"right:{Math.Round(whatWeKnow.rightSensor, 2)} {Math.Round(rightValue, 2)} {Math.Round(rightValuePrevious, 2)} {GlobalConstants.DegreeOffsetRight} {currentRotation}");
                            rightValuePrevious = rightValue;
                        }

                        sensorRotationLog.Add(log);
                        foreach (var item in AffectedCells.Keys)
                        {
                            bitmap.SetPixel(
                            item.Item1,
                            item.Item2, newColor);
                        }

                        UpdateMapWithTheNewPoints();

                    
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                    throw;
                }
            }
        }
        private void UpdateMapWithTheNewPoints()
        {
            foreach (var item in pointsThatNeedVisualChange)
            {
                 if ((gridForMapping[item.Item1, item.Item2].IsPossible >= 8 + gridForMapping[item.Item1, item.Item2].IsProvenEmpty || gridForMapping[item.Item1, item.Item2].WasSpottedFromTooClose) && !gridForMapping[item.Item1, item.Item2].WasWithinBoundingBox)
                {
                    bitmap.SetPixel(
                        item.Item1,
                        item.Item2, Color.Red);
                }
                else
                {
                    if(gridForMapping[item.Item1, item.Item2].RobotCenterPassedHere)
                    {
                        bitmap.SetPixel(
                        item.Item1,
                        item.Item2, Color.Green);
                    }
                    else
                    {
                        bitmap.SetPixel(
                         item.Item1,
                         item.Item2, Color.Black);
                    }
                }
            }
            pointsThatNeedVisualChange.Clear();
        }
        public void StopInterpreting()
        {
            stopInterpreting = true;
        }

        public async void UpdateMCL(JsonMessageClass whatWeKnow)
        {
           
            bool midIsRelevant = false;
            bool leftIsRelevant = false;
            bool rightIsRelevant = false;

            double dR = whatWeKnow.rightMovement * 0.1;  // Right wheel movement
            double dL = whatWeKnow.leftMovement * 0.1;   // Left wheel movement

            double d = (dL + dR) / 2.0;  // Linear displacement
            currentRotation = whatWeKnow.direction;

            double tempValueToCalcNewRotation = 0;

            tempValueToCalcNewRotation = 360 - currentRotation;  // ADD SOMETHING LIKE THIS TO MCL

            currentRotation = tempValueToCalcNewRotation;

            midValue = kalmanMid.Output(whatWeKnow.midSensor); // This should be changed for real data I think. By this I mean the whole method
            if (Math.Abs(midValue - midValuePrevious) < 5)
            {
                midIsRelevant = true;
            }
            midValuePrevious = midValue;

            leftValue = kalmanLeft.Output(whatWeKnow.leftSensor);
            if (Math.Abs(leftValue - leftValuePrevious) < 5)
            {
                leftIsRelevant = true;
            }

            leftValuePrevious = leftValue;

            rightValue = kalmanRight.Output(whatWeKnow.rightSensor);
            if (Math.Abs(rightValue - rightValuePrevious) < 5)
            {
                rightIsRelevant = true;
            }
            rightValuePrevious = rightValue;

            await formControl.MonteLocalization.StartTasksToMoveParticles((float)d, (float)currentRotation);


            if (midIsRelevant && leftIsRelevant && rightIsRelevant)
            {
                await formControl.MonteLocalization.StartTasksToUpdateWeights(
                       [midValue, leftValue, rightValue], 75);
                formControl.MonteLocalization.Resample(false, 0);
                await formControl.MonteLocalization.StartTasksToUpdateWeights(
                       [midValue, leftValue, rightValue], 75);
            }
            var currentEstimate = formControl.MonteLocalization.EstimatePosition();

            //formControl.estimatedPos = formControl.MonteLocalization.EstimatePosition();
            formControl.newInfoForAutoMovement_FLAG = true;
            
            double tempTheta = 0;
            if (currentEstimate.Theta > 0)
            {
                tempTheta = currentEstimate.Theta;
            }
            else
            {
                tempTheta = 360 + currentEstimate.Theta;
            }

            try
            {
                //double shiftedX = currentEstimate.X + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.LeftSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.LeftSensorOffsets.Item2, 2)) * Math.Cos((tempTheta + GlobalConstants.LeftDegrees) * Math.PI / 180));
                //double shiftedY = currentEstimate.Y + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.LeftSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.LeftSensorOffsets.Item2, 2)) * Math.Sin((tempTheta + GlobalConstants.LeftDegrees) * Math.PI / 180));
                //bitmap.SetPixel((int)shiftedX, (int)shiftedY, Color.Blue);

                //shiftedX = currentEstimate.X + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.MidSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.MidSensorOffsets.Item2, 2)) * Math.Cos((tempTheta + GlobalConstants.MidDegrees) * Math.PI / 180));
                //shiftedY = currentEstimate.Y + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.MidSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.MidSensorOffsets.Item2, 2)) * Math.Sin((tempTheta + GlobalConstants.MidDegrees) * Math.PI / 180));
                //bitmap.SetPixel((int)shiftedX, (int)shiftedY, Color.Yellow);

                //shiftedX = currentEstimate.X + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.RightSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.RightSensorOffsets.Item2, 2)) * Math.Cos((tempTheta + GlobalConstants.RightDegrees) * Math.PI / 180));
                //shiftedY = currentEstimate.Y + (int)Math.Round(Math.Sqrt(Math.Pow(GlobalConstants.RightSensorOffsets.Item1, 2) + Math.Pow(GlobalConstants.RightSensorOffsets.Item2, 2)) * Math.Sin((tempTheta + GlobalConstants.RightDegrees) * Math.PI / 180));
                //bitmap.SetPixel((int)shiftedX, (int)shiftedY, Color.Green);

                bitmap.SetPixel((int)currentEstimate.X, (int)currentEstimate.Y, Color.Red);

            }
            catch (Exception)
            {
                throw;
            }
           
        }
        public void StartInterpreting()
        {
            stopInterpreting = false;
            while (!stopInterpreting)   // A loop where we take the messages from a string queue (stringsToBeInterpreted) and do actions depending on the first keyword of the message.
            {
                try
                {

                    if (stringsToBeInterpreted.TryDequeue(out (string key, object value) entry))
                    {

                        if (entry.key != null)
                        {
                            if (entry.key == "mapPoint")
                            {
                                 MakeMap((JsonMessageClass)entry.value);
                               // JsonMessageClass mes = (JsonMessageClass)entry.value;
                               // formControl.addToTextBox(mes.direction.ToString() + "  |  ");
                                 counter++;
                                if (counter > 2)
                                {
                                    counter = 0;
                                    UpdateImg();
                                }
                            }
                            else if (entry.key == "MCL")
                            {


                                UpdateMCL((JsonMessageClass)entry.value);

                                counter++;
                                if (counter > 2)
                                {
                                    counter = 0;
                                    UpdateImg();
                                }
                            }
                            else if (entry.key == "save")
                            {
                                using (bitmap)
                                {
                                    bitmap.Bitmap.Save("bigMap.JPG", ImageFormat.Jpeg);

                                }
                               
                            }
                            else if (entry.key == "calib")
                            {
                                var message = (magDataMessage)entry.value;
                                magDataList.Add(message.x + " " + message.y + " " + message.z);
                            }
                            else if (entry.key == "endCalib")
                            {
                                using (TextWriter tw = new StreamWriter("magDataCalibrated.txt"))
                                {
                                    foreach (String s in magDataList)
                                        tw.WriteLine(s);
                                }
                            }

                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Source);
                    Console.WriteLine(ex.Message);


                }
            }
        }



         private void FillRotatedRectangle(CustomBitmap bmp, (double, double)[] corners, Color color)
        {
            // Get bounding box
            double minX = Math.Min(Math.Min(corners[0].Item1, corners[1].Item1), Math.Min(corners[2].Item1, corners[3].Item1));
            double maxX = Math.Max(Math.Max(corners[0].Item1, corners[1].Item1), Math.Max(corners[2].Item1, corners[3].Item1));
            double minY = Math.Min(Math.Min(corners[0].Item2, corners[1].Item2), Math.Min(corners[2].Item2, corners[3].Item2));
            double maxY = Math.Max(Math.Max(corners[0].Item2, corners[1].Item2), Math.Max(corners[2].Item2, corners[3].Item2));

            // Iterate over all pixels in the bounding box
            for (int x = (int)minX; x <= (int)maxX; x++)
            {
                for (int y = (int)minY; y <= (int)maxY; y++)
                {
                    // Check if the point (x, y) is inside the rotated rectangle
                    if (IsPointInsidePolygon(x, y, corners))
                    {

                        if (x >= 0 && x < bmp.Width && y >= 0 && y < bmp.Height)
                        {
                            //bmp.SetPixel(x, y, color);
                            gridForMapping[x, y].WasWithinBoundingBox = true;
                            pointsThatNeedVisualChange.Add((x, y));
                        }
                    }
                }
            }
        }

        static bool IsPointInsidePolygon(double x, double y, (double, double)[] polygon)
        {
            int count = polygon.Length;
            bool inside = false;

            for (int i = 0, j = count - 1; i < count; j = i++)
            {
                double xi = polygon[i].Item1, yi = polygon[i].Item2;
                double xj = polygon[j].Item1, yj = polygon[j].Item2;

                bool intersect = ((yi > y) != (yj > y)) &&
                                 (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
                if (intersect)
                    inside = !inside;
            }

            return inside;
        }

        static (double, double)[] GetExpandedRectangle(double Cx, double Cy, double W, double H, double theta, double margin)
        {
            double newW = W + 2 * margin;
            double newH = H + 2 * margin;
            double halfW = newW / 2, halfH = newH / 2;

            (double, double)[] corners = new (double, double)[4]
            {
            (-halfW, -halfH), // Bottom-left
            (halfW, -halfH),  // Bottom-right
            (halfW, halfH),   // Top-right
            (-halfW, halfH)   // Top-left
            };

            for (int i = 0; i < corners.Length; i++)
            {
                corners[i] = RotatePoint(corners[i].Item1, corners[i].Item2, Cx, Cy, theta);
            }

            return corners;
        }

        static (double, double) RotatePoint(double x, double y, double Cx, double Cy, double theta)
        {
            double cosT = Math.Cos(theta);
            double sinT = Math.Sin(theta);

            double newX = x * cosT - y * sinT + Cx;
            double newY = x * sinT + y * cosT + Cy;

            return (newX, newY);
        }
    }
}
