using System;
using System.Collections.Generic;
using System.Drawing.Imaging;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using System.Collections;

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
        private double currentDegrees = 0;
        private int degreeOffsetMid = 0;
        private int degreeOffsetLeft = 90;
        private int degreeOffsetRight = -90;      
        private double MidDegrees = Math.Atan2(0, 30) * (180.0 / Math.PI);  //mid  change Math.Atan2(x,y) with new values if sensors are moved
        private double LeftDegrees = Math.Atan2(-24, 17) * (180.0 / Math.PI);  //left
        private double RightDegrees = Math.Atan2(26, 20) * (180.0 / Math.PI);  //right
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
        public Interpreter(ref ConcurrentQueue<(string key, object value)> strings, ref CustomBitmap actualMap, ref double curX, ref double curY, ref double curRotation, Form1 formReference)
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

            for (int angle = angleMin; angle <= angleMax; angle++)
            {
                double radians = angle * Math.PI / 180.0;
                int targetX = (int)Math.Round(Ax + distance * Math.Cos(radians));
                int targetY = (int)Math.Round(Ay + distance * Math.Sin(radians));

                // Store the exact arc boundary point
                if (IsInBounds(targetX, targetY))
                    arcBoundaryPoints.Add((targetX, targetY));

                // Use Bresenham to get all interior points
                foreach (var point in BresenhamLine(Ax, Ay, targetX, targetY))
                {
                    if (point != (targetX, targetY) && IsInBounds(point.Item1, point.Item2))
                        interiorPoints.Add(point);
                }
            }

            foreach (var (x, y) in arcBoundaryPoints)
            {
               // if (!gridForMapping[x, y].IsProvenEmpty)
                //{
                if (gridForMapping[x, y].IsPossible <= 15)
                    gridForMapping[x, y].IsPossible += 3;
                // }

                if (gridForMapping[x, y].IsPossible >= 8/*gridForMapping[x, y].IsProvenEmpty*/)
                {
                    pointsThatNeedVisualChange.Add((x, y));
                }
            }

            foreach (var (x, y) in interiorPoints)
            {
                //gridForMapping[x, y].IsProvenEmpty++;

                if (gridForMapping[x, y].IsPossible > 0)
                    gridForMapping[x, y].IsPossible --;

                if (gridForMapping[x, y].IsPossible + 1 == 7)
                {
                    pointsThatNeedVisualChange.Add((x, y));
                }
            }
            inputLog.Add($"Arc logic over. Added {pointsThatNeedVisualChange.Count} points for change");
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
        private void MakeMap(JsonMessageClass whatWeKnow) // Paints the map with the information we got. 
        {
            if (bitmap != null)
            {
                try
                {
                    int tempY = bitmap.Height / 2;
                    int tempX = bitmap.Width / 2;
                    currentRotation = whatWeKnow.direction + 270; //270;    // ADJUST HERE BY 90 IN A DIRECTION NOT SURE WHICH // Added change idk what now
                    double movementDistance = whatWeKnow.movement* 0.1;
                    currentX += movementDistance * Math.Cos(currentRotation * Math.PI / 180);
                    currentY += movementDistance * Math.Sin(currentRotation * Math.PI / 180);
                    int x = tempX + (int)Math.Round(currentX);
                    int y = tempY + (int)Math.Round(currentY);
                    int offsedX = 0;
                    int offsedY = 0;
                    int nehsto = 0;
                    Color newColor = Color.Green;
                   bitmap.SetPixel(
                        x,
                        y, newColor);
                    int centralPixelX;
                    int centralPixelY;
                    if (!formControl.currentlyMappingSimulation)
                    {

                        offsedY = -30;
                        offsedX = 0;
                    }
                     newColor = Color.White;
                  //  newColor = Color.Red;
                    currentDegrees = MidDegrees;
                    nehsto = 1;
                    if (whatWeKnow.midSensor < 290)
                    {
                        midValue = kalmanMid.Output(whatWeKnow.midSensor);
                        if(Math.Abs(midValue - midValuePrevious) < 5)
                        {



                            var tempMin = degreeOffsetMid + currentRotation - 15;
                            var tempMax = degreeOffsetMid + currentRotation + 15;
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


                            //inputLog.Add($"Arc with:{x}| {y} | {midValue} | {(int)tempMin} | {(int)tempMax}");
                            MarkPossibleArea(x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                                y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)),
                                midValue, (int)tempMin, (int)tempMax);



                            centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(midValue * nehsto * Math.Cos((/*degreeOffsetLeft*/degreeOffsetMid + currentRotation) * Math.PI / 180));
                            centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(midValue * nehsto * Math.Sin((degreeOffsetMid + currentRotation) * Math.PI / 180));
                            bitmap.SetPixel(
                               centralPixelX,
                               centralPixelY, newColor);




                        }
                        inputLog.Add($"mid:{Math.Round(whatWeKnow.midSensor, 2)} {Math.Round(midValue, 2)} {Math.Round(midValuePrevious, 2)}");
                        midValuePrevious = midValue;
                    }


                    if (whatWeKnow.leftSensor < 290)
                    {
                        if (!formControl.currentlyMappingSimulation)
                        {

                            offsedY = -17;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                            offsedX = -24;
                        }
                        //   newColor = Color.Green;

                        leftValue = kalmanLeft.Output(whatWeKnow.leftSensor);
                        if (Math.Abs(leftValue - leftValuePrevious) < 5)
                        {
                            currentDegrees = LeftDegrees;
                            nehsto = -1;

                            var tempMin = degreeOffsetLeft + currentRotation - 15;
                            var tempMax = degreeOffsetLeft + currentRotation + 15;
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


                            MarkPossibleArea(x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                                y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)),
                                leftValue * nehsto, (int)tempMin, (int)tempMax);  // No idea why I did this, sorry

                            centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(leftValue * nehsto * Math.Cos((degreeOffsetLeft + currentRotation) * Math.PI / 180));
                            centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(leftValue * nehsto * Math.Sin((degreeOffsetLeft + currentRotation) * Math.PI / 180));
                            bitmap.SetPixel(
                         centralPixelX,
                         centralPixelY, newColor);





                        }
                        inputLog.Add($"left:{Math.Round(whatWeKnow.leftSensor, 2)} {Math.Round(leftValue, 2)} {Math.Round(leftValuePrevious, 2)}");
                        leftValuePrevious = leftValue;
                    }

                    if (whatWeKnow.rightSensor < 290)
                    {
                        if (!formControl.currentlyMappingSimulation)
                        {

                            offsedY = -20;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                            offsedX = 26;
                        }
                        //    newColor = Color.Blue;
                        rightValue = kalmanRight.Output(whatWeKnow.rightSensor);
                        if (Math.Abs(rightValue - rightValuePrevious) < 5)
                        {
                            currentDegrees = RightDegrees;
                            nehsto = -1;


                            var tempMin = degreeOffsetRight + currentRotation - 15;
                            var tempMax = degreeOffsetRight + currentRotation + 15;
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


                            MarkPossibleArea(x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)),
                                y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)),
                                rightValue * nehsto, (int)tempMin, (int)tempMax);



                            centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(rightValue * nehsto * Math.Cos((degreeOffsetRight + currentRotation) * Math.PI / 180));
                            centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(rightValue * nehsto * Math.Sin((degreeOffsetRight + currentRotation) * Math.PI / 180));
                            bitmap.SetPixel(
                         centralPixelX,
                         centralPixelY, newColor);
                        }
                        inputLog.Add($"right:{Math.Round(whatWeKnow.rightSensor, 2)} {Math.Round(rightValue, 2)} {Math.Round(rightValuePrevious, 2)}");
                        rightValuePrevious = rightValue;
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
                 if (gridForMapping[item.Item1, item.Item2].IsPossible >= 8 /*gridForMapping[item.Item1, item.Item2].IsProvenEmpty*/)
                {
                    bitmap.SetPixel(
                        item.Item1,
                        item.Item2, Color.Red);
                }
                else
                {
                    bitmap.SetPixel(
                         item.Item1,
                         item.Item2, Color.Black);
                }
            }
            pointsThatNeedVisualChange.Clear();
        }
        public void StopInterpreting()
        {
            stopInterpreting = true;
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
    }
}
