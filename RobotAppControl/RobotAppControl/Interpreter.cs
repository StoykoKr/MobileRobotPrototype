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
        private ConcurrentQueue<string> stringsToBeInterpreted;
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

        private double MidDegrees = Math.Atan2(5, 30) * (180.0 / Math.PI);  //mid  change Math.Atan2(x,y) with new values if sensors are moved
        private double LeftDegrees = Math.Atan2(-22, 16) * (180.0 / Math.PI);  //left
        private double RightDegrees = Math.Atan2(23, 15) * (180.0 / Math.PI);  //right
        // get the data for the points here and use it in the path planning method with modifications for end points. This should avoid colliding paths.
        // PIDs or something needs to be done to make it move in a line.. right now it is not.
        // When we are planning a path right now it does not consider starting rotation at all. It thinks that it is in the correct rotation and the given spot and just goes.
        // a way to resolve the aforementioned problem is to make it check the starting rotation and add a movement and a turn before the start of the path to adjust it accordingly.
        // The problem here is that the easiest way for that would require moving backwards.. which we currently can't. If we could tho the task is simple just use the existing logic for planned turns and play with the values a little
        // last time there was another bug/issues with the wheels related to PIDs and/or constant changes in the PWM signal
        public Interpreter(ref ConcurrentQueue<string> strings, ref CustomBitmap actualMap, ref double curX, ref double curY, ref double curRotation, Form1 formReference)
        {
            magDataList = new List<string>();
            stringsToBeInterpreted = strings;
            bitmap = actualMap;
            currentX = curX;
            currentY = curY;
            currentRotation = curRotation;
            formControl = formReference;
        }
        private void UpdateImg()  // Invokes the UI thread to show the results of the newly painted map.
        {
            formControl.Invoke(formControl.myDelagate);
        }
        private void MakeMap(string[] whatWeKnow) // Paints the map with the information we got. 
        {
            if (bitmap != null)
            {

                try
                {

                    int tempY = bitmap.Height / 2;
                    int tempX = bitmap.Width / 2;
                    currentRotation = double.Parse(whatWeKnow[2]) + 270; //270;    // ADJUST HERE BY 90 IN A DIRECTION NOT SURE WHICH // Added change idk what now
                    double movementDistance = float.Parse(whatWeKnow[1]) * 0.1;
                    currentX += movementDistance * Math.Cos(currentRotation * Math.PI / 180);
                    currentY += movementDistance * Math.Sin(currentRotation * Math.PI / 180);
                    int x = tempX + (int)Math.Round(currentX);
                    int y = tempY + (int)Math.Round(currentY);
                    int offsedX = 0;
                    int offsedY = 0;
                    int nehsto = 0;
                    Color newColor = Color.White;
                    for (int j = 3; j < whatWeKnow.Length - 1; j += 2)
                    {
                        if (float.Parse(whatWeKnow[j + 1]) < 350)  // if too far ignore
                        {
                            if (j == 3)
                            {
                                offsedY = -30;  //  5 CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED 
                                offsedX = 5;  // they may be reversed or with opposite sign aka + -  same with the rest of the values
                                newColor = Color.Red;
                                currentDegrees = MidDegrees;
                                nehsto = 1;
                            //    bitmap.SetPixel(
                        // x,
                       //  y, Color.White);
                            }
                            else if (j == 5)
                            {
                                offsedY = -16;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                                offsedX = -22;
                                newColor = Color.Green;
                                currentDegrees = LeftDegrees;
                                nehsto = -1;

                            }
                            else
                            {
                                offsedY = -15;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                                offsedX = 23;
                                newColor = Color.Blue;
                                currentDegrees = RightDegrees;
                                nehsto = -1;
                            }
                            int centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(float.Parse(whatWeKnow[j + 1]) * nehsto * Math.Cos(((float.Parse(whatWeKnow[j]) + currentRotation)) * Math.PI / 180));
                            int centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(float.Parse(whatWeKnow[j + 1]) * nehsto * Math.Sin(((float.Parse(whatWeKnow[j]) + currentRotation)) * Math.PI / 180));
                            bitmap.SetPixel(
                         centralPixelX,
                         centralPixelY, newColor);
                        }
                    }
                }

                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                    throw;
                }
            }
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

                    if (stringsToBeInterpreted.TryDequeue(out string entry))
                    {

                        if (entry != null)
                        {
                            string[] splitData = entry.Split('|');
                            if (splitData[0] == "mapPoint")
                            {
                                MakeMap(splitData);
                                counter++;
                                if (counter > 2)
                                {
                                    counter = 0;
                                    UpdateImg();
                                }
                            }
                            else if (splitData[0] == "save")
                            {
                                using (bitmap)
                                {
                                    bitmap.Bitmap.Save("bigMap.JPG", ImageFormat.Jpeg);

                                }
                                Console.WriteLine("Saved!");
                            }
                            else if (splitData[0] == "calib")
                            {
                                magDataList.Add(splitData[1] + " " + splitData[2] + " " + splitData[3]);
                            }
                            else if (splitData[0] == "endCalib")
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
