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

        public Interpreter(ref ConcurrentQueue<(string key, object value)> strings, ref CustomBitmap actualMap, ref double curX, ref double curY, ref double curRotation, Form1 formReference)
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
        private void MakeMap(JsonMessageClass whatWeKnow) // Paints the map with the information we got. 
        {
            if (bitmap != null)
            {
                try
                {
                    int tempY = bitmap.Height / 2;
                    int tempX = bitmap.Width / 2;
                    currentRotation = whatWeKnow.direction + 270; //270;    // ADJUST HERE BY 90 IN A DIRECTION NOT SURE WHICH // Added change idk what now
                    double movementDistance = whatWeKnow.movement * 0.1;
                    currentX += movementDistance * Math.Cos(currentRotation * Math.PI / 180);
                    currentY += movementDistance * Math.Sin(currentRotation * Math.PI / 180);
                    int x = tempX + (int)Math.Round(currentX);
                    int y = tempY + (int)Math.Round(currentY);
                    int offsedX = 0;
                    int offsedY = 0;
                    int nehsto = 0;
                    Color newColor = Color.White;



                    offsedY = -30;
                    offsedX = 0;
                    newColor = Color.Red;
                    currentDegrees = MidDegrees;
                    nehsto = 1;
                    int centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(whatWeKnow.midSensor * nehsto * Math.Cos((degreeOffsetLeft/*degreeOffsetMid*/ + currentRotation) * Math.PI / 180));
                    int centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(whatWeKnow.midSensor * nehsto * Math.Sin((degreeOffsetLeft/*degreeOffsetMid*/ + currentRotation) * Math.PI / 180));
                    bitmap.SetPixel(
                 centralPixelX,
                 centralPixelY, newColor);



                    offsedY = -17;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                    offsedX = -24;
                    newColor = Color.Green;
                    currentDegrees = LeftDegrees;
                    nehsto = -1;
                    centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(whatWeKnow.leftSensor * nehsto * Math.Cos((degreeOffsetLeft + currentRotation) * Math.PI / 180));
                    centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(whatWeKnow.leftSensor * nehsto * Math.Sin((degreeOffsetLeft + currentRotation) * Math.PI / 180));
                    bitmap.SetPixel(
                 centralPixelX,
                 centralPixelY, newColor);


                    offsedY = -20;  //   CHANGE OFFSETS ACCORDING TO THE ACTUAL DISTANCE NEEDS TO BE MEASURED
                    offsedX = 26;
                    newColor = Color.Blue;
                    currentDegrees = RightDegrees;
                    nehsto = -1;
                    centralPixelX = x + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Cos((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(whatWeKnow.rightSensor * nehsto * Math.Cos((degreeOffsetRight + currentRotation) * Math.PI / 180));
                    centralPixelY = y + (int)Math.Round(Math.Sqrt(Math.Pow(offsedX, 2) + Math.Pow(offsedY, 2)) * Math.Sin((currentDegrees + currentRotation) * Math.PI / 180)) + (int)Math.Round(whatWeKnow.rightSensor * nehsto * Math.Sin((degreeOffsetRight + currentRotation) * Math.PI / 180));
                    bitmap.SetPixel(
                 centralPixelX,
                 centralPixelY, newColor);

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
