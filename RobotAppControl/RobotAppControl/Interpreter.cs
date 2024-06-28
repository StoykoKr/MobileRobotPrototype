using System;
using System.Collections.Generic;
using System.Drawing.Imaging;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Concurrent;

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

        public Interpreter(ref ConcurrentQueue<string> strings, ref CustomBitmap actualMap, ref double curX, ref double curY, ref double curRotation, Form1 formReference)
        {
            stringsToBeInterpreted = strings;
            bitmap = actualMap;
            currentX = curX;
            currentY = curY;
            currentRotation = curRotation;
            formControl = formReference;

        }
        private void UpdateImg()
        {
            formControl.Invoke(formControl.myDelagate);
        }
        private List<string[]> knownList = new List<string[]>();
        private void MakeMap(string[] whatWeKnow)
        {
            if (bitmap != null)
            {

                try
                {

                    int tempY = bitmap.Height / 2;
                    int tempX = bitmap.Width / 2;
                    // whatWeKnow[1]; //how much we moved
                    //whatWeKnow[2];// direction
                    // if (whatWeKnow[2] != "225.0")
                    //  {
                    currentRotation = double.Parse(whatWeKnow[2]);
                    //   }
                    movedTotalRecieved += double.Parse(whatWeKnow[1]);
                    double movementDistance = float.Parse(whatWeKnow[1]) * 0.1;
                    currentX += movementDistance * Math.Cos(currentRotation * Math.PI / 180);
                    currentY += movementDistance * Math.Sin(currentRotation * Math.PI / 180);
                    int x = tempX + (int)Math.Round(currentX);
                    int y = tempY + (int)Math.Round(currentY);
                    //   knownList.Add(whatWeKnow);
                    Color newColor = Color.White;
                    for (int j = 3; j < whatWeKnow.Length; j += 2)
                    {
                        if (float.Parse(whatWeKnow[j + 1]) < 200)  // if too far ignore
                        {
                            int centralPixelX = x + (int)Math.Round(float.Parse(whatWeKnow[j + 1]) * Math.Cos(((float.Parse(whatWeKnow[j]) + currentRotation)) * Math.PI / 180));
                            int centralPixelY = y + (int)Math.Round(float.Parse(whatWeKnow[j + 1]) * Math.Sin(((float.Parse(whatWeKnow[j]) + currentRotation)) * Math.PI / 180));


                            if (j == 3)
                            {
                                newColor = Color.Red;
                                bitmap.SetPixel(
                         x,
                         y, Color.White);
                            }
                            else if ( j == 5)
                            {
                                newColor = Color.Green;

                            }else
                            {
                                newColor = Color.Blue;
                            }
                            
                            
                            
                            
                            //    for (int i = -1; i < 2; i++)
                            //     {
                            //       for (int k = -1; k < 2; k++)
                            //         {
                            //   bitmap.SetPixel(
                            //   centralPixelX + i,
                            //   centralPixelY + k, Color.White);

                            //     }
                            //    }
                            bitmap.SetPixel(
                         centralPixelX,
                         centralPixelY, newColor);
                        }
                    }
                    //    if (knownList.Count > 200)
                    //  {
                    //breakpoint spot
                    //}
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
        List<string[]> listOfArr = new List<string[]>();

        public void StartInterpreting()
        {
            stopInterpreting = false;
            while (!stopInterpreting)
            {
                try
                {

                    if (stringsToBeInterpreted.TryDequeue(out string entry))
                    {

                        if (entry != null)
                        {
                            string[] splitData = entry.Split('|');
                            if (splitData[0] == "scanRSSI") // update readings on the current position
                            { }
                            else if (splitData[0] == "report") // mark the current readings as the specified position
                            {
                                listOfArr.Add(splitData);
                                if (listOfArr.Count > 500)
                                {
                                    //???
                                }
                            }
                            else if (splitData[0] == "guess") // get data on current position and make a guess 
                            { }
                            else if (splitData[0] == "mapPoint")
                            {
                                MakeMap(splitData);
                                counter++;
                                if (counter > 2)
                                {
                                    counter = 0;
                                    UpdateImg();
                                }
                            }
                            else if (splitData[0] == "ready")
                            { }
                            else if (splitData[0] == "moved")
                            {
                                // [1] -> how much  [2] -> currentRotation
                                //  currentRotation = double.Parse(splitData[2]);
                                //  currentX += double.Parse(splitData[1]) * Math.Cos(currentRotation * Math.PI / 180);
                                //  currentY += double.Parse(splitData[1]) * Math.Sin(currentRotation * Math.PI / 180);
                            }
                            else if (splitData[0] == "save")
                            {
                                using (bitmap)
                                {
                                    bitmap.Bitmap.Save("bigMap.JPG", ImageFormat.Jpeg);

                                }
                                Console.WriteLine("Saved!");
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
