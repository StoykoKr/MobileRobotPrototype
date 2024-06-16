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
       
        public Interpreter(ref ConcurrentQueue<string> strings,ref CustomBitmap actualMap, ref double curX, ref double curY, ref double curRotation,Form1 formReference) {
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
        private void MakeMap(string[] whatWeKnow)
        {
            if (bitmap != null)
            {

                try
                {

                    int tempY = bitmap.Height / 2;
                    int tempX = bitmap.Width / 2;
                    // whatWeKnow[1]; //row
                    // whatWeKnow[2]; //col
                    //whatWeKnow[3];// direction
                    if (whatWeKnow[3] != "225.0")
                    {
                        currentRotation = double.Parse(whatWeKnow[3]);
                    }
                    currentX += float.Parse(whatWeKnow[1]) * 0.5 * Math.Cos(currentRotation * Math.PI / 180);
                    currentY += float.Parse(whatWeKnow[1]) * 0.5 * Math.Sin(currentRotation * Math.PI / 180);
                    int x = tempX + (int)Math.Round(currentX);
                    int y = tempY + (int)Math.Round(currentY);

                    for (int j = 4; j < whatWeKnow.Length; j += 2)
                    {
                        if (float.Parse(whatWeKnow[j + 1]) < 120)  // if too far ignore
                        {
                            int centralPixelX = x + (int)Math.Round(float.Parse(whatWeKnow[j + 1]) * 5 * Math.Cos((float.Parse(whatWeKnow[j]) - 90 + currentRotation) * Math.PI / 180));
                            int centralPixelY = y + (int)Math.Round(float.Parse(whatWeKnow[j + 1]) * 5 * Math.Sin((float.Parse(whatWeKnow[j]) - 90 + currentRotation) * Math.PI / 180));
                              Color newColor = new Color();
                              if (j == 4)
                              {
                                  bitmap.SetPixel(
                                      x,
                                      y, Color.Yellow);
                                  newColor = Color.White;
                              }
                              else if (j == 6)
                              {
                                  newColor = Color.Green;
                              }
                              else if (j == 8)
                              {
                                  newColor = Color.Red;
                              } 
                                for (int i = -1; i < 2; i++)
                               {
                                   for (int k = -1; k < 2; k++)
                                    {
                            bitmap.SetPixel(
                            centralPixelX + i,
                            centralPixelY + k, newColor);

                                 }
                                }


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
        public void StopInterpreting() {
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
                            {}
                            else if (splitData[0] == "report") // mark the current readings as the specified position
                            {
                                listOfArr.Add(splitData);
                                if(listOfArr.Count > 10)
                                {
                                    //???
                                }
                            }
                            else if (splitData[0] == "guess") // get data on current position and make a guess 
                            {}
                            else if (splitData[0] == "mapPoint")
                            {
                                MakeMap(splitData);
                                counter++;
                                listOfArr.Add(splitData);
                                if (counter > 2)
                                {
                                    counter = 0;
                                    UpdateImg();
                                }
                            }
                            else if (splitData[0] == "ready")
                            {}
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
