using System.Collections.Concurrent;
using System.Diagnostics;
using System.Text;
using System.Drawing.Imaging;
using System.Net.Sockets;
using MQTTnet;
using Newtonsoft.Json;

namespace RobotAppControl
{
    public partial class Form1 : Form
    {
        private string actualAreaPath = "C:\\Users\\s\\Desktop\\The_actual_area_we_are_scanning.png";
        private string readyMapForTheAutonomousPart = "C:\\Users\\s\\Desktop\\third.jpg";  //This should point to the normal map not the _mcl version



        public Grid _grid;
        public Grid _MCL_grid;
        private Robot _robot = null;
        private Robot simulatedActualRobot = null;
        private PictureBox PictureBox;
        private int picture_offsetX = 0, picture_offsetY = 0;
        private int _xPos = 0;  // mouseposition at MouseDown
        private int _yPos = 0;  // mouseposition at MouseDown
        private bool _dragging = false;
        private Rectangle _imgRect = new Rectangle(0, 0, 0, 0);
        private CustomBitmap custom;
        private CustomBitmap customMCL;
        private CustomBitmap occupancyMap;
        ConcurrentQueue<(string key, object value)> stringsToBeInterpreted;
        ConcurrentQueue<(float, float, float)> TotalRevievedStrings = new ConcurrentQueue<(float, float, float)>();
        Interpreter? interpreter;
        Thread interpreterThread;
        double currentX = 0;
        double currentY = 0;
        double currentRotation = 0;
        Keys currentlyPressedKey = Keys.None;
        private bool currentlyControlling = false;
        private Rectangle rectangle;
        public delegate void RefreshTheImg();
        public RefreshTheImg myDelagate;
        private bool settingStart = false;
        private bool settingEnd = false;
        public int startX = 0;
        public int startY = 0;
        public int endX = 0;
        public int endY = 0;
        private List<Node> finalPath = null;
        private MqttClientFactory mqttFactory = new MqttClientFactory();
        public bool newInfoForAutoMovement_FLAG = false;
        ConcurrentQueue<string> rawMagDataToBeWorkedOn;
        List<double> segments = new List<double>();
        List<double> turns = new List<double>();
        List<bool> turnsss = new List<bool>();
        private IMqttClient? imagePublisherClient = null;
        public MonteCarloLocal MonteLocalization = null;
        public volatile bool currentlyMappingSimulation = false;
        public CustomBitmap simulatedMapArea;
        public Grid simulatedMapGrid;
        public List<JsonMessageClass> feedback = new List<JsonMessageClass>();
        public List<(double, double, double, double, double)> values_angleToTarget_SteeringAngle = new List<(double, double, double, double, double)>();
        public List<(double, double, double, double)> values = new List<(double, double, double, double)>();
        public List<magDataMessage> testAgain = new List<magDataMessage>();
        public Form1()
        {
            InitializeComponent();
            stringsToBeInterpreted = new ConcurrentQueue<(string key, object value)>();
            rawMagDataToBeWorkedOn = new ConcurrentQueue<string>();
            PictureBox = this.pBox_Area;
            this.KeyPreview = true;
            myDelagate = new RefreshTheImg(RefreshPicture);
            InitMQTTClient();
        }
       
        private Grid SetObstaclesFromMap(CustomBitmap map)
        {
            Grid gridToBeSet = new Grid(map.Width, map.Height);

            for (int x = 0; x < map.Width; x++)
            {
                for (int y = 0; y < map.Height; y++)
                {
                    Color pixelColor = map.GetPixel(x, y);
                    if (IsObstacle(pixelColor))
                    {
                        gridToBeSet.SetWalkable(x, y, false);
                    }
                }
            }
            for (int x = 0; x < map.Width; x++)
            {
                for (int y = 0; y < map.Height; y++)
                {
                    if (gridToBeSet.IsWalkable(x, y) == false)
                    {
                        for (int i = -15; i <= 15; i++)
                        {
                            for (int j = -15; j <= 15; j++)
                            {
                                if (gridToBeSet.IsWalkable(x + i, y + j) == true)
                                {
                                    float tentativeCost = 1 + (50 - Math.Abs(i) - Math.Abs(j)) / 2;
                                    if (gridToBeSet.GetCost(x + i, y + j) < tentativeCost)
                                    {
                                        gridToBeSet.SetCost(x + i, y + j, tentativeCost);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return gridToBeSet;
        }

        private bool IsObstacle(Color color)
        {
            // Define your criteria for an obstacle. For example, a pixel is an obstacle if it is black.
            return color.R + color.G + color.B < 100; // Example threshold for black
        }
        private void StartInterpreting()
        {
            if (interpreter != null)
            {

                interpreterThread = new Thread(() => interpreter.StartInterpreting());
                interpreterThread.Start();
            }
        }
        private void StopInterpreting()
        {
            if (interpreter != null)
            {
                interpreter.StopInterpreting();
                interpreter = null;
            }
        }
        private CustomBitmap LoadImageAsCustomBitmap(string filepath)
        {
            Bitmap bmploc = new Bitmap(filepath);
            Rectangle rect = new Rectangle(0, 0, bmploc.Width, bmploc.Height);
            BitmapData bmpData =
            bmploc.LockBits(rect, ImageLockMode.ReadOnly,
            PixelFormat.Format32bppArgb);
            IntPtr ptr = bmpData.Scan0;
            int bytes = Math.Abs(bmpData.Stride) * bmploc.Height;
            byte[] rgbValues = new byte[bytes];

            System.Runtime.InteropServices.Marshal.Copy(ptr, rgbValues, 0, bytes);

            CustomBitmap customBitmap = new CustomBitmap(bmploc.Width, bmploc.Height);

            bmploc.UnlockBits(bmpData);


            for (int counter = 0; counter < bytes; counter += 4)
            {
                customBitmap.WriteToBits(counter / 4, Color.FromArgb(255, rgbValues[counter + 2], rgbValues[counter + 1], rgbValues[counter]));

            }
            return customBitmap;

        }

        private void LoadDefaultImage()
        {
            //custom = LoadImageAsCustomBitmap("Default");
            //_imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
            //PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);
            MultiLoadLoadImageAsMap(readyMapForTheAutonomousPart);

        }
        private void SaveDefaultImage()
        {
            custom.Bitmap.Save("Default");
        }
        private void LoadImageAsMap()
        {
            using (OpenFileDialog dlg = new OpenFileDialog())
            {
                dlg.Title = "Open Image";
                dlg.Filter = "Image Files | *.jpg; *.jpeg; *.png; *.gif; *.tif";

                if (dlg.ShowDialog() == DialogResult.OK)
                {
                    // bmp = new Bitmap(dlg.FileName);


                    custom = LoadImageAsCustomBitmap(dlg.FileName);

                    _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
                    PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);

                }


            }
        }
        private void MultiLoadLoadImageAsMap(string? path)
        {
            if (path == null)
            {

                using (OpenFileDialog dlg = new OpenFileDialog())
                {
                    dlg.Title = "Open Image";
                    dlg.Filter = "Image Files | *.jpg; *.jpeg; *.png; *.gif; *.tif";

                    if (dlg.ShowDialog() == DialogResult.OK)
                    {
                        // bmp = new Bitmap(dlg.FileName);


                        custom = LoadImageAsCustomBitmap(dlg.FileName);

                        var mclName = dlg.FileName;
                        var split = mclName.Split("\\");
                        var theOne = split.Last().Split(".");
                        StringBuilder newOne = new StringBuilder();
                        for (int i = 0; i < split.Length - 1; i++)
                        {
                            newOne.Append(split[i]);
                            newOne.Append("\\\\");
                        }
                        newOne.Append(theOne[0]);
                        newOne.Append("_mcl.");
                        newOne.Append(theOne[1]);
                        customMCL = LoadImageAsCustomBitmap(newOne.ToString());

                        _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
                        PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);

                    }
                }
            }
            else
            {
                custom = LoadImageAsCustomBitmap(path);

                var mclName = path;
                var split = mclName.Split("\\");
                var theOne = split.Last().Split(".");
                StringBuilder newOne = new StringBuilder();
                for (int i = 0; i < split.Length - 1; i++)
                {
                    newOne.Append(split[i]);
                    newOne.Append("\\\\");
                }
                newOne.Append(theOne[0]);
                newOne.Append("_mcl.");
                newOne.Append(theOne[1]);
                customMCL = LoadImageAsCustomBitmap(newOne.ToString());

                _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
                PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);
            }
        }
        private void btn_LoadImageAsMap_Click(object sender, EventArgs e)
        {
            LoadImageAsMap();
        }
        private void pBox_Area_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left)
                return;

            _dragging = true;
            _xPos = e.X;
            _yPos = e.Y;
        }
        private void pBox_Area_Paint(object sender, PaintEventArgs e)
        {
            if (PictureBox != null && _dragging)
            {
                e.Graphics.DrawImage(custom.Bitmap, _imgRect);
            }
        }
        private void pBox_Area_MouseMove(object sender, MouseEventArgs e)
        {
            if (!_dragging || custom == null)
                return;

            if (e.Button == MouseButtons.Left)
            {
                picture_offsetX = picture_offsetX - (_xPos - e.X);
                picture_offsetY = picture_offsetY - (_yPos - e.Y);
                _xPos = e.X;
                _yPos = e.Y;
                _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);

                PictureBox.Invalidate(); // to redraw theimage
            }

        }
        private async void RequestDataFromBot()
        {
            int counterForRequest = 0;
            Stopwatch sw = Stopwatch.StartNew();
            while (counterForRequest < 5)
            {
                if (sw.ElapsedMilliseconds > 250)
                {
                    var message = new
                    {
                        requestMapData = 1
                    };
                    counterForRequest++;
                    PublishJsonMessageAsync("MapDataRequest", message, 1);
                    sw.Restart();
                }
            }
        }
        public void RefreshPicture()
        {
            PictureBox.Invalidate();
            _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
            PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);
        }
        private void AreaClick(Point coordinates) // TODO add MQTT call
        {
            txtBox_TextOutput.Clear();
            txtBox_TextOutput.AppendText($"Point x = {coordinates.X - picture_offsetX} \n");
            txtBox_TextOutput.AppendText($"Point y = {coordinates.Y - picture_offsetY} \n");
            txtBox_TextOutput.AppendText($"Key pressed = {currentlyPressedKey} \n");

            if (MonteLocalization == null)
            {
                MonteLocalization = new MonteCarloLocal(400, coordinates.X - picture_offsetX, coordinates.Y - picture_offsetY, 20, 5, _MCL_grid);// _grid is old
                currentX = coordinates.X - picture_offsetX;
                currentY = coordinates.Y - picture_offsetY;
                txtBox_TextOutput.AppendText($"MonteLocalization started \n");
            }

            if (settingStart)
            {
                startX = coordinates.X - picture_offsetX;
                startY = coordinates.Y - picture_offsetY;
                if (_robot == null)
                {
                    _robot = new Robot(_grid, custom, startX, startY, this, 63);//old was 16.5
                    _robot.setThetaActual(90);
                }
                settingStart = false;
                mclIsON = true;
                currentlyMapping = false;
            }
            else if (settingEnd)
            {
                endX = coordinates.X - picture_offsetX;
                endY = coordinates.Y - picture_offsetY;
                settingEnd = false;
            }
        }
        private void pBox_Area_Click(object sender, EventArgs e)
        {
            MouseEventArgs me = (MouseEventArgs)e;
            if (me.Button != MouseButtons.Right)
                return;
            Point coordinates = me.Location;
            AreaClick(coordinates);
        }
        private bool CheckConnection()
        {
            return true;

        }
        private void ImageSave()
        {
            SaveFileDialog saveFileDialog1 = AskSaveFile();
            if (saveFileDialog1.FileName != "")
            {
                System.IO.FileStream fs =
                   (System.IO.FileStream)saveFileDialog1.OpenFile();
                switch (saveFileDialog1.FilterIndex)
                {
                    case 1:
                        custom.Bitmap.Save(fs,
                          ImageFormat.Jpeg);
                        break;

                    case 2:
                        custom.Bitmap.Save(fs,
                           ImageFormat.Bmp);
                        break;

                    case 3:
                        custom.Bitmap.Save(fs,
                           ImageFormat.Gif);
                        break;
                    case 4:
                        custom.Bitmap.Save(fs,
                           ImageFormat.Png);
                        break;
                }

                fs.Close();
            }
        }
        private void btn_SaveImg_Click(object sender, EventArgs e)
        {
            ImageSave();

        }
        private void HandleAdjacentPixels(int iCentr, int jCentr, int spread, Dictionary<Tuple<int, int>, int> affectedCells)
        {
            Color current = custom.GetPixel(iCentr, jCentr);
            Color currentAdj;
            if ((current.R + current.G + current.B) >= 10)
            {
                for (int i = iCentr - spread; i <= iCentr + spread; i++)
                {
                    for (int j = jCentr - spread; j <= jCentr + spread; j++)
                    {
                        if (i >= 0 && i < custom.Width && j >= 0 && j < custom.Height)
                        {
                            currentAdj = custom.GetPixel(i, j);
                            if ((currentAdj.R /*+ currentAdj.G*/ + currentAdj.B) >= 250)
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
            }
        }
        public SaveFileDialog AskSaveFile()
        {
            if (this.InvokeRequired)
            {
                return (SaveFileDialog)Invoke(new Func<SaveFileDialog>(() => AskSaveFile()));
            }
            else
            {
                var sfd = new SaveFileDialog();
                sfd.Filter = "JPeg Image|*.jpg|Bitmap Image|*.bmp|Gif Image|*.gif|PNG Image|*.png";
                sfd.Title = "Save an Image File";
                sfd.ShowDialog();
                return sfd;
            }
        }
        private string filelocationToSaveOccMap;
        private int filterOption = 0;
        private void ConvertToOccMap(int size, bool is_MCL_map)
        {
            occupancyMap = new CustomBitmap(custom.Width, custom.Height);
            using (occupancyMap)
            {
                Graphics g = Graphics.FromImage(occupancyMap.Bitmap);
                g.Clear(Color.White);
                Dictionary<Tuple<int, int>, int> AffectedCells = new Dictionary<Tuple<int, int>, int>();
                for (int i = 0; i < occupancyMap.Width; i++)
                {
                    for (int j = 0; j < occupancyMap.Height; j++)
                    {
                        HandleAdjacentPixels(i, j, 2, AffectedCells);
                    }
                }
                foreach (var item in AffectedCells)
                {
                    if (item.Value > 5)
                    {
                        rectangle = new Rectangle(item.Key.Item1, item.Key.Item2, 1, 1);
                        rectangle.Inflate(size, size);
                        g.FillRectangle(Brushes.Black, rectangle);
                    }
                }
                if (String.IsNullOrEmpty(filelocationToSaveOccMap))
                {
                    SaveFileDialog saveFileDialog1 = AskSaveFile();
                    if (saveFileDialog1.FileName != "")
                    {
                        filelocationToSaveOccMap = saveFileDialog1.FileName;
                        System.IO.FileStream fs =
                           (System.IO.FileStream)saveFileDialog1.OpenFile();
                        filterOption = saveFileDialog1.FilterIndex;
                        switch (saveFileDialog1.FilterIndex)
                        {
                            case 1:
                                occupancyMap.Bitmap.Save(fs,
                                  ImageFormat.Jpeg);
                                break;

                            case 2:
                                occupancyMap.Bitmap.Save(fs,
                                   ImageFormat.Bmp);
                                break;
                            case 3:
                                occupancyMap.Bitmap.Save(fs,
                                   ImageFormat.Gif);
                                break;
                            case 4:
                                occupancyMap.Bitmap.Save(fs,
                                   ImageFormat.Png);
                                break;
                        }

                        fs.Close();
                    }
                }
                else
                {
                    string path = filelocationToSaveOccMap;
                    var mclName = path;
                    var split = mclName.Split("\\");
                    var theOne = split.Last().Split(".");
                    StringBuilder newOne = new StringBuilder();
                    for (int i = 0; i < split.Length - 1; i++)
                    {
                        newOne.Append(split[i]);
                        newOne.Append("\\\\");
                    }
                    newOne.Append(theOne[0]);
                    newOne.Append("_mcl.");
                    newOne.Append(theOne[1]);
                    FileStream fs = new System.IO.FileStream(newOne.ToString(), System.IO.FileMode.Append, System.IO.FileAccess.Write);
                    switch (filterOption)
                    {
                        case 1:
                            occupancyMap.Bitmap.Save(fs,
                              ImageFormat.Jpeg);
                            break;

                        case 2:
                            occupancyMap.Bitmap.Save(fs,
                               ImageFormat.Bmp);
                            break;

                        case 3:
                            occupancyMap.Bitmap.Save(fs,
                               ImageFormat.Gif);
                            break;
                        case 4:
                            occupancyMap.Bitmap.Save(fs,
                               ImageFormat.Png);
                            break;
                    }

                    fs.Close();

                }
            }
            if (is_MCL_map == false)
            {
                ConvertToOccMap(2, true);
            }
        }

        private void StartConvertingToOcccupancyThread()
        {
            Task task = new Task(() => ConvertToOccMap(32, false));
            task.Start();
        }
        private void btn_ConvertLoadedToOccupancyGrid_Click(object sender, EventArgs e)
        {
            StartConvertingToOcccupancyThread();
        }

       
        Keys lastSentKey = Keys.Enter;

        private double NormalizeAngleDeg(double angle)
        {
            angle %= 360;
            return (angle < 0) ? angle + 360 : angle;
        }
        private double thetaNew = 0;


        public double GetPredictedDistance(double x, double y, double theta, double sensorDirectionLookingAt, (double, double) startingPointOffsets, Color co)
        {

            double thetaused = theta;
            if (thetaused > 360) { thetaused -= 360; }
            if (thetaused < 0) { thetaused += 360; }
            double headingRadians = (thetaused-90) * Math.PI / 180.0; //adjusting for screen 

            double offsetX = startingPointOffsets.Item1 * Math.Cos(headingRadians) - startingPointOffsets.Item2 * Math.Sin(headingRadians);
            double offsetY = startingPointOffsets.Item1 * Math.Sin(headingRadians) + startingPointOffsets.Item2 * Math.Cos(headingRadians);

            double shiftedX = x + offsetX;
            double shiftedY = y + offsetY;


          //  custom.SetPixel((int)shiftedX, (int)shiftedY, co);


            double angl = thetaused + sensorDirectionLookingAt;

            if (angl < 0)
                angl += 360;
            if (angl >= 360)
                angl -= 360;

            // return Raycast(shiftedX, shiftedY, angl, 335, map);
            //if (drawTheDistances)
            //{
          //  MonteCarloLocal.DrawRaycast(shiftedX, shiftedY, angl, 335, _MCL_grid, custom, Color.Blue);
            //}
            return MonteCarloLocal.RaycastCone(shiftedX, shiftedY, angl, 335, GlobalConstants.SensorDispersion, _MCL_grid);
          //  return 0;
        }
        private double SaltTheNumber(double num)
        {
            var randBase = Random.Shared.NextDouble();
            if (randBase < 0.5)
            {
                return num + Random.Shared.NextDouble() * 4;
            }
            else
            {
                var toRetrn = num - Random.Shared.NextDouble() * 4;

                return toRetrn < 0 ? num : toRetrn;
            }
        }
        private async void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            double movementLeft = 0;
            double movementRight = 0;

            if (simMovementInMapButtonIsPressed)
            {
                if (e.KeyCode == Keys.W)
                {
                    movementLeft = 2;
                    movementRight = 2;
                    currentX += movementLeft * Math.Cos(thetaNew * Math.PI / 180);
                    currentY += movementRight * Math.Sin(thetaNew * Math.PI / 180);
                }
                else if (e.KeyCode == Keys.A)
                {
                    movementLeft = -1;
                    movementRight = 1;
                    double d = (movementLeft + movementRight) / 2.0;
                    double deltaTheta = (movementLeft - movementRight) / _robot.WheelBase;

                    if (Math.Abs(deltaTheta) < 1e-6)
                    {
                        currentX += d * Math.Cos(thetaNew);
                        currentY += d * Math.Sin(thetaNew);
                    }
                    else
                    {
                        currentX += (d / deltaTheta) * (Math.Sin(thetaNew + deltaTheta) - Math.Sin(thetaNew));
                        currentY -= (d / deltaTheta) * (Math.Cos(thetaNew + deltaTheta) - Math.Cos(thetaNew));
                    }
                    thetaNew += deltaTheta * 180 / Math.PI;
                    thetaNew = NormalizeAngleDeg(thetaNew);
                }
                else if (e.KeyCode == Keys.S)
                {
                    movementLeft = -2;
                    movementRight = -2;
                    currentX += movementLeft * Math.Cos(thetaNew * Math.PI / 180);
                    currentY += movementRight * Math.Sin(thetaNew * Math.PI / 180);
                }
                else if (e.KeyCode == Keys.D)
                {
                    movementLeft = 1;
                    movementRight = -1;
                    double d = (movementLeft + movementRight) / 2.0;
                    double deltaTheta = (movementLeft - movementRight) / _robot.WheelBase;

                    if (Math.Abs(deltaTheta) < 1e-6)
                    {
                        currentX += d * Math.Cos(thetaNew);
                        currentY += d * Math.Sin(thetaNew);
                    }
                    else
                    {
                        currentX += (d / deltaTheta) * (Math.Sin(thetaNew + deltaTheta) - Math.Sin(thetaNew));
                        currentY -= (d / deltaTheta) * (Math.Cos(thetaNew + deltaTheta) - Math.Cos(thetaNew));
                    }
                    thetaNew += deltaTheta * 180 / Math.PI;
                    thetaNew = NormalizeAngleDeg(thetaNew);

                }




                //double leftSensor = (float)MonteCarloLocal.DrawRaycast(currentX, currentY, thetaNew + GlobalConstants.DegreeOffsetLeft, 350, _MCL_grid,custom,Color.Green);
                //double midSensor = (float)MonteCarloLocal.DrawRaycast(currentX, currentY, thetaNew, 350, _MCL_grid,custom,Color.Red);
                //double rightSensor = (float)MonteCarloLocal.DrawRaycast(currentX, currentY, thetaNew + GlobalConstants.DegreeOffsetRight, 350, _MCL_grid,custom, Color.Blue);


                //double leftSensor = (float)MonteCarloLocal.Raycast(currentX, currentY, thetaNew + GlobalConstants.DegreeOffsetLeft, 350, _MCL_grid);
                //double midSensor = (float)MonteCarloLocal.Raycast(currentX, currentY, thetaNew, 350, _MCL_grid);
                //double rightSensor = (float)MonteCarloLocal.Raycast(currentX, currentY, thetaNew + GlobalConstants.DegreeOffsetRight, 350, _MCL_grid);

                //  addToTextBox($"Real are {leftSensor}|{midSensor}|{rightSensor}{Environment.NewLine}");
               // addToTextBox($"Theta from code {thetaNew} vs theta from estimation {MonteLocalization.GetEstimatedPos().Theta} {Environment.NewLine}");


                double leftSensor  = GetPredictedDistance(currentX, currentY, thetaNew, GlobalConstants.DegreeOffsetLeft, GlobalConstants.LeftSensorOffsets, Color.Green);
                double midSensor = GetPredictedDistance(currentX, currentY, thetaNew, GlobalConstants.DegreeOffsetMid, GlobalConstants.MidSensorOffsets, Color.Red);
                double rightSensor = GetPredictedDistance(currentX, currentY, thetaNew, GlobalConstants.DegreeOffsetRight, GlobalConstants.RightSensorOffsets, Color.Blue);

                var message = new
                {
                    direction = thetaNew,
                    leftMovement = movementLeft * 10,
                    rightMovement = movementRight * 10,
                    leftSensor = SaltTheNumber(leftSensor),
                    rightSensor = SaltTheNumber(rightSensor),
                    midSensor = SaltTheNumber(midSensor),
                    handSensor = 0,
                    mappingFlag = 0
                };
                await PublishJsonMessageAsync("DataForMapping", message, 0);

                custom.SetPixel((int)currentX, (int)currentY, Color.Black);
                RefreshPicture();
            }
            else if (currentlyMappingSimulation)
            {
                //simulatedActualRobot
                if (e.KeyCode == Keys.W)
                {
                    keyCurrentlyPressedForSim = "w";
                }
                else if (e.KeyCode == Keys.A)
                {
                    keyCurrentlyPressedForSim = "a";
                }
                else if (e.KeyCode == Keys.S)
                {
                    keyCurrentlyPressedForSim = "s";

                }
                else if (e.KeyCode == Keys.D)
                {
                    keyCurrentlyPressedForSim = "d";
                }


            }
            else if (currentlyControlling && currentlyPressedKey == Keys.None && lastSentKey != e.KeyCode)
            {
                currentlyPressedKey = e.KeyCode;
                lastSentKey = e.KeyCode;
                //WriteDataSingular(currentlyPressedKey.ToString());
                var message = new
                {
                    stopSignal = "false",
                    manualCommand = currentlyPressedKey.ToString()
                };

                await PublishJsonMessageAsync("Movement", message, 2);


                //if (e.KeyCode == Keys.W)
                //{
                //    movementLeft = 2;
                //    movementRight = 2;
                //    currentX += 2 * Math.Cos(_robot.getThetaActual() * Math.PI / 180);
                //    currentY += 2 * Math.Sin(_robot.getThetaActual() * Math.PI / 180);
                //}
                //else if (e.KeyCode == Keys.A)
                //{
                //    movementLeft = 2;
                //    movementRight = 0;
                //    double d = (movementLeft + movementRight) / 2.0;
                //    double deltaTheta = (movementRight - movementLeft) / _robot.WheelBase;
                //    double theta = _robot.getThetaActual() * Math.PI / 180;
                //    _robot.setThetaActual(_robot.getThetaActual() + deltaTheta * 180 / Math.PI);
                //    if (Math.Abs(deltaTheta) < 1e-6)
                //    {
                //        currentX += d * Math.Cos(theta);
                //        currentY += d * Math.Sin(theta);
                //    }
                //    else
                //    {
                //        currentX += (d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta));
                //        currentY -= (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta));
                //    }
                //}
                //else if (e.KeyCode == Keys.S)
                //{
                //    movementLeft = -2;
                //    movementRight = -2;
                //    currentX -= 2 * Math.Cos(_robot.getThetaActual() * Math.PI / 180);
                //    currentY -= 2 * Math.Sin(_robot.getThetaActual() * Math.PI / 180);
                //}
                //else if (e.KeyCode == Keys.D)
                //{
                //    movementLeft = 0;
                //    movementRight = 2;
                //    double d = (movementLeft + movementRight) / 2.0;
                //    double deltaTheta = (movementRight - movementLeft) / _robot.WheelBase;
                //    double theta = _robot.getThetaActual() * Math.PI / 180;
                //    _robot.setThetaActual(_robot.getThetaActual() + deltaTheta * 180 / Math.PI);
                //    if (Math.Abs(deltaTheta) < 1e-6)
                //    {
                //        currentX += d * Math.Cos(theta);
                //        currentY += d * Math.Sin(theta);

                //    }
                //    else
                //    {
                //        currentX += (d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta));
                //        currentY -= (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta));
                //    }

                //}


                //currentRotation = _robot.getThetaActual();
                //double left = currentRotation - 90;
                //double right = currentRotation + 90;
                //if (left > 360)
                //{
                //    left -= 360;
                //}else if (left < 0)
                //{
                //    left += 360;
                //}
                //if (right > 360)
                //{
                //    right -= 360;
                //}
                //else if (right < 0)
                //{
                //    right += 360;
                //}
                //var message = new
                //{
                //    direction = currentRotation,
                //    leftMovement = movementLeft * 10,
                //    rightMovement = movementRight * 10,
                //    leftSensor = (float)MonteCarloLocal.Raycast(currentX, currentY, left, 400, _MCL_grid),
                //    rightSensor = (float)MonteCarloLocal.Raycast(currentX, currentY, right, 400, _MCL_grid),
                //    midSensor = (float)MonteCarloLocal.Raycast(currentX, currentY, currentRotation, 400, _MCL_grid),
                //    handSensor = 0,
                //    mappingFlag = 0
                //};
                //await PublishJsonMessageAsync("DataForMapping", message, 2);

            }
            else if (!currentlyControlling)
            {
                if (e.KeyCode == Keys.W)
                {
                    movementLeft = 2;
                    movementRight = 2;
                    currentX += 2 * Math.Cos(_robot.getThetaActual() * Math.PI / 180);
                    currentY += 2 * Math.Sin(_robot.getThetaActual() * Math.PI / 180);
                }
                else if (e.KeyCode == Keys.A)
                {
                    movementLeft = 2;
                    movementRight = 0;
                    double d = (movementLeft + movementRight) / 2.0;
                    double deltaTheta = (movementRight - movementLeft) / _robot.WheelBase;
                    double theta = _robot.getThetaActual() * Math.PI / 180;
                    _robot.setThetaActual(_robot.getThetaActual() + deltaTheta * 180 / Math.PI);
                    if (Math.Abs(deltaTheta) < 1e-6)
                    {
                        currentX += d * Math.Cos(theta);
                        currentY += d * Math.Sin(theta);
                    }
                    else
                    {
                        currentX += (d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta));
                        currentY -= (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta));
                    }
                }
                else if (e.KeyCode == Keys.S)
                {
                    movementLeft = -2;
                    movementRight = -2;
                    currentX -= 2 * Math.Cos(_robot.getThetaActual() * Math.PI / 180);
                    currentY -= 2 * Math.Sin(_robot.getThetaActual() * Math.PI / 180);
                }
                else if (e.KeyCode == Keys.D)
                {
                    movementLeft = 0;
                    movementRight = 2;
                    double d = (movementLeft + movementRight) / 2.0;
                    double deltaTheta = (movementRight - movementLeft) / _robot.WheelBase;
                    double theta = _robot.getThetaActual() * Math.PI / 180;
                    _robot.setThetaActual(_robot.getThetaActual() + deltaTheta * 180 / Math.PI);
                    if (Math.Abs(deltaTheta) < 1e-6)
                    {
                        currentX += d * Math.Cos(theta);
                        currentY += d * Math.Sin(theta);

                    }
                    else
                    {
                        currentX += (d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta));
                        currentY -= (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta));
                    }

                }


                currentRotation = _robot.getThetaActual();
                if (currentRotation < 0)
                {
                    currentRotation = 360 + currentRotation;
                }


                double deltaThetat = (movementRight - movementLeft) / _robot.WheelBase;
                await MonteLocalization.StartTasksToMoveParticles((movementLeft + movementRight) / 2, currentRotation, (deltaThetat * 180 / Math.PI)/*currentRotation*/);


                //   await MonteLocalization.StartTasksToMoveParticles((movementLeft + movementRight) / 2, (float)currentRotation);
                addToTextBox(currentRotation + Environment.NewLine);
                //await MonteLocalization.StartTasksToUpdateWeights(
                //        [MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, GlobalConstants.DegreeOffsetMid, GlobalConstants.MidDegrees, GlobalConstants.MidSensorOffsets, _MCL_grid),
                //                   MonteLocalization.GetPredictedDistance(currentX, currentY,currentRotation, GlobalConstants.DegreeOffsetLeft, GlobalConstants.LeftDegrees, GlobalConstants.LeftSensorOffsets, _MCL_grid),
                //                    MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, GlobalConstants.DegreeOffsetRight, GlobalConstants.RightDegrees, GlobalConstants.RightSensorOffsets, _MCL_grid)]);
                //      await MonteLocalization.StartTasksToUpdateWeights(
                //[MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, 0,0,(0,0), _MCL_grid),
                //                         MonteLocalization.GetPredictedDistance(currentX, currentY,currentRotation, -90,0,(0,0), _MCL_grid),
                //                          MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, 90,0,(0,0), _MCL_grid)],
                //75);


                // addToTextBox(MonteLocalization.totalWeightPublic.ToString() +""+ Environment.NewLine);
                var estimatedPos = MonteLocalization.GetEstimatedPos();
                double tempTheta = 0;
                if (estimatedPos.Theta > 0)
                {
                    tempTheta = estimatedPos.Theta;
                }
                else
                {
                    tempTheta = 360 + estimatedPos.Theta;
                }
                addToTextBox(tempTheta + "" + Environment.NewLine);
                // MonteLocalization.Resample(false, 0);
                MonteLocalization.ResampleGPT();
                try
                {
                    DrawParticles();
                    if (_grid.IsWalkable((int)currentX, (int)currentY) == true)
                    {
                        custom.SetPixel((int)currentX, (int)currentY, Color.Green);
                    }
                    if (_grid.IsWalkable((int)estimatedPos.X, (int)estimatedPos.Y) == true)
                    {
                        custom.SetPixel((int)estimatedPos.X, (int)estimatedPos.Y, Color.Red);
                    }
                    txtBoxWeight.Text = estimatedPos.Theta.ToString();
                    PictureBox.Invalidate();
                    /* foreach (var item in MonteLocalization.Particles)
                     {
                     addToTextBox($"Point: { item.X} + {item.Y} ");

                     }*/
                }
                catch (Exception)
                {

                    throw;
                }



            }

        }
        public void DrawParticles()
        {
            foreach (var item in MonteLocalization.Particles)
            {
                if (_grid.IsWalkable((int)item.X, (int)item.Y) == true || _grid.IsWalkable((int)item.X, (int)item.Y) == false)
                  custom.SetPixel((int)item.X, (int)item.Y, Color.Blue);
               // {
                    //if (item.Theta < 90)
                    //{
                    //    custom.SetPixel((int)item.X, (int)item.Y, Color.Blue);

                    //}
                    //else if (item.Theta < 180)
                    //{
                    //    custom.SetPixel((int)item.X, (int)item.Y, Color.Red);
                    //}
                    //else if (item.Theta < 270)
                    //{
                    //    custom.SetPixel((int)item.X, (int)item.Y, Color.Purple);
                    //}
                    //else
                    //{
                    //    custom.SetPixel((int)item.X, (int)item.Y, Color.Green);
                    //}
                //}
            }
        }
        private async void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            if (currentlyMappingSimulation)
            {
                keyCurrentlyPressedForSim = "n";
            }
            if (currentlyControlling && currentlyPressedKey != Keys.None)
            {
                lastSentKey = Keys.Enter;
                currentlyPressedKey = Keys.None;
                //WriteDataSingular(currentlyPressedKey.ToString());
                var message = new
                {
                    stopSignal = "true",
                    manualCommand = currentlyPressedKey.ToString()
                };

                await PublishJsonMessageAsync("Movement", message, 1);
            }
        }
        private void ControlRobotLogic()
        {
            if (currentlyControlling)
            {
                currentlyControlling = false;
                StopInterpreting();
            }
            else
            {
                currentlyControlling = true;
                StartInterpreting();
            }
        }
        private void btn_ControlRobot_Click(object sender, EventArgs e)
        {
            ControlRobotLogic();
        }
        private void btn_ConnectionButton_Click(object sender, EventArgs e)
        {
            // ConnectionButton();
        }
        public bool currentlyMapping = false;
        private void CreateNewImage()
        {

            custom = new CustomBitmap(3500, 3500);
            for (int counter = 0; counter < custom.Width * custom.Height; counter++)
            {
                custom.WriteToBits(counter, Color.FromArgb(255, 0, 0, 0));

            }
            _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
            PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);
            interpreter = new Interpreter(ref stringsToBeInterpreted, ref custom, ref currentX, ref currentY, ref currentRotation, this, 63);
            currentlyMapping = true;
            mclIsON = false;
            // RefreshPicture();
        }
        private void btn_CreateNewImage_Click(object sender, EventArgs e)
        {
            CreateNewImage();
        }
        private void ClosingProcedure()
        {
            StopInterpreting();
        }
        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            ClosingProcedure();
        }
        private void SetObstacles()
        {
            _grid = SetObstaclesFromMap(custom);
            if (customMCL != null)
                _MCL_grid = SetObstaclesFromMap(customMCL);
        }
        private void button1_Click(object sender, EventArgs e)
        {
            SetObstacles();
        }
        private void PlanPath()
        {

            var start = new Node(startX, startY);
            var goal = new Node(endX, endY);



            var thetaStar = new ThetaStar(_grid);
            var path = thetaStar.FindPath(start, goal);

            if (path != null)
            {
                finalPath = path;
                _robot.ExecutePath(path);
                Node previous = null;//start;

                foreach (var item in path)
                {
                    if (previous != null)
                    {

                        txtBox_TextOutput.AppendText($"From ({previous.X},{previous.Y}) to ({item.X},{item.Y}) : {Math.Truncate(FindTurnDegree(previous, item) * 10) / 10}degrees | {Math.Truncate(thetaStar.Heuristic(previous, item) * 10) / 10}cm \n ");  // which is {FindTurnDegree(previous, item)} \n ");
                        turns.Add(Math.Truncate(FindTurnDegree(previous, item) * 10) / 10);
                        segments.Add(Math.Truncate(thetaStar.Heuristic(previous, item) * 10) / 10);
                    }
                    previous = item;
                }
            }
            else
            {
                MessageBox.Show("No path");
            }

            RefreshPicture();
        }
        private void btn_ManualRotation_Click(object sender, EventArgs e)
        {
            startX = (int)MonteLocalization.GetEstimatedPos().X;
            startY = (int)MonteLocalization.GetEstimatedPos().Y;
            PlanPath();

        }
        private double findOppositeSide(double adjacent, double theta)
        {
            // Convert angle from degrees to radians
            double thetaRad = (90 - theta) * Math.PI / 180.0;
            // Calculate the opposite side
            double opposite = adjacent * Math.Tan(thetaRad);
            return opposite;
        }
        private double TheThetaWeWant(double sideOne, double sideTwo)
        {

            if (sideOne < sideTwo)
            {
                turnsss.Add(true);
                return (180 - (sideTwo - sideOne)) / 2;
            }
            else
            {
                turnsss.Add(false); // lqvo
                return (180 - (sideOne - sideTwo)) / 2;
            }
        }
        private List<object> goodPath()
        {
            List<object> res = new List<object>();
            for (int i = 0; i < segments.Count; i++)
            {
                if (i + 1 < segments.Count)
                {
                    var TURN = TheThetaWeWant(turns[i], turns[i + 1]);
                    var advance = findOppositeSide(16.5, TURN); // WTF is 16.5?!? ohh it should be the distance between the wheel and center for rotation calcs
                    segments[i] = segments[i] - advance;
                    segments[i + 1] = segments[i + 1] - advance;

                    var message = new
                    {
                        stopSignal = "false",
                        move = segments[i] * 10,
                        turn = TURN * (turnsss[i] == true ? -1 : 1)
                    };
                    res.Add(message);
                    // res.Add("moveForward~" + (segments[i] * 10).ToString());
                    // res.Add($"turn~{TURN * (turnsss[i] == true ? -1 : 1)}");
                }
                else
                {
                    var message = new
                    {
                        stopSignal = "false",
                        move = segments[i] * 10
                    };
                    res.Add(message);
                }
            }
            return res;
        }
        private double FindTurnDegree(Node start, Node next)
        {
            double baseDegree = 0;
            double sideOne = Math.Abs(start.X - next.X);
            double sideTwo = Math.Abs(start.Y - next.Y);
            if (start.X > next.X)
            {
                if (start.Y < next.Y)
                {
                    baseDegree = 180;
                    return baseDegree + (90 - (Math.Atan(sideTwo / sideOne) * (180 / Math.PI)));
                }
                else if (start.Y > next.Y)
                {
                    baseDegree = 270;
                    return baseDegree + (Math.Atan(sideTwo / sideOne) * (180 / Math.PI));
                }
                else
                {
                    baseDegree = 270;
                }
            }
            else if (start.X < next.X)
            {
                if (start.Y < next.Y)
                {
                    baseDegree = 90;
                    return baseDegree + (Math.Atan(sideTwo / sideOne) * (180 / Math.PI));
                }
                else if (start.Y > next.Y)
                {
                    baseDegree = 0;
                    return baseDegree + (90 - (Math.Atan(sideTwo / sideOne) * (180 / Math.PI)));
                }
                else
                {
                    baseDegree = 90;
                }
            }
            else
            {
                if (start.Y < next.Y)
                {
                    baseDegree = 180;
                }
                else if (start.Y > next.Y)
                {
                    baseDegree = 0;
                }
                else
                {
                    baseDegree = -1;
                }
            }

            return baseDegree;
        }
        private void SetStart()
        {
            settingStart = true;
            settingEnd = false;
        }
        private void btn_SetStart_Click(object sender, EventArgs e)
        {
            SetStart();
        }
        private void SetEnd()
        {
            settingEnd = true;
            settingStart = false;
        }
        private void btn_SetEnd_Click(object sender, EventArgs e)
        {
            SetEnd();
        }
       
        public static double NormalizeAngle(double angle)
        {
            angle = (angle + Math.PI) % (2 * Math.PI);
            if (angle < 0)
                angle += 2 * Math.PI;
            return angle - Math.PI;
        }
        public double CalculateSteeringAngle(double x, double y, double theta, Node lookaheadPoint)
        {
            double dx = lookaheadPoint.X - x;
            double dy = -(lookaheadPoint.Y - y);


            double angleToTarget = Math.Atan2(dy, dx);
            double heading = (360 - theta) * Math.PI / 180.0;

            double steeringAngle = NormalizeAngle(angleToTarget - heading);
            addToTextBox("Steering to: " + steeringAngle + Environment.NewLine);




            values_angleToTarget_SteeringAngle.Add((
                angleToTarget * 180 / Math.PI,
                  steeringAngle,
                steeringAngle * 180 / Math.PI,
                 theta, heading
               ));

            try
            {
                custom.SetPixel((int)lookaheadPoint.X, (int)lookaheadPoint.Y, Color.Cyan);

            }
            catch (Exception)
            {

                throw;
            }
            return steeringAngle;
        }


        private bool turning = false;
        private Queue<double> steeringAngleHistory = new Queue<double>();
        private const int smoothingWindow = 2; // Adjust as needed
        private const double turningThreshold = 0.4; // Base threshold
        private const double hysteresisMargin = 0.2; // Extra margin to prevent flipping

        public readonly object _lockRightDistance = new object();
        public readonly object _lockLeftDistance = new object();

        public double rightDistance = 0;
        public double leftDistance = 0;
        private double GetSmoothedSteeringAngle(double newSteeringAngle)
        {
            steeringAngleHistory.Enqueue(newSteeringAngle);
            if (steeringAngleHistory.Count > smoothingWindow)
                steeringAngleHistory.Dequeue();

            return steeringAngleHistory.Average();
        }
        public void SetWheelVelocities(Robot robot, double rawSteeringAngle, double baseVelocity, double lookahead)
        {

            double steeringAngle = GetSmoothedSteeringAngle(rawSteeringAngle);
            double newLeftValue = 0;
            double newRightValue = 0;
            double radius = 0;

            if (Math.Abs(steeringAngle) < 0.2)
            {
                newLeftValue = baseVelocity;
                newRightValue = baseVelocity;
            }
            else
            {
                radius = lookahead / (2 * Math.Sin(steeringAngle));
                newLeftValue = baseVelocity * (1 - (robot.WheelBase / (2 * radius)));
                newRightValue = baseVelocity * (1 + (robot.WheelBase / (2 * radius)));
            }
           

            if (!turning && Math.Abs(steeringAngle) > turningThreshold + hysteresisMargin)
            {
                turning = true;
            }
            else if (turning && Math.Abs(steeringAngle) < turningThreshold - hysteresisMargin)
            {
                turning = false;

            }
            // values.Add((newLeftValue, newRightValue, 0, 0));
            if (turning)
            {
                if (newLeftValue < newRightValue)
                {
                    newLeftValue = -0.4;
                    newRightValue = 0.4;
                }
                else
                {
                    newLeftValue = 0.4;
                    newRightValue = -0.4;
                }


                //if (newRightValue < newRightValue)
                //{
                //    newRightValue = -0.4;
                //}
                //else
                //{
                //    newRightValue = 0.4;
                //}
            }
            else
            {

                lock (_lockRightDistance) lock (_lockLeftDistance)
                    {

                        if (leftDistance < 15 && rightDistance < 15)
                        {

                            if (leftDistance > rightDistance)
                            {
                                //newRightValue = 0.05;
                                //newLeftValue = -0.4;
                                newRightValue *= 1.2;
                                newLeftValue *= 0.7;
                            }
                            else
                            {
                                //newRightValue = -0.4;
                                //newLeftValue = 0.05;
                                newRightValue *= 0.7;
                                newLeftValue *= 1.2;
                            }

                        }
                        else if (leftDistance < 12) // left is too close 
                        {
                            //newRightValue = -0.4;
                            //newLeftValue = 0.05;

                            //newRightValue *= 0.2;
                            //newLeftValue *= 1.2;
                            newRightValue = -0.6;
                            newLeftValue = 0.25;

                        }
                        else if (rightDistance < 12)
                        {
                            //newRightValue = 0.05;
                            //newLeftValue = -0.4;

                            //newRightValue *= 1.2;
                            //newLeftValue *= 0.2;
                            newRightValue = 0.25;
                            newLeftValue = -0.6;



                        }
                    }
            }
            // Set final velocities
            robot.LeftWheelVelocity =newLeftValue;
            robot.RightWheelVelocity = newRightValue; 

            values.Add((robot.LeftWheelVelocity, robot.RightWheelVelocity, radius, steeringAngle));
        }


        public (double X, double Y, double Theta) estimatedPos;

        public KalmanFilter kalmanLeft = new KalmanFilter(0.7f, 15, 7.5f, 4, 3, 10);
        public KalmanFilter kalmanRight = new KalmanFilter(0.7f, 15, 7.5f, 4, 3, 10);
        public KalmanFilter kalmanMid = new KalmanFilter(0.7f, 15, 7.5f, 4, 3, 10);

    
        public List<(double, double)> tempAngles = new List<(double, double)>();
        public bool stopPurePursuitFLAG = false;
        public async Task PurePursuitControlAdaptive(Robot robot, List<Node> path, double maxLookahead, double baseVelocity, double dt)
        {
            Node currentGoal = path[0];
           // RequestDataFromBot();
            int firstInstance = 1;
            robot.currentLeftWheelVelocity = 0;
            robot.currentRightWheelVelocity = 0;
            double tempTheta = 0;
            Stopwatch sw = Stopwatch.StartNew();
            Stopwatch testsw = Stopwatch.StartNew();
            do
            {
                if (testsw .ElapsedMilliseconds > 320/*newInfoForAutoMovement_FLAG*/)  // if simulating switch it to the timer
               {

                    newInfoForAutoMovement_FLAG = false;

                    estimatedPos = MonteLocalization.GetEstimatedPos();

                    robot._currentX = estimatedPos.X;
                    robot._currentY = estimatedPos.Y;

                    tempTheta = estimatedPos.Theta;
                    addToTextBox("Temp yep : " + tempTheta + Environment.NewLine);
                    if (tempTheta < 0)
                        tempTheta += 360;

                    robot.setThetaActual(tempTheta);

                    var SmallGoal_BigGoal = FindLookaheadPoint(estimatedPos.X, estimatedPos.Y, currentGoal, path, maxLookahead);
                    currentGoal = SmallGoal_BigGoal.Item2;
                    var steeringAngle = CalculateSteeringAngle(estimatedPos.X, estimatedPos.Y, tempTheta, SmallGoal_BigGoal.Item1);
                    SetWheelVelocities(robot, steeringAngle, baseVelocity, maxLookahead);

                    double rightVelo = robot.RightWheelVelocity * 7;  // if simulating better mutiply these by like 5 or 10 otherwise it will be too slow real time
                    double leftVelo = robot.LeftWheelVelocity * 7;
                    if (sw.ElapsedMilliseconds > 150)
                    {
                      
                        if (firstInstance == 1)
                        {
                            var message = new
                            {
                                firstInstance = firstInstance,
                                stopSignal = "false",
                                rightVelocity = rightVelo,
                                leftVelocity = leftVelo
                            };
                            PublishJsonMessageAsync("LeftRightSpeed", message, 2);
                            firstInstance = 0;
                        }
                        else
                        {
                            var message = new
                            {
                                stopSignal = "false",
                                rightVelocity = rightVelo,
                                leftVelocity = leftVelo
                            };
                            PublishJsonMessageAsync("LeftRightSpeed", message, 0);
                        }
                     

                        if (simMovementInMapButtonIsPressed)
                        {

                            var movementLeft = leftVelo * dt;
                           var  movementRight = rightVelo * dt;
                            double d = (movementLeft + movementRight) / 2.0;
                            double deltaTheta = (movementLeft - movementRight) / _robot.WheelBase;

                            if (Math.Abs(deltaTheta) < 1e-6)
                            {
                                currentX += d * Math.Cos(thetaNew*Math.PI/180);
                                currentY += d * Math.Sin(thetaNew * Math.PI / 180);
                            }
                            else
                            {
                                currentX += (d / deltaTheta) * (Math.Sin(thetaNew * Math.PI / 180 + deltaTheta) - Math.Sin(thetaNew * Math.PI / 180));
                                currentY -= (d / deltaTheta) * (Math.Cos(thetaNew * Math.PI / 180 + deltaTheta) - Math.Cos(thetaNew * Math.PI / 180));
                            }
                            thetaNew += deltaTheta * 180 / Math.PI;
                            thetaNew = NormalizeAngleDeg(thetaNew);

                            custom.SetPixel((int)currentX, (int)currentY, Color.Black);

                            double leftSensor = GetPredictedDistance(currentX, currentY, thetaNew, GlobalConstants.DegreeOffsetLeft, GlobalConstants.LeftSensorOffsets, Color.Green);
                            double midSensor = GetPredictedDistance(currentX, currentY, thetaNew, GlobalConstants.DegreeOffsetMid, GlobalConstants.MidSensorOffsets, Color.Red);
                            double rightSensor = GetPredictedDistance(currentX, currentY, thetaNew, GlobalConstants.DegreeOffsetRight, GlobalConstants.RightSensorOffsets, Color.Blue);

                            var message = new
                            {
                                direction = thetaNew,
                                leftMovement = robot.LeftWheelVelocity * dt * 10,//movementLeft * 10,
                                rightMovement = robot.RightWheelVelocity * dt * 10,// movementRight * 10,
                                leftSensor = SaltTheNumber(leftSensor),
                                rightSensor = SaltTheNumber(rightSensor),
                                midSensor = SaltTheNumber(midSensor),
                                handSensor = 0,
                                mappingFlag = 0
                            };
                            PublishJsonMessageAsync("DataForMapping", message, 0);
                            SafeUpdate(RefreshPicture);
                            //   RefreshPicture();
                            //   newInfoForAutoMovement_FLAG = true;
                        }  
                        
                        sw.Restart();
                        testsw.Restart();
                    }
                }

            } while (((Math.Abs(estimatedPos.X - currentGoal.X) + Math.Abs(estimatedPos.Y - currentGoal.Y)) > 15 || currentGoal != path.Last()) && !stopPurePursuitFLAG);

            var messagelast = new
            {

                stopSignal = "true",
                rightVelocity = 0,
                leftVelocity = 0

            };
            await PublishJsonMessageAsync("LeftRightSpeed", messagelast, 2);

            SafeUpdate(() => RefreshPicture());
            startX = (int)estimatedPos.X;
            startY = (int)estimatedPos.Y;


        }
        public (Node, Node) FindLookaheadPoint(double x, double y, Node nextPoint, List<Node> path, double lookaheadDistance)
        {

            double dx = nextPoint.X - x;
            double dy = nextPoint.Y - y;
            double segmentLength = Math.Sqrt(dx * dx + dy * dy);
            var next = nextPoint;
            while (segmentLength <= lookaheadDistance)
            {
                if (next == path.Last())
                {
                    return (next, next);
                }
                else
                {
                    dx += path[path.IndexOf(next) + 1].X - next.X;
                    dy += path[path.IndexOf(next) + 1].Y - next.Y;
                    segmentLength = Math.Sqrt(dx * dx + dy * dy);
                    next = path[path.IndexOf(next) + 1];
                }
            }

            double interpolationFactor = lookaheadDistance / segmentLength;
            double lookaheadX = x + interpolationFactor * dx;
            double lookaheadY = y + interpolationFactor * dy;

            return (new Node((int)lookaheadX, (int)lookaheadY), next);
        }

        private async Task ExecutePlan()
        {
            txtBox_TextOutput.AppendText($"Execute Plan Pressed\n");
            if (simMovementInMapButtonIsPressed)
            {
                newInfoForAutoMovement_FLAG = true;
            }
            else
            {
            stopPurePursuitFLAG = false; 

            }
            Task purePursuitTask = new Task(() => PurePursuitControlAdaptive(_robot, finalPath, 30, 0.4, 1));
            purePursuitTask.Start();
            //await PurePursuitControlAdaptive(_robot, finalPath, 20, 1, 0.5);
            //  RefreshPicture();

        }
        private async void btn_ExecuteRoute_Click(object sender, EventArgs e)
        {
            ExecutePlan();
          

        }
        MqttClientOptions mqttClientOptions = new MqttClientOptionsBuilder().WithWebSocketServer(o => o.WithUri("ws://localhost:9001/mqtt"))
                     .WithClientId("ServerClient")
                     .WithCleanSession()
                    .Build();
        private async void InitMQTTClient()
        {
            if (imagePublisherClient == null)
            {
                imagePublisherClient = mqttFactory.CreateMqttClient();
                imagePublisherClient.ApplicationMessageReceivedAsync += delegate (MqttApplicationMessageReceivedEventArgs args)
                {
                    // THIS is called in some bullshit thread or something so we need to fix it
                    string topic = args.ApplicationMessage.Topic;
                    var str = Encoding.UTF8.GetString(args.ApplicationMessage.Payload);

                    switch (topic)
                    {
                        case "DataForMapping":
                            HandleTopicMap(str);
                            break;
                        case "Movement":
                            HandleTopicTwo(str);
                            break;
                        case "CaliberationMagData":
                            HandleTopicThree(str);
                            break;
                        case "ServoPosConfirm":
                            // HandleTopicThree(str);
                            break;
                        case "location/commands":
                            HandleTopicLocationCommands(str);
                            break;
                        case "location/markers":
                            HandleTopicLocationMarker(str);
                            break;
                        default:
                            addToTextBox($"Unknown topic: {topic}");
                            break;
                    }

                    return Task.CompletedTask;
                };

                imagePublisherClient.ConnectedAsync += async e =>
                {
                    addToTextBox($"Connected to MQTT broker\n");
                };

                await imagePublisherClient.ConnectAsync(mqttClientOptions);

                imagePublisherClient.DisconnectedAsync += async e =>
                {

                    await ReconnectToBrokerAsync();
                };


                await imagePublisherClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("ServoPosConfirm").Build());
                await imagePublisherClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("DataForMapping").Build());
                await imagePublisherClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("CaliberationMagData").Build());
                await imagePublisherClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("Movement").Build());
                await imagePublisherClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("location/commands").Build());
                await imagePublisherClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("location/markers").Build());

            }
        }
        public void addToTextBox(string messageToAdd)
        {
            SafeUpdate(() => txtBox_TextOutput.AppendText(messageToAdd));
        }

        private void SafeUpdate(Action action)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(action);
            }
            else
            {
                action();
            }
        }
        private async Task ReconnectToBrokerAsync()
        {
            while (!imagePublisherClient.IsConnected)
            {
                try
                {
                    await imagePublisherClient.ConnectAsync(mqttClientOptions);
                }
                catch
                {
                    await Task.Delay(5000);  // Wait 5 seconds before retrying
                }
            }
        }
        private bool mclIsON = false;
        private async void HandleTopicMap(string payload)
        {
            JsonMessageClass? message = JsonConvert.DeserializeObject<JsonMessageClass>(payload);
            if (message != null)
            {
                if (message.mappingFlag == 1 && !mclIsON && currentlyMapping)
                {
                    stringsToBeInterpreted.Enqueue(("mapPoint", message));
                    feedback.Add(message);
                }
                else if (mclIsON && !currentlyMapping)
                {

                    stringsToBeInterpreted.Enqueue(("MCL", message));
                    feedback.Add(message);
                }
            }
        }

        private void HandleTopicTwo(string payload)
        {
            addToTextBox($"{payload}\n");

        }

        private void HandleTopicThree(string payload)
        {
            magDataMessage? message = JsonConvert.DeserializeObject<magDataMessage>(payload);
            if (message != null)
            {
                stringsToBeInterpreted.Enqueue(("calib", message));
                testAgain.Add(message);
            }
        }

        private void HandleTopicLocationMarker(string payload)
        {
            JsonMarkerMessage? message = JsonConvert.DeserializeObject<JsonMarkerMessage>(payload);
            if (message != null)
            {
                startX = message.start.x;
                startY = message.start.y;
                endX = message.end.x;
                endY = message.end.y;
                if (_robot == null)
                {
                    _robot = new Robot(_MCL_grid, custom, startX, startY, this, 63);
                }
                if (MonteLocalization == null)
                {
                    MonteLocalization = new MonteCarloLocal(25, startX, startY, 50, 5, _MCL_grid);// _grid is old
                    currentX = startX;
                    currentY = startY;
                }
                addToTextBox($"Added points {message.start.x}, {message.start.y} and {message.end.x}, {message.end.y}");
            }

        }
        private void HandleTopicLocationCommands(string payload)
        {

            JsonLocationCommandsClass? message = JsonConvert.DeserializeObject<JsonLocationCommandsClass>(payload);



            if (message != null)
            {

                if (message.command == "plan_route")
                {
                    SafeUpdate(PlanPath);
                    SafeUpdate(PictureBox.Invalidate);
                }
                if (message.command == "execute_route")
                {
                    SafeUpdate(() => ExecutePlan());
                }
                if (message.command == "load_map")
                {
                    SafeUpdate(LoadDefaultImage);
                    SafeUpdate(SetObstacles);
                }
                if (message.command == "image_request")
                {
                    PublishImageChunks();
                }
                if (message.command == "W" || message.command == "A" || message.command == "S" || message.command == "D" || message.command == "N")
                {
                    if (currentlyMappingSimulation)
                    {
                        keyCurrentlyPressedForSim = message.command.ToLower();

                    }
                    else
                    {
                        var jsonMessage = new
                        {
                            stopSignal = "false",
                            manualCommand = message.command
                        };
                        addToTextBox(message.command + Environment.NewLine);
                        PublishJsonMessageAsync("Movement", jsonMessage, 2);
                    }

                }
                if (message.command == "start_simulation")
                {
                    BeginSimulation();
                }






                if (message.command == "stop")
                {
                    var jsonMessage = new
                    {
                        stopSignal = "true",
                        manualCommand = "None"
                    };
                    PublishJsonMessageAsync("Movement", jsonMessage, 1);

                }

            }
            /* if (payload != null)
             {
                 if (payload == "plan_route")
                 {
                     addToTextBox("we pressed planRoute" + Environment.NewLine);
                     PlanPath();
                 }
                 if (payload == "execute_route")
                 {
                     ExecutePlan();
                     addToTextBox("we did indeed press execute yay" + Environment.NewLine);
                 }

                 if (payload == "W" || payload == "A" || payload == "S" || payload == "D")
                 {
                     var jsonMessage = new
                     {
                         stopSignal = "false",
                         manualCommand = payload
                     };
                     addToTextBox(payload + Environment.NewLine);
                     PublishJsonMessageAsync("Movement", jsonMessage, 2);

                 }
                 if (payload == "stop")
                 {
                     var jsonMessage = new
                     {
                         stopSignal = "true",
                         manualCommand = "None"
                     };
                     PublishJsonMessageAsync("Movement", jsonMessage, 1);

                 }

             }*/
            // addToTextBox(payload + Environment.NewLine);
        }
        public List<object> messages = new List<object>();
        private async Task PublishJsonMessageAsync(string topic, object message, int qosLevel)
        {
            var payload = JsonConvert.SerializeObject(message);
            messages.Add(payload);
            var mqttMessage = new MqttApplicationMessageBuilder()
                .WithTopic(topic)  // Specify the topic
                .WithPayload(Encoding.UTF8.GetBytes(payload))
                .WithQualityOfServiceLevel(qosLevel == 0 ? MQTTnet.Protocol.MqttQualityOfServiceLevel.AtMostOnce : qosLevel == 1 ? MQTTnet.Protocol.MqttQualityOfServiceLevel.AtLeastOnce : qosLevel == 2 ? MQTTnet.Protocol.MqttQualityOfServiceLevel.ExactlyOnce : throw new Exception())
                .WithRetainFlag(false)
                .Build();

            await imagePublisherClient.PublishAsync(mqttMessage);
        }


        private async void PublishImageChunks()
        {
            byte[] imageBytes = ImageToByte(custom.Bitmap);
            int chunkSize = 500 * 1024;
            int numberOfChunks = (int)Math.Ceiling((double)imageBytes.Length / chunkSize);

            for (int i = 0; i < numberOfChunks; i++)
            {
                int currentChunkSize = Math.Min(chunkSize, imageBytes.Length - i * chunkSize);
                byte[] chunk = new byte[currentChunkSize];
                Array.Copy(imageBytes, i * chunkSize, chunk, 0, currentChunkSize);

                var message = new
                {
                    chunkNumber = i,
                    totalChunks = numberOfChunks,
                    data = chunk
                };

                await PublishJsonMessageAsync("location/image", message, 1);
            }
        }

        public static byte[] ImageToByte(Image img)
        {
            using (var stream = new MemoryStream())
            {
                img.Save(stream, System.Drawing.Imaging.ImageFormat.Png);
                return stream.ToArray();
            }
        }

        private void btnSetServo_Click(object sender, EventArgs e)
        {
            //  int servoAngle = int.Parse(txtBoxServo.Text);
            // WriteData($"servo~{servoAngle}~");
            LoadDefaultImage();


        }
        private void button2_Click(object sender, EventArgs e)
        {

        }
        bool isTakingMagData = false;
        private async void btnCalib_Click(object sender, EventArgs e)
        {
            if (isTakingMagData)
            {
                MessageBox.Show("Stopping Collection");
                stringsToBeInterpreted.Enqueue(("endCalib", 0));
            }
            else
            {
                var message = new
                {
                    calMag = 1

                };
                await PublishJsonMessageAsync("Movement", message, 2);

                isTakingMagData = true;
                MessageBox.Show("We are collecting data.");
            }
        }
        private void btnRelayTest_Click(object sender, EventArgs e)
        {
            // WriteData($"relay~");
            SaveDefaultImage();
        }
        private void button3_Click(object sender, EventArgs e)
        {
            //  RequestDataFromBot();
            //  KalmanTest();
            SendMarkerForTesting();
        }
        private void SendMarkerForTesting()
        {
            var message = new
            {
                markerX = startX,
                makerY = startY
            };
            PublishJsonMessageAsync("location/robot", message, 2);
        }


        private async void btnArmDown_Click(object sender, EventArgs e)
        {
            var message = new
            {
                stopServos = false,
                posArmOne = 90
                //  stopServos = false,
                //  answer = true,
                //  wantedDirection = 90
            };

            await PublishJsonMessageAsync("HandServoControl", message, 2);
            // await PublishJsonMessageAsync("wantedDirection", message);
        }

        private async void btnArmUp_Click(object sender, EventArgs e)
        {
            var message = new
            {
                stopServos = false,
                posArmOne = 0
            };

            await PublishJsonMessageAsync("HandServoControl", message, 2);
        }

        private async void btnGrab_Click(object sender, EventArgs e)
        {
            interpreter = new Interpreter(ref stringsToBeInterpreted, ref custom, ref currentX, ref currentY, ref currentRotation, this, 63);
            //var message = new
            //{
            //    stopServos = false,
            //    posArmTwo = 150
            //};

            //await PublishJsonMessageAsync("HandServoControl", message, 2);

            currentlyMapping = false;
            mclIsON = true;
        }

        private async void btnRelease_Click(object sender, EventArgs e)
        {
            var message = new
            {
                stopServos = false,
                posArmTwo = 130
            };

            await PublishJsonMessageAsync("HandServoControl", message, 2);
        }

        private void btnExplode_Click(object sender, EventArgs e)
        {

        }

        private void btnSendImage_Click(object sender, EventArgs e)
        {
            PublishImageChunks();
        }

        private void btnLoadTwo_Click(object sender, EventArgs e)
        {
            MultiLoadLoadImageAsMap(null);
        }

        public volatile string keyCurrentlyPressedForSim = "n";

        public List<string> wtfIsThisDoing = new List<string>();
        private async void ControllingTask()
        {
            double movementLeft = 0;
            double movementRight = 0;
            while (currentlyMappingSimulation)
            {
                movementLeft = 0;
                movementRight = 0;
                if (keyCurrentlyPressedForSim == "w")
                {
                    movementLeft = 2;
                    movementRight = 2;
                    simulatedActualRobot._currentX += 2 * Math.Cos(simulatedActualRobot.getThetaActual() * Math.PI / 180);
                    simulatedActualRobot._currentY += 2 * Math.Sin(simulatedActualRobot.getThetaActual() * Math.PI / 180);

                    addToTextBox($"Current: {simulatedActualRobot.getThetaActual()} {Environment.NewLine}");
                }
                else if (keyCurrentlyPressedForSim == "a")
                {
                    //movementLeft = 2;
                    //movementRight = 0;
                    //double d = (movementLeft + movementRight) / 2.0;  
                    //double deltaTheta = (movementRight - movementLeft) / simulatedActualRobot.WheelBase;  
                    //double theta = simulatedActualRobot.getThetaActual() * Math.PI / 180;
                    //simulatedActualRobot.setThetaActual(simulatedActualRobot.getThetaActual() + deltaTheta * 180 / Math.PI);
                    //if (Math.Abs(deltaTheta) < 1e-6)
                    //{
                    //    simulatedActualRobot._currentX += d * Math.Cos(theta);
                    //    simulatedActualRobot._currentY += d * Math.Sin(theta);


                    //}
                    //else  // If rotating, move along an arc
                    //{
                    //    simulatedActualRobot._currentX += (d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta));
                    //    simulatedActualRobot._currentY -= (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta));
                    //}



                    //double dL = 0;  
                    //double dR = 2;  

                    //double d = (dL + dR) / 2.0;  // Linear displacement

                    //// Get the current rotation from the magnetometer (already in degrees)
                    //double newTheta = currentRotation * Math.PI / 180;  // Convert to radians
                    //double deltaTheta = newTheta - (prevMagnetometerReading * Math.PI / 180);  // Compute actual heading change
                    //prevMagnetometerReading = currentRotation;  // Store current rotation for next iteration

                    //if (Math.Abs(deltaTheta) < 1e-6)  // If rotation is negligible, move straight
                    //{
                    //    currentX += d * Math.Cos(newTheta);
                    //    currentY += d * Math.Sin(newTheta);
                    //}
                    //else
                    //{
                    //    currentX -= (d / deltaTheta) * (Math.Sin(newTheta) - Math.Sin(newTheta - deltaTheta));
                    //    currentY += (d / deltaTheta) * (Math.Cos(newTheta - deltaTheta) - Math.Cos(newTheta));
                    //}




                    ////  addToTextBox($"Added: {(d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta))} | {(d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta))} + \n");
                    //addToTextBox($"Current: {simulatedActualRobot.getThetaActual()} {Environment.NewLine}");


                }
                else if (keyCurrentlyPressedForSim == "s")
                {
                    movementLeft = -2;
                    movementRight = -2;
                    simulatedActualRobot._currentX -= 2 * Math.Cos(simulatedActualRobot.getThetaActual() * Math.PI / 180);
                    simulatedActualRobot._currentY -= 2 * Math.Sin(simulatedActualRobot.getThetaActual() * Math.PI / 180);
                    addToTextBox($"Current: {simulatedActualRobot.getThetaActual()} {Environment.NewLine}");

                }
                else if (keyCurrentlyPressedForSim == "d")
                {
                    //simulatedActualRobot.setThetaActual(simulatedActualRobot.getThetaActual() + (1.5 + getRand() / 2));
                    //movementLeft = 0;
                    //movementRight = 2;
                    //double d = (movementLeft + movementRight) / 2.0;
                    //double deltaTheta = (movementRight - movementLeft) / simulatedActualRobot.WheelBase;
                    //double theta = simulatedActualRobot.getThetaActual() * Math.PI / 180;
                    //simulatedActualRobot.setThetaActual(simulatedActualRobot.getThetaActual() + deltaTheta * 180 / Math.PI);
                    //if (Math.Abs(deltaTheta) < 1e-6)
                    //{
                    //    simulatedActualRobot._currentX += d * Math.Cos(theta);
                    //    simulatedActualRobot._currentY += d * Math.Sin(theta);

                    //}
                    //else  
                    //{
                    //    simulatedActualRobot._currentX += (d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta));
                    //    simulatedActualRobot._currentY -= (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta));
                    //}




                    movementLeft = 0;
                    movementRight = 2;
                    double d = (movementLeft + movementRight) / 2.0;
                    double deltaTheta = (movementRight - movementLeft) / simulatedActualRobot.WheelBase;
                    double theta = simulatedActualRobot.getThetaActual() * Math.PI / 180;
                    simulatedActualRobot.setThetaActual(simulatedActualRobot.getThetaActual() + deltaTheta * 180 / Math.PI);
                    if (Math.Abs(deltaTheta) < 1e-6)
                    {
                        simulatedActualRobot._currentX += d * Math.Cos(theta);
                        simulatedActualRobot._currentY += d * Math.Sin(theta);

                    }
                    else
                    {
                        simulatedActualRobot._currentX += (d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta));
                        simulatedActualRobot._currentY -= (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta));
                    }

                    addToTextBox($"Current: {simulatedActualRobot.getThetaActual()} {Environment.NewLine}");

                    // addToTextBox($"Added: {string.Format("{0:0.00}",(d / deltaTheta) * (Math.Sin(theta + deltaTheta) - Math.Sin(theta)))} | {string.Format("{0:0.00}", (d / deltaTheta) * (Math.Cos(theta + deltaTheta) - Math.Cos(theta)))} + {Environment.NewLine}");
                }
                if (keyCurrentlyPressedForSim != "n")
                {
                    double left = (float)simulatedActualRobot.getThetaActual() - 90;
                    if (left < 0)
                    {
                        left += 360;
                    }
                    double right = (float)simulatedActualRobot.getThetaActual() + 90;
                    if (right > 360)
                    {
                        right -= 360;
                    }


                    var jsonMessage = new
                    {
                        direction = (float)simulatedActualRobot.getThetaActual(),
                        leftMovement = movementLeft * 10,
                        rightMovement = movementRight * 10,
                        leftSensor = (float)MonteCarloLocal.Raycast(simulatedActualRobot._currentX, simulatedActualRobot._currentY, left, 400, simulatedMapGrid),
                        rightSensor = (float)MonteCarloLocal.Raycast(simulatedActualRobot._currentX, simulatedActualRobot._currentY, right, 400, simulatedMapGrid),
                        midSensor = (float)MonteCarloLocal.Raycast(simulatedActualRobot._currentX, simulatedActualRobot._currentY, (float)simulatedActualRobot.getThetaActual(), 400, simulatedMapGrid),
                        handSensor = 0,
                        mappingFlag = 1
                    };
                    //SafeUpdate(() => { txtBoxWeight.Text = simulatedActualRobot.getThetaActual().ToString(); });
                    // txtBoxWeight.Text = simulatedActualRobot.getThetaActual().ToString();
                    addToTextBox($"Current: {simulatedActualRobot.getThetaActual()}| {left} | {right} {Environment.NewLine}");
                    movementLeft = 0;
                    movementRight = 0;
                    await PublishJsonMessageAsync("DataForMapping", jsonMessage, 1);
                }
                await Task.Delay(75);
            }

        }


        public void BeginSimulation()
        {
            CreateNewImage();
            simulatedMapArea = LoadImageAsCustomBitmap(actualAreaPath);
            simulatedMapGrid = SetObstaclesFromMap(simulatedMapArea);
            simulatedActualRobot = new Robot(simulatedMapGrid, simulatedMapArea, 150, 300, this, 63);
            currentlyMappingSimulation = true;
            simulatedActualRobot.setThetaActual(90);
            StartInterpreting();
            Task.Run(ControllingTask);

        }

        private void button4_Click(object sender, EventArgs e)
        {
            BeginSimulation();
        }

        private void btnSwitchView_Click(object sender, EventArgs e)
        {
            PictureBox.Invalidate();
            _imgRect = new Rectangle(picture_offsetX, picture_offsetY, simulatedMapArea.Width, simulatedMapArea.Height);
            PictureBox.CreateGraphics().DrawImage(simulatedMapArea.Bitmap, _imgRect);
            Thread.Sleep(2500);
        }

        private void btnScanView_Click(object sender, EventArgs e)
        {
            stopPurePursuitFLAG = true;
            var message = new
            {
                stopSignal = "true"
            };
            PublishJsonMessageAsync("LeftRightSpeed", message, 2);
        }

        private int testing_firstInstance = 1;
        private int testing_lastClicked = 0; // 0 none clicked, 1 forward clicked, 2 backward clicked
        private int countTimesMessageSent = 0;
        private async void btnSingleMotorSpeedTestForward_Click(object sender, EventArgs e)
        {
            var message = new
            {
                stopSignal = "false",
                firstInstance = testing_firstInstance,
                leftVelocity = 1.0,
                rightVelocity = 1.0
            };
            if (testing_lastClicked == 1)
            {
                message = new
                {
                    stopSignal = "false",
                    firstInstance = testing_firstInstance,
                    leftVelocity = 1.0,
                    rightVelocity = -1.0
                };
                await PublishJsonMessageAsync("LeftRightSpeed", message, 2);
                testing_lastClicked = 0;
            }
            else
            {
                if (testing_firstInstance == 1)
                {
                    countTimesMessageSent++;
                    await PublishJsonMessageAsync("LeftRightSpeed", message, 2);
                    testing_firstInstance = 0;
                    testing_lastClicked = 1;
                }
                else
                {
                    await PublishJsonMessageAsync("LeftRightSpeed", message, 2);
                    testing_lastClicked = 1;
                }
            }

        }

        private async void btnSingleMotorTestBackward_Click(object sender, EventArgs e)
        {
            var message = new
            {
                stopSignal = "false",
                firstInstance = testing_firstInstance,
                leftVelocity = -1.0,
                rightVelocity = 2.0
            };
            if (testing_lastClicked == 2)
            {
                message = new
                {
                    stopSignal = "false",
                    firstInstance = testing_firstInstance,
                    leftVelocity = 1.0,
                    rightVelocity = 2.0
                };
                await PublishJsonMessageAsync("LeftRightSpeed", message, 2);
                testing_lastClicked = 0;

            }
            else
            {


                if (testing_firstInstance == 1)
                {
                    await PublishJsonMessageAsync("LeftRightSpeed", message, 2);
                    testing_firstInstance = 0;
                    testing_lastClicked = 2;
                }
                else
                {
                    await PublishJsonMessageAsync("LeftRightSpeed", message, 0);
                    testing_lastClicked = 2;
                }
            }
        }
        private bool simMovementInMapButtonIsPressed = false;
        private void btnSimMovementManual_Click(object sender, EventArgs e)
        {
           simMovementInMapButtonIsPressed = !simMovementInMapButtonIsPressed;
        }
    }
}