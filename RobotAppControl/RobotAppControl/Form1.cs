using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Drawing.Imaging;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Windows.Forms;
using System.Xml.Linq;
using MQTTnet;
using MQTTnet.Client;
using System.Collections;
using MQTTnet.Server;
using Newtonsoft.Json;
using Microsoft.VisualBasic;
using static System.Windows.Forms.AxHost;
using System.Globalization;

namespace RobotAppControl
{
    public partial class Form1 : Form
    {
        public Grid _grid;
        public Grid _MCL_grid;
        private Robot _robot = null;

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
        Listener? listener;
        Interpreter? interpreter;
        Thread listenThread;
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
        private int startX = 0;
        private int startY = 0;
        private int endX = 0;
        private int endY = 0;
        private List<Node> finalPath = null;
        private MqttFactory mqttFactory = new MqttFactory();
        private bool newInfoForAutoMovement_FLAG = false;
        ConcurrentQueue<string> rawMagDataToBeWorkedOn;
        List<double> segments = new List<double>();
        List<double> turns = new List<double>();
        List<bool> turnsss = new List<bool>();
        private IMqttClient? imagePublisherClient = null;
        private MonteCarloLocal MonteLocalization = null;
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
        private void StartListen()
        {
            if (listener == null)
            {
                // listener = new Listener(ref stringsToBeInterpreted, ref TotalRevievedStrings);
                //listenThread = new Thread(() => listener.BeginListening(stream));
                //  listenThread.Start();
            }
        }
        private Grid SetObstaclesFromMap(CustomBitmap map, Grid gridToBeSet)
        {
            gridToBeSet = new Grid(map.Width, map.Height);

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
                        for (int i = -8; i <= 8; i++)
                        {
                            for (int j = -8; j <= 8; j++)
                            {
                                if (gridToBeSet.IsWalkable(x + i, y + j) == true)
                                {
                                    float tentativeCost = 1 + (20 - Math.Abs(i) - Math.Abs(j)) / 2;
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
        public List<string> ConvertPathToCommands(List<Node> path)
        {
            var commands = new List<string>();

            for (int i = 0; i < path.Count - 1; i++)
            {
                int dx = path[i + 1].X - path[i].X;
                int dy = path[i + 1].Y - path[i].Y;

                if (dx != 0)
                {
                    commands.Add(dx > 0 ? "turn~0~" : "turn~180~");
                    commands.Add($"moveForward~{Math.Abs(dx)}~");
                }
                else if (dy != 0)
                {
                    commands.Add(dy > 0 ? "turn~90~" : "turn~-90~");
                    commands.Add($"moveForward~{Math.Abs(dy)}~");
                }
            }

            return commands;
        }
        public void ExecutePath(List<string> commannds)
        {
            foreach (var command in commannds)
            {
                WriteData(command);
            }
        }
        private bool IsObstacle(Color color)
        {
            // Define your criteria for an obstacle. For example, a pixel is an obstacle if it is black.
            return color.R + color.G + color.B < 100; // Example threshold for black
        }
        private void StopListening()
        {
            if (listener != null)
            {
                listener.Stop();
                listener = null;
            }
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
            custom = LoadImageAsCustomBitmap("Default");
            _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
            PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);
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
        private void MultiLoadLoadImageAsMap()
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
            while (counterForRequest < 30)
            {
                if (sw.ElapsedMilliseconds > 200)
                {
                    var message = new
                    {
                        requestMapData = 1
                    };
                    counterForRequest++;
                    await PublishJsonMessageAsync("MapDataRequest", message, 1);
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
                MonteLocalization = new MonteCarloLocal(750, coordinates.X - picture_offsetX, coordinates.Y - picture_offsetY, 20,5, _grid);// _grid is old
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
                    _robot = new Robot(_grid, custom, startX, startY, this, 16.5);
                }
                settingStart = false;
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
                }

                fs.Close();
            }
        }
        private void btn_SaveImg_Click(object sender, EventArgs e)
        {
            ImageSave();

        }
        private void HandleAdjacentPixels(int iCentr, int jCentr, int spread, Graphics graphics, Dictionary<Tuple<int, int>, int> affectedCells)
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
                            if ((currentAdj.R + currentAdj.G + currentAdj.B) >= 10)
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
                sfd.Filter = "JPeg Image|*.jpg|Bitmap Image|*.bmp|Gif Image|*.gif";
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
                        HandleAdjacentPixels(i, j, 2, g, AffectedCells); // 2 or 3 should work fine?
                    }
                }
                foreach (var item in AffectedCells)
                {
                    if (item.Value > 5)
                    {
                        rectangle = new Rectangle(item.Key.Item1, item.Key.Item2, 1, 1);
                        rectangle.Inflate(size, size);    // adds x to the size in EACH direction so 25 * 2  with 30 it looks a bit too much but logic says it should be more correct?
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
                    }

                    fs.Close();

                }
            }
            if (is_MCL_map == false)
            {
                ConvertToOccMap(3, true);
            }
        }

        private void StartConvertingToOcccupancyThread()
        {
            Task task = new Task(() => ConvertToOccMap(25, false));
            task.Start();
        }
        private void btn_ConvertLoadedToOccupancyGrid_Click(object sender, EventArgs e)
        {
            StartConvertingToOcccupancyThread();
        }

        void WriteData(String message) // needs update
        {
            try
            {
                //Byte[] data = System.Text.Encoding.ASCII.GetBytes($"{message}~");
                if (CheckConnection())
                {
                    //stream.Write(data, 0, data.Length);
                }
            }
            catch (ArgumentNullException e)
            {
                Console.WriteLine("ArgumentNullException: {0}", e);
            }
            catch (SocketException e)
            {
                Console.WriteLine("SocketException: {0}", e);
            }
        }
        Keys lastSentKey = Keys.Enter;
        int counter = 0;
        int movementVal = 0;
        private async void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (currentlyControlling && currentlyPressedKey == Keys.None && lastSentKey != e.KeyCode)
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
            }
            else if (!currentlyControlling)
            {
                if (e.KeyCode == Keys.W)
                {
                    if (MonteLocalization != null)
                    {
                        currentRotation = _robot.getThetaActual();// _robot.getThetaVisual();
                        movementVal = 1;
                      //  MonteLocalization.StartTasksToMoveParticles(1, (float)currentRotation);
                        currentX += 1 * Math.Cos(currentRotation * Math.PI / 180);
                        currentY += 1 * Math.Sin(currentRotation * Math.PI / 180);
                        counter++;
                    }
                }
                else if (e.KeyCode == Keys.A)
                {
                    _robot.setThetaActual(_robot.getThetaActual() - 1.5);
                    currentRotation = _robot.getThetaActual();// _robot.getThetaVisual();
                    movementVal = 0;
                   // MonteLocalization.StartTasksToMoveParticles(0, (float)currentRotation);
                    txtBoxServo.Text = currentRotation.ToString();
                }
                else if (e.KeyCode == Keys.S)
                {
                    if (MonteLocalization != null)
                    {
                        currentRotation = _robot.getThetaActual();// _robot.getThetaVisual();
                        movementVal = -1;
                      //  MonteLocalization.StartTasksToMoveParticles(-1, (float)currentRotation);
                        currentX -= 1 * Math.Cos(currentRotation * Math.PI / 180);
                        currentY -= 1 * Math.Sin(currentRotation * Math.PI / 180);
                        counter++;
                    }
                }
                else if (e.KeyCode == Keys.D)
                {
                    _robot.setThetaActual(_robot.getThetaActual() + 1.5);
                    currentRotation = _robot.getThetaActual();// _robot.getThetaVisual();
                    movementVal = 0;
                 //   MonteLocalization.StartTasksToMoveParticles(0, (float)currentRotation);
                    txtBoxServo.Text = currentRotation.ToString();
                }

                //  currentRotation = _robot.getThetaVisual();
                await MonteLocalization.StartTasksToMoveParticles(movementVal, (float)currentRotation);
                await MonteLocalization.StartTasksToUpdateWeights(
                        [MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, 0, _grid), //_MCL_grid
                                   MonteLocalization.GetPredictedDistance(currentX, currentY,currentRotation, -90, _grid),
                                    MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, 90, _grid)],
                        2);
                addToTextBox(MonteLocalization.totalWeightPublic.ToString() + Environment.NewLine);
                    MonteLocalization.Resample();

                /*    MonteLocalization.UpdateWeightsOld(
                        [MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, 0, _grid),
                                   MonteLocalization.GetPredictedDistance(currentX, currentY,currentRotation, -90, _grid),
                                    MonteLocalization.GetPredictedDistance(currentX, currentY, currentRotation, 90, _grid)],
                        2);*/
                    var estimatedPos = MonteLocalization.EstimatePosition();
                    try
                    {
                    //    DrawParticles();
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
        private void DrawParticles()
        {
            foreach (var item in MonteLocalization.Particles)
            {
                if (_grid.IsWalkable((int)item.X, (int)item.Y) == true)
                {
                    custom.SetPixel((int)item.X, (int)item.Y, Color.Blue);
                }
            }
        }
        private async void Form1_KeyUp(object sender, KeyEventArgs e)
        {
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
                StopListening();
                StopInterpreting();
            }
            else
            {
                currentlyControlling = true;
                StartListen();
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
        private void CreateNewImage()
        {
            custom = new CustomBitmap(5000, 5000);
            for (int counter = 0; counter < custom.Width * custom.Height; counter++)
            {
                custom.WriteToBits(counter, Color.FromArgb(255, 0, 0, 0));

            }
            _imgRect = new Rectangle(picture_offsetX, picture_offsetY, custom.Width, custom.Height);
            PictureBox.CreateGraphics().DrawImage(custom.Bitmap, _imgRect);
            interpreter = new Interpreter(ref stringsToBeInterpreted, ref custom, ref currentX, ref currentY, ref currentRotation, this);
            // RefreshPicture();
        }
        private void btn_CreateNewImage_Click(object sender, EventArgs e)
        {
            CreateNewImage();
        }
        private void ClosingProcedure()
        {
            StopListening();
            StopInterpreting();
        }
        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            ClosingProcedure();
        }
        private void SetObstacles()
        {
            _grid = SetObstaclesFromMap(custom, _grid);
            // _MCL_grid = SetObstaclesFromMap(customMCL, _MCL_grid);
        }
        private void button1_Click(object sender, EventArgs e)
        {
            SetObstacles();
        }
        private void PlanPath()
        {
            // Define start and goal positions
            //   var start = new NewNode(startX, startY);
            //   var goal = new NewNode(endX, endY);
            var start = new Node(startX, startY);
            var goal = new Node(endX, endY);
            // Find path using A* algorithm
            //   var aStar = new AStar(_grid);
            //   var path = aStar.FindPath(start, goal);


            var thetaStar = new ThetaStar(_grid);
            //  var thetaStar = new NewThetaStar(_grid);
            var path = thetaStar.FindPath(start, goal);
            //  var path = thetaStar.FindPath(start,goal);
            // Execute path with robot
            if (path != null)
            {
                finalPath = path;
                _robot.ExecutePath(path);
                Node previous = null;//start;
                txtBox_TextOutput.Clear();
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
                //   txtBox_TextOutput.AppendText($"From ({previous.X},{previous.Y}) to ({goal.X},{goal.Y}) which is {FindTurnDegree(previous, goal)} \n ");
            }
            else
            {
                MessageBox.Show("No path");
            }

            // Refresh the picture box to show the path
            RefreshPicture();
        }
        private void btn_ManualRotation_Click(object sender, EventArgs e)
        {
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
        private List<string> CookedPath(List<Node> list)
        {
            List<string> result = new List<string>();
            int movedx = 0;
            int movedy = 0;

            for (int i = 0; i < list.Count - 1; i++)
            {
                int dx = list[i + 1].X - list[i].X;
                int dy = list[i + 1].Y - list[i].Y;


                if (dx != 0)
                {
                    if (movedy != 0)
                    {
                        result.Add("moveForward~" + (Math.Abs(movedy) * 10).ToString() + "~");
                        result.Add(dx > 0 ? "turn~90~" : "turn~-90~");
                        movedx = 0;
                        movedy = 0;
                    }
                    movedx += dx;
                }
                else if (dy != 0)
                {
                    if (movedx != 0)
                    {
                        result.Add("moveForward~" + (Math.Abs(movedx) * 10).ToString() + "~");
                        result.Add(dy > 0 ? "turn~90~" : "turn~-90~");
                        movedx = 0;
                        movedy = 0;
                    }
                    movedy += dy;
                }
            }
            result.Add("moveForward~" + (Math.Abs(movedy + movedx) * 10).ToString() + "~");
            return result;
        }
        public double CalculateSteeringAngle(Robot robot, Node lookaheadPoint)
        {
            double dx = lookaheadPoint.X - robot._currentX;
            double dy = lookaheadPoint.Y - robot._currentY;
            double angleToTarget = Math.Atan2(dy, dx);// * 180/Math.PI;
            double angleinDegree = angleToTarget * 180 / Math.PI;
            double steeringAngle = angleToTarget - robot.getThetaVisual() * Math.PI / 180;
            return steeringAngle;
        }
        public void SetWheelVelocities(Robot robot, double steeringAngle, double baseVelocity)
        {
            double radius = robot.WheelBase / (2 * Math.Sin(steeringAngle));
            robot.LeftWheelVelocity = baseVelocity * (1 - robot.WheelBase / (2 * radius));
            robot.RightWheelVelocity = baseVelocity * (1 + robot.WheelBase / (2 * radius));
            var message = new
            {
                stopSignal = "false",
                leftVelocity = robot.LeftWheelVelocity,
                rightSpeed = robot.RightWheelVelocity
            };
            //   PublishJsonMessageAsync("LeftRightSpeed", message);
        }
        private (double X, double Y, double Theta) estimatedPos;
        public void UpdatePosition(Robot robot, double dt)
        {
            double v = (robot.LeftWheelVelocity + robot.RightWheelVelocity) / 2.0;
            double omega = (robot.RightWheelVelocity - robot.LeftWheelVelocity) / robot.WheelBase;

            //robot._currentX += (int)(v * Math.Cos(robot.getThetaVisual() * Math.PI / 180) * dt);
            //robot._currentY += (int)(v * Math.Sin(robot.getThetaVisual() * Math.PI / 180) * dt);

            estimatedPos = MonteLocalization.EstimatePosition();
            robot._currentX = (int)estimatedPos.X;
            robot._currentY = (int)estimatedPos.Y;
            robot.setThetaActual(estimatedPos.Theta);

            currentX = robot._currentX;
            currentY = robot._currentY;
            currentRotation = robot.getThetaVisual();


            try
            {
                //DrawParticles();
                if (_grid.IsWalkable((int)currentX, (int)currentY) == true)
                {
                    custom.SetPixel((int)currentX, (int)currentY, Color.Green);
                }
                if (_grid.IsWalkable((int)estimatedPos.X, (int)estimatedPos.Y) == true)
                {
                    custom.SetPixel((int)estimatedPos.X, (int)estimatedPos.Y, Color.Red);
                }
            }
            catch (Exception)
            {

                throw;
            }


        }
        public List<(double, double)> tempAngles = new List<(double, double)>();
        public async void PurePursuitControlAdaptive(Robot robot, List<Node> path, double maxLookahead, double baseVelocity, double dt)
        {
            RequestDataFromBot();
            Node currentGoal = path[0];
            Stopwatch sw = Stopwatch.StartNew();
            int firstInstance = 1;
            do
            {

                //  double lookaheadDistance = CalculateAdaptiveLookahead(robot, nextWaypoint, minLookahead, maxLookahead, thresholdDistance);
                //    if (newInfoForAutoMovement_FLAG)
                //    {
                // newInfoForAutoMovement_FLAG = false;
                var SmallGoal_BigGoal = FindLookaheadPoint(robot, currentGoal, path, maxLookahead);
                currentGoal = SmallGoal_BigGoal.Item2;
                var steeringAngle = CalculateSteeringAngle(robot, SmallGoal_BigGoal.Item1);
                SetWheelVelocities(robot, steeringAngle, baseVelocity);
                //tempAngles.Add((robot.LeftWheelVelocity, robot.RightWheelVelocity));
                // UpdatePosition(robot, dt);
                var message = new
                {
                    stopSignal = "false",
                    firstInstance = firstInstance,
                    leftVelocity = robot.LeftWheelVelocity,
                    rightSpeed = robot.RightWheelVelocity
                };
                // SafeUpdate(()=>DrawParticles());
                /* SafeUpdate(() =>
                 {
                     if (_grid.IsWalkable((int)estimatedPos.X, (int)estimatedPos.Y) == true)
                     {
                         custom.SetPixel((int)estimatedPos.X, (int)estimatedPos.Y, Color.Red);
                     }
                 });*/
                if (sw.ElapsedMilliseconds > 750)
                {
                    if (firstInstance == 1)
                    {
                        await PublishJsonMessageAsync("LeftRightSpeed", message, 2);
                        firstInstance = 0;
                    }
                    else
                    {
                        PublishJsonMessageAsync("LeftRightSpeed", message, 0);
                    }
                    // addToTextBox($"Velocity: {robot.LeftWheelVelocity}/{robot.RightWheelVelocity}");
                    sw.Restart();
                }

                // }

            } while ((Math.Abs(estimatedPos.X - currentGoal.X) + Math.Abs(estimatedPos.Y - currentGoal.Y)) > 10 || currentGoal != path.Last());


        }
        public (Node, Node) FindLookaheadPoint(Robot robot, Node nextPoint, List<Node> path, double lookaheadDistance)
        {
            // Calculate the vector from the current point to the next point
            double dx = nextPoint.X - robot._currentX;
            double dy = nextPoint.Y - robot._currentY;
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

            // Interpolate along the segment to create a virtual lookahead point
            double interpolationFactor = lookaheadDistance / segmentLength;
            double lookaheadX = robot._currentX + interpolationFactor * dx;
            double lookaheadY = robot._currentY + interpolationFactor * dy;

            return (new Node((int)lookaheadX, (int)lookaheadY), next);
        }

        private async void ExecutePlan()
        {
            // ExecutePath(CookedPath(finalPath));
            txtBox_TextOutput.AppendText($"Execute Plan Pressed\n");

            Task purePursuitTask = new Task(() => PurePursuitControlAdaptive(_robot, finalPath, 10, 1, 2));
            purePursuitTask.Start();
            //PurePursuitControlAdaptive(_robot, finalPath, 10, 1, 2);
            /*
            var output = goodPath();
            foreach (var item in output)
            {
                await PublishJsonMessageAsync("Movement", item);
            } */
        }
        private void btn_ExecuteRoute_Click(object sender, EventArgs e)
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
                    // Do some work with the message...
                    string topic = args.ApplicationMessage.Topic;
                    string str = Encoding.UTF8.GetString(args.ApplicationMessage.PayloadSegment);

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
                            HandleTopicThree(str);
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
        private void HandleTopicMap(string payload)
        {
            JsonMessageClass? message = JsonConvert.DeserializeObject<JsonMessageClass>(payload);
            if (message != null)
            {
                if (message.mappingFlag == 1)
                {
                    stringsToBeInterpreted.Enqueue(("mapPoint", message));
                }
                else
                {
                    // _robot.setThetaActual(message.direction);
                    // currentRotation = message.direction;
                    MonteLocalization.StartTasksToMoveParticles(message.movement, message.direction);
                    MonteLocalization.StartTasksToUpdateWeights([message.midSensor, message.leftSensor, message.rightSensor], 4);
                      MonteLocalization.Resample();
                     MonteLocalization.StartTasksToUpdateWeights([message.midSensor, message.leftSensor, message.rightSensor], 4);
                   // addToTextBox($"<{message.leftSensor} {message.midSensor} {message.rightSensor} | {message.handSensor}> \n");
                }
                TotalRevievedStrings.Enqueue((message.leftSensor, message.rightSensor, message.midSensor));
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
            }
        }

        private void HandleTopicLocationMarker(string payload)
        {
            // meh get the info and put markers
        }
        private void HandleTopicLocationCommands(string payload)
        {

            JsonLocationCommandsClass? message = JsonConvert.DeserializeObject<JsonLocationCommandsClass>(payload);
            if (message != null)
            {
                if (message.command == "W" || message.command == "A" || message.command == "S" || message.command == "D")
                {
                    var jsonMessage = new
                    {
                        stopSignal = "false",
                        manualCommand = message.command
                    };
                    PublishJsonMessageAsync("Movement", jsonMessage, 2);

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
        }
        private async Task PublishJsonMessageAsync(string topic, object message, int qosLevel)
        {
            var payload = JsonConvert.SerializeObject(message);
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
            KalmanTest();
        }
        private void KalmanTest()
        {
            Random random = new Random();
            KalmanFilter kalmanFilter = new KalmanFilter(0.2f,15,7.5f,4,3,10);
            int currentBase = 10;
            int currentnumb = 0;
            int output = 0;
            int errCounter = 0;
            for (int i = 0; i < 500; i++)
            {
                if (random.NextDouble() > 0.55)
                    currentBase = random.Next(10, 250);
                currentnumb = random.Next(7) + currentBase;
                output = (int)kalmanFilter.Output(currentnumb);
                if(Math.Abs(currentnumb - output) >= 10)
                {

                 errCounter++;
                }
                addToTextBox("<" + (currentnumb - output)+"|" + currentnumb +"|"+output+ ">" + Environment.NewLine);
               // else
               // addToTextBox("<" +currentnumb + " | " +kalmanFilter.Output(currentnumb).ToString("F", CultureInfo.InvariantCulture) + ">");
            }
            addToTextBox("The number of times the error exceeded 10 was: " + errCounter);

        }

        private void btnStop_Click_1(object sender, EventArgs e)
        {

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
            var message = new
            {
                stopServos = false,
                posArmTwo = 150
            };

            await PublishJsonMessageAsync("HandServoControl", message, 2);
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
            MultiLoadLoadImageAsMap();
        }
    }
}