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

namespace RobotAppControl
{
    public partial class Form1 : Form
    {
        public Grid _grid;
        private Robot _robot;

        private PictureBox PictureBox;
        private int picture_offsetX = 0, picture_offsetY = 0;
        private int _xPos = 0;  // mouseposition at MouseDown
        private int _yPos = 0;  // mouseposition at MouseDown
        private bool _dragging = false;
        private Rectangle _imgRect = new Rectangle(0, 0, 0, 0);
        private CustomBitmap custom;
        private CustomBitmap occupancyMap;
        ConcurrentQueue<string> stringsToBeInterpreted;
        ConcurrentQueue<string> TotalRevievedStrings;
        TcpListener? server = null;
        Listener? listener;
        Interpreter? interpreter;
        Thread listenThread;
        Thread interpreterThread;
        NetworkStream stream;
        double currentX = 0;
        double currentY = 0;
        double currentRotation = 0;
        Keys currentlyPressedKey = Keys.None;
        private bool currentlyControlling = false;
        Int32 port = 13000;
        IPAddress localAddr = IPAddress.Parse("192.168.43.144");
        private Pen pen = new Pen(Brushes.Black);
        private Rectangle rectangle;
        TcpClient client;
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
        private IMqttClient mqttClient = null;
        ConcurrentQueue<string> rawMagDataToBeWorkedOn;
        List<double> segments = new List<double>();
        List<double> turns = new List<double>();
        List<bool> turnsss = new List<bool>();
        public Form1()
        {
            InitializeComponent();
            stringsToBeInterpreted = new ConcurrentQueue<string>();
            TotalRevievedStrings = new ConcurrentQueue<string>();
            rawMagDataToBeWorkedOn = new ConcurrentQueue<string>();
            PictureBox = this.pBox_Area;
            // this.KeyDown += new KeyEventHandler(Form1_KeyDown);
            this.KeyPreview = true;
            myDelagate = new RefreshTheImg(RefreshPicture);
            server = new TcpListener(localAddr, port);
            server.Start();

        }
        private void StartListen()
        {
            if (listener == null)
            {
                listener = new Listener(ref stringsToBeInterpreted, ref TotalRevievedStrings);
                listenThread = new Thread(() => listener.BeginListening(stream));
                listenThread.Start();
            }
        }
        private void SetObstaclesFromMap(CustomBitmap map)
        {
            _grid = new Grid(map.Width, map.Height);

            for (int x = 0; x < map.Width; x++)
            {
                for (int y = 0; y < map.Height; y++)
                {
                    Color pixelColor = map.GetPixel(x, y);
                    if (IsObstacle(pixelColor))
                    {
                        _grid.SetWalkable(x, y, false);
                    }
                }
            }
            for (int x = 0; x < map.Width; x++)
            {
                for (int y = 0; y < map.Height; y++)
                {
                    if (_grid.IsWalkable(x, y) == false)
                    {
                        for (int i = -8; i <= 8; i++)
                        {
                            for (int j = -8; j <= 8; j++)
                            {
                                if (_grid.IsWalkable(x + i, y + j) == true)
                                {
                                    float tentativeCost = 1 + (20 - Math.Abs(i) - Math.Abs(j)) / 2;
                                    if (_grid.GetCost(x + i, y + j) < tentativeCost)
                                    {
                                        _grid.SetCost(x + i, y + j, tentativeCost);
                                    }
                                }
                            }
                        }
                    }
                }
            }
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
        private CustomBitmap foolingAround;
        private void someTomefoolery()
        {
            foolingAround = new CustomBitmap(custom.Width, custom.Height);

            List<bool> trues = new List<bool>();
            List<bool> falses = new List<bool>();
            List<bool> truesAndHasRed = new List<bool>();

            for (int i = 0; i < foolingAround.Width; i++)
            {
                for (int j = 0; j < foolingAround.Height; j++)
                {
                    Color c = Color.FromArgb(_grid.GetCost(i, j) > 1 ? ((_grid.GetCost(i, j) * 20) > 255 ? 255 : (int)Math.Round(_grid.GetCost(i, j) * 20)) : 0, 0, 0);
                    foolingAround.SetPixel(i, j, c);
                    if (_grid.IsWalkable(i, j) == true)
                    {
                        trues.Add(true);
                        if (_grid.GetCost(i, j) > 1)
                        {
                            truesAndHasRed.Add(true);
                        }
                        foolingAround.SetPixel(i, j, c);
                    }
                    else
                    {
                        falses.Add(false);
                        foolingAround.SetPixel(i, j, Color.Blue);
                    }
                }
            }


            SaveFileDialog saveFileDialog1 = AskSaveFile();
            if (saveFileDialog1.FileName != "")
            {
                System.IO.FileStream fs =
                   (System.IO.FileStream)saveFileDialog1.OpenFile();
                switch (saveFileDialog1.FilterIndex)
                {
                    case 1:
                        foolingAround.Bitmap.Save(fs,
                          ImageFormat.Jpeg);
                        break;

                    case 2:
                        foolingAround.Bitmap.Save(fs,
                           ImageFormat.Bmp);
                        break;

                    case 3:
                        foolingAround.Bitmap.Save(fs,
                           ImageFormat.Gif);
                        break;
                }

                fs.Close();
            }

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
        }// TODO add MQTT call
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
            txtBox_TextOutput.AppendText($"Key pressed = {currentlyPressedKey}");

            if (settingStart)
            {
                startX = coordinates.X - picture_offsetX;
                startY = coordinates.Y - picture_offsetY;
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
            if (stream == null)
            {
                StopListening();
                StopInterpreting();
                return false;
            }
            return true;
            /*try  // no idea what i was doing here
            {
                // Socket socket = stream.Socket;
                //  bool answ = socket.Connected && !socket.Poll(1000, SelectMode.SelectRead);
                return true;
            }
            catch (Exception)
            {
                //  StopListening();
                //  StopInterpreting();
                return false;
            } */
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
        private void ConvertToOccMap()
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
                        HandleAdjacentPixels(i, j, 5, g, AffectedCells); // the third one is the spread value aka how many "rings" around the middle
                    }
                }
                foreach (var item in AffectedCells)
                {
                    if (item.Value > 10)
                    {
                        rectangle = new Rectangle(item.Key.Item1, item.Key.Item2, 1, 1);
                        rectangle.Inflate(2, 2);
                        g.FillRectangle(Brushes.Black, rectangle);


                        // occupancyMap.SetPixel(item.Key.Item1, item.Key.Item2, Color.Black);
                    }
                }
                SaveFileDialog saveFileDialog1 = AskSaveFile();
                if (saveFileDialog1.FileName != "")
                {
                    System.IO.FileStream fs =
                       (System.IO.FileStream)saveFileDialog1.OpenFile();
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
            MessageBox.Show("Success! Image converted and saved.");
        }
        private void StartConvertingToOcccupancyThread()
        {
            Thread thread = new Thread(() => ConvertToOccMap());
            thread.Start();
        }
        private void btn_ConvertLoadedToOccupancyGrid_Click(object sender, EventArgs e)
        {
            StartConvertingToOcccupancyThread();
        }
        void WriteDataSingular(String message)
        {
            try
            {
                Byte[] data = System.Text.Encoding.ASCII.GetBytes($"smurf~{message}~");
                if (CheckConnection())
                {
                    stream.Write(data, 0, data.Length);
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
        void WriteData(String message)
        {
            try
            {
                Byte[] data = System.Text.Encoding.ASCII.GetBytes($"{message}~");
                if (CheckConnection())
                {
                    stream.Write(data, 0, data.Length);
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
        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (currentlyControlling && currentlyPressedKey == Keys.None)
            {
                currentlyPressedKey = e.KeyCode;
                WriteDataSingular(currentlyPressedKey.ToString());

            }

        }
        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            if (currentlyControlling && currentlyPressedKey != Keys.None)
            {
                currentlyPressedKey = Keys.None;
                WriteDataSingular(currentlyPressedKey.ToString());

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
        private void ConnectionButton()
        {
            if (CheckConnection())
            {
                MessageBox.Show("Already connected???");
            }
            else
            {
                client = server.AcceptTcpClient();
                stream = client.GetStream();


                MessageBox.Show("we got here");
            }
        }
        private void btn_ConnectionButton_Click(object sender, EventArgs e)
        {
            ConnectionButton();
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
            SetObstaclesFromMap(custom);
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
            _robot = new Robot(_grid, custom, start.X, start.Y, this);
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
        private double TheThetaWeWant(double sideOne,double sideTwo)
        {
            
            if(sideOne < sideTwo)
            {
                turnsss.Add(true);
                return (180 - (sideTwo - sideOne))/2; 
            }
            else
            {
                turnsss.Add(false); // lqvo
                return (180 - (sideOne - sideTwo))/2;
            }
        }
        private List<string> goodPath()
        {
            List<string> res = new List<string>();
            for (int i = 0; i < segments.Count; i++)
            {
                if(i + 1 < segments.Count)
                {
                    var TURN = TheThetaWeWant(turns[i], turns[i + 1]);
                    var advance = findOppositeSide(16.5, TURN);
                    //if ()
                    //{

                   // }else
                    segments[i] = segments[i] - advance;
                    segments[i+1] = segments[i+1] - advance;
                    res.Add("moveForward~" + (segments[i] * 10).ToString());
                    res.Add($"turn~{TURN * (turnsss[i] == true ? -1 : 1)}");
                }
                else
                {
                    res.Add("moveForward~" + (segments[i]*10).ToString());
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
        private void ExecutePlan()
        {
            // ExecutePath(CookedPath(finalPath));
            var output = goodPath();
            foreach (var item in output)
            {
                WriteData(item);
            }
        }
        private void btn_ExecuteRoute_Click(object sender, EventArgs e)
        {
            ExecutePlan();
        }
        private int PWMSignalStrenght = 0;
        private void WritePWMTOScreen()
        {
            txtBox_TextOutput.Clear();
            txtBox_TextOutput.AppendText($"PWM = {PWMSignalStrenght}");
        }
        private void SendPWMdata()
        {
            WriteData($"pwm~{PWMSignalStrenght}~");
        }
        private void btnLessPWM_Click(object sender, EventArgs e)
        {
            if (PWMSignalStrenght > 0)
            {
                PWMSignalStrenght -= 25;
            }

            if (PWMSignalStrenght < 0)
            {
                PWMSignalStrenght = 0;
            }
            WritePWMTOScreen();
            SendPWMdata();
        }
        private void btnCheckCurrentPWM_Click(object sender, EventArgs e)
        {
            WritePWMTOScreen();
        }
        private void btnMorePWM_Click(object sender, EventArgs e)
        {
            if (PWMSignalStrenght < 250)
            {
                PWMSignalStrenght += 25;
            }

            if (PWMSignalStrenght > 250)
            {
                PWMSignalStrenght = 250;
            }
            WritePWMTOScreen();
            SendPWMdata();
        }
        private void btnDir_Click(object sender, EventArgs e)
        {
            WriteData("dir~");
        }
        private void btnStop_Click(object sender, EventArgs e)
        {

            WriteData("break~");
        }
        private async void btnMQTT_Click(object sender, EventArgs e)
        {
            using (mqttClient = mqttFactory.CreateMqttClient())
            {
                var mqttClientOptions = new MqttClientOptionsBuilder()
                    .WithClientId("Client1")
                    .WithTcpServer("-broker here-", 1883) // Use a public broker for demo purposes
                    .WithCleanSession()
                    .Build();

                mqttClient.ApplicationMessageReceivedAsync += e =>
                {
                    Console.WriteLine("Received application message.");
                    Console.WriteLine($"Topic: {e.ApplicationMessage.Topic}");
                    Console.WriteLine($"Payload: {Encoding.UTF8.GetString(e.ApplicationMessage.Payload)}");
                    Console.WriteLine($"QoS: {e.ApplicationMessage.QualityOfServiceLevel}");
                    Console.WriteLine($"Retain: {e.ApplicationMessage.Retain}");
                    Console.WriteLine();
                    return Task.CompletedTask;
                };

                mqttClient.ConnectedAsync += async e =>
                {
                    Console.WriteLine("Connected successfully with MQTT Brokers.");

                    // Subscribe to a topic
                    await mqttClient.SubscribeAsync(new MqttClientSubscribeOptionsBuilder()
                        .WithTopicFilter("testinTopic")
                        .Build());

                    Console.WriteLine("Subscribed to topic 'testinTopic'");
                };

                mqttClient.DisconnectedAsync += async e =>
                {
                    Console.WriteLine("Disconnected from MQTT Brokers.");
                    await Task.Delay(TimeSpan.FromSeconds(5));

                    try
                    {
                        await mqttClient.ConnectAsync(mqttClientOptions, CancellationToken.None); // Since 3.0.5 with CancellationToken
                    }
                    catch
                    {
                        Console.WriteLine("Reconnecting failed.");
                    }
                };


                try
                {
                    await mqttClient.ConnectAsync(mqttClientOptions, CancellationToken.None); // Since 3.0.5 with CancellationToken
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Connecting to MQTT Brokers failed: {ex.Message}");
                }

                Console.WriteLine("Press key to exit.");
                Console.ReadLine();
            }
        }
        private void btnSetServo_Click(object sender, EventArgs e)
        {
            int servoAngle = int.Parse(txtBoxServo.Text);
            WriteData($"servo~{servoAngle}~");
        }
        private void button2_Click(object sender, EventArgs e)
        {

        }
        bool isTakingMagData = false;
        private void btnCalib_Click(object sender, EventArgs e)
        {
            if (isTakingMagData)
            {
                stringsToBeInterpreted.Enqueue("endCalib|");
            }
            else
            {
                WriteData($"calMag~");
                isTakingMagData = true;
                MessageBox.Show("We are collecting data.");
            }
        }
        private void btnRelayTest_Click(object sender, EventArgs e)
        {
            WriteData($"relay~");
        }
        private void button3_Click(object sender, EventArgs e)
        {

            for (int i = 0; i <= 360; i++)
            {
              //  stringsToBeInterpreted.Enqueue($"mapPoint|{30}|{i}|0|{0}|90|{40}|-90|{40}|`");
            }
            for (int i = 360; i >= 0; i--)
            {
               // stringsToBeInterpreted.Enqueue($"mapPoint|{30}|{i}|0|{0}|90|{-40}|-90|{-40}|`");
            }
            for (int i = 180; i < 360; i++)
            {
               // stringsToBeInterpreted.Enqueue($"mapPoint|{25}|{180}|0|{0}|90|{-0}|-90|{-0}|`");
            }
            for (int i = 1; i < 40; i++)
            {
              // stringsToBeInterpreted.Enqueue($"mapPoint|{25}|{90}|0|{0}|90|{-0}|-90|{-0}|`");
            }
            for (int i = 1; i < 40; i++)
            {
             //   stringsToBeInterpreted.Enqueue($"mapPoint|{24}|{135}|0|{0}|90|{-0}|-90|{-0}|`");
            }
            for (int i = 360; i > 180; i--)
            {
             //   stringsToBeInterpreted.Enqueue($"mapPoint|{25}|{i}|0|{0}|90|{-10}|-90|{-10}|`");
            }
            for (int i = 1; i < 40; i++)
            {
               // stringsToBeInterpreted.Enqueue($"mapPoint|{25}|{45}|0|{0}|90|{-0}|-90|{-0}|`");
            }

        }
    }
}