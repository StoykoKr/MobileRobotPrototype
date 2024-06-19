using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing.Imaging;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Windows.Forms;
using System.Xml.Linq;

namespace RobotAppControl
{
    public partial class Form1 : Form
    {
        private Grid _grid;
        private Robot _robot;

        private PictureBox PictureBox;
        private int picture_offsetX = 0, picture_offsetY = 0;
        private int _xPos = 0;  // mouseposition at MouseDown
        private int _yPos = 0;  // mouseposition at MouseDown
        private bool _dragging = false;
        private Rectangle _imgRect = new Rectangle(0, 0, 0, 0);
        private CustomBitmap custom;
        private CustomBitmap occupancyMap;
        private bool canHear = false;
        ConcurrentQueue<string> stringsToBeInterpreted;
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
        public Form1()
        {
            InitializeComponent();
            stringsToBeInterpreted = new ConcurrentQueue<string>();
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
                listener = new Listener(ref stringsToBeInterpreted, ref canHear);
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
                    commands.Add($"moveForward~{Math.Abs(dx)}~"); // assuming 1 unit is 10 cm
                }
                else if (dy != 0)
                {
                    commands.Add(dy > 0 ? "turn~90~" : "turn~-90~");
                    commands.Add($"moveForward~{Math.Abs(dy)}~"); // assuming 1 unit is 10 cm
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
            return color.R < 50 && color.G < 50 && color.B < 50; // Example threshold for black
        }
        private void btn_RunAStar_Click(object sender, EventArgs e)
        {
            // Define start and goal positions
            var start = new Node(startX, startY);
            var goal = new Node(endX, endY);

            // Find path using A* algorithm
            var aStar = new AStar(_grid);
            var path = aStar.FindPath(start, goal);

            // Execute path with robot
            _robot = new Robot(_grid, custom, start.X, start.Y, this);
            _robot.ExecutePath(path);

            // Refresh the picture box to show the path
            RefreshPicture();
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
        private void btn_LoadImageAsMap_Click(object sender, EventArgs e)
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
        private void pBox_Area_Click(object sender, EventArgs e)
        {
            MouseEventArgs me = (MouseEventArgs)e;
            if (me.Button != MouseButtons.Right)
                return;
            Point coordinates = me.Location;
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
        private bool CheckConnection()
        {
            if (stream == null)
            {
                StopListening();
                StopInterpreting();
                return false;
            }
            try
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
            }
        }
        private void btn_SaveImg_Click(object sender, EventArgs e)
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
        private void HandleAdjacentPixels(int iCentr, int jCentr, int spread, Graphics graphics)
        {
            Color current = custom.GetPixel(iCentr, jCentr);
            // Color smallCurrent;
            if (current.R >= 200 && current.B >= 200 && current.G >= 200)
            {
                int blackCount = 0;
                for (int i = iCentr - spread; i <= iCentr + spread; i++)
                {
                    for (int j = jCentr - spread; j <= jCentr + spread; j++)
                    {
                        if (i >= 0 && i < custom.Width && j >= 0 && j < custom.Height)
                        {
                            if (custom.GetPixel(i, j).R > 20)
                            {
                                blackCount++;
                            }
                        }
                    }
                }
                if (blackCount >= spread)
                {

                    rectangle = new Rectangle(iCentr, jCentr, 1, 1);
                    rectangle.Inflate(spread, spread);
                    graphics.FillRectangle(Brushes.Black, rectangle);


                    /*  for (int i = iCentr - spread * 3; i <= iCentr + spread * 3; i++)
                      {
                          for (int j = jCentr - spread * 3; j <= jCentr + spread * 3; j++)
                          {
                              if (i >= 0 && i < custom.Width && j >= 0 && j < custom.Height)
                              {
                                  smallCurrent = custom.GetPixel(i, j);
                                  if (smallCurrent.R > 230)
                                  {
                                      graphics.DrawLine(pen, iCentr, jCentr, i, j);   // feels very unoptimised.. because it is but the result is cool. Acceptable if we do this only once
                                  }

                              }
                          }
                      } */
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
            pen.Width = 4;
            using (occupancyMap)
            {
                Graphics g = Graphics.FromImage(occupancyMap.Bitmap);
                g.Clear(Color.White);
                for (int i = 0; i < occupancyMap.Width; i++)
                {
                    for (int j = 0; j < occupancyMap.Height; j++)
                    {
                        HandleAdjacentPixels(i, j, 7, g); // the third one is the spread value aka how many "rings" around the middle
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
        private void btn_ConvertLoadedToOccupancyGrid_Click(object sender, EventArgs e)
        {
            Thread thread = new Thread(() => ConvertToOccMap());
            thread.Start();
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
                if (currentlyPressedKey == Keys.S)
                {
                    ///
                }
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

        private void btn_ControlRobot_Click(object sender, EventArgs e)
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

        private void btn_ConnectionButton_Click(object sender, EventArgs e)
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

        private void btn_CreateNewImage_Click(object sender, EventArgs e)
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

        private void btn_ManualPosition_Click(object sender, EventArgs e)
        {

        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            StopListening();
            StopInterpreting();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            SetObstaclesFromMap(custom);
        }

        private void btn_ManualRotation_Click(object sender, EventArgs e)
        {
            // Define start and goal positions
            var start = new Node(startX, startY);
            var goal = new Node(endX, endY);

            // Find path using A* algorithm
            var aStar = new AStar(_grid);
            var path = aStar.FindPath(start, goal);

            // Execute path with robot
            _robot = new Robot(_grid, custom, start.X, start.Y, this);
            if (path != null)
            {
                finalPath = path;
                _robot.ExecutePath(path);
            }
            else
            {
                MessageBox.Show("No path");
            }

            // Refresh the picture box to show the path
            RefreshPicture();
        }

        private void btn_SetStart_Click(object sender, EventArgs e)
        {
            settingStart = true;
            settingEnd = false;
        }

        private void btn_SetEnd_Click(object sender, EventArgs e)
        {
            settingEnd = true;
            settingStart = false;
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
                        result.Add("moveForward~" + (Math.Abs(movedy)*10).ToString() + "~");
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
                        result.Add("moveForward~" + (Math.Abs(movedx)*10).ToString() + "~");
                        result.Add(dy > 0 ? "turn~90~" : "turn~-90~");
                        movedx = 0;
                        movedy = 0;
                    }
                    movedy += dy;
                }
            }
            result.Add("moveForward~" + (Math.Abs(movedy + movedx)*10).ToString() + "~");
            return result;
        }
        private void btn_ExecuteRoute_Click(object sender, EventArgs e)
        {
            ExecutePath(CookedPath(finalPath));
        }
    }
}
/*
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
                    commands.Add($"moveForward~{Math.Abs(dx)}~"); // assuming 1 unit is 10 cm
                }
                else if (dy != 0)
                {
                    commands.Add(dy > 0 ? "turn~90~" : "turn~-90~");
                    commands.Add($"moveForward~{Math.Abs(dy)}~"); // assuming 1 unit is 10 cm
                }
            }*/