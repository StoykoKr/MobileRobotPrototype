namespace RobotAppControl
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            pBox_Area = new PictureBox();
            btn_CreateNewImage = new Button();
            btn_LoadImageAsMap = new Button();
            btn_ManualRotation = new Button();
            txtBox_TextOutput = new TextBox();
            btn_SaveImg = new Button();
            btn_ConvertLoadedToOccupancyGrid = new Button();
            btn_ControlRobot = new Button();
            btn_ConnectionButton = new Button();
            button1 = new Button();
            btn_SetStart = new Button();
            btn_SetEnd = new Button();
            btn_ExecuteRoute = new Button();
            btnSetServo = new Button();
            txtBoxServo = new TextBox();
            button2 = new Button();
            btnCalib = new Button();
            btnRelayTest = new Button();
            button3 = new Button();
            btnArmDown = new Button();
            btnArmUp = new Button();
            btnGrab = new Button();
            btnRelease = new Button();
            txtBoxWeight = new TextBox();
            btnExplode = new Button();
            btnSendImage = new Button();
            btnLoadTwo = new Button();
            button4 = new Button();
            btnSwitchView = new Button();
            btnScanView = new Button();
            ((System.ComponentModel.ISupportInitialize)pBox_Area).BeginInit();
            SuspendLayout();
            // 
            // pBox_Area
            // 
            pBox_Area.Location = new Point(43, 28);
            pBox_Area.Name = "pBox_Area";
            pBox_Area.Size = new Size(806, 586);
            pBox_Area.TabIndex = 0;
            pBox_Area.TabStop = false;
            pBox_Area.Click += pBox_Area_Click;
            pBox_Area.Paint += pBox_Area_Paint;
            pBox_Area.MouseDown += pBox_Area_MouseDown;
            pBox_Area.MouseMove += pBox_Area_MouseMove;
            // 
            // btn_CreateNewImage
            // 
            btn_CreateNewImage.Location = new Point(1100, 57);
            btn_CreateNewImage.Name = "btn_CreateNewImage";
            btn_CreateNewImage.Size = new Size(118, 23);
            btn_CreateNewImage.TabIndex = 1;
            btn_CreateNewImage.Text = "Start new scan";
            btn_CreateNewImage.UseVisualStyleBackColor = true;
            btn_CreateNewImage.Click += btn_CreateNewImage_Click;
            // 
            // btn_LoadImageAsMap
            // 
            btn_LoadImageAsMap.Location = new Point(1062, 28);
            btn_LoadImageAsMap.Name = "btn_LoadImageAsMap";
            btn_LoadImageAsMap.Size = new Size(156, 23);
            btn_LoadImageAsMap.TabIndex = 2;
            btn_LoadImageAsMap.Text = "Load an existing map";
            btn_LoadImageAsMap.UseVisualStyleBackColor = true;
            btn_LoadImageAsMap.Click += btn_LoadImageAsMap_Click;
            // 
            // btn_ManualRotation
            // 
            btn_ManualRotation.Location = new Point(1104, 332);
            btn_ManualRotation.Name = "btn_ManualRotation";
            btn_ManualRotation.Size = new Size(118, 23);
            btn_ManualRotation.TabIndex = 4;
            btn_ManualRotation.Text = "Calculate route";
            btn_ManualRotation.UseVisualStyleBackColor = true;
            btn_ManualRotation.Click += btn_ManualRotation_Click;
            // 
            // txtBox_TextOutput
            // 
            txtBox_TextOutput.Location = new Point(855, 361);
            txtBox_TextOutput.Multiline = true;
            txtBox_TextOutput.Name = "txtBox_TextOutput";
            txtBox_TextOutput.Size = new Size(283, 178);
            txtBox_TextOutput.TabIndex = 5;
            // 
            // btn_SaveImg
            // 
            btn_SaveImg.Location = new Point(986, 545);
            btn_SaveImg.Name = "btn_SaveImg";
            btn_SaveImg.Size = new Size(118, 23);
            btn_SaveImg.TabIndex = 6;
            btn_SaveImg.Text = "Save as raw map";
            btn_SaveImg.UseVisualStyleBackColor = true;
            btn_SaveImg.Click += btn_SaveImg_Click;
            // 
            // btn_ConvertLoadedToOccupancyGrid
            // 
            btn_ConvertLoadedToOccupancyGrid.Location = new Point(986, 580);
            btn_ConvertLoadedToOccupancyGrid.Name = "btn_ConvertLoadedToOccupancyGrid";
            btn_ConvertLoadedToOccupancyGrid.Size = new Size(152, 46);
            btn_ConvertLoadedToOccupancyGrid.TabIndex = 11;
            btn_ConvertLoadedToOccupancyGrid.Text = "Convert loaded map to occupancy grid";
            btn_ConvertLoadedToOccupancyGrid.UseVisualStyleBackColor = true;
            btn_ConvertLoadedToOccupancyGrid.Click += btn_ConvertLoadedToOccupancyGrid_Click;
            // 
            // btn_ControlRobot
            // 
            btn_ControlRobot.Location = new Point(1054, 115);
            btn_ControlRobot.Name = "btn_ControlRobot";
            btn_ControlRobot.Size = new Size(164, 23);
            btn_ControlRobot.TabIndex = 12;
            btn_ControlRobot.Text = "Manual robot control";
            btn_ControlRobot.UseVisualStyleBackColor = true;
            btn_ControlRobot.Click += btn_ControlRobot_Click;
            // 
            // btn_ConnectionButton
            // 
            btn_ConnectionButton.Location = new Point(1079, 86);
            btn_ConnectionButton.Name = "btn_ConnectionButton";
            btn_ConnectionButton.Size = new Size(139, 23);
            btn_ConnectionButton.TabIndex = 13;
            btn_ConnectionButton.Text = "WaitForConnection";
            btn_ConnectionButton.UseVisualStyleBackColor = true;
            btn_ConnectionButton.Click += btn_ConnectionButton_Click;
            // 
            // button1
            // 
            button1.Location = new Point(977, 280);
            button1.Name = "button1";
            button1.Size = new Size(139, 23);
            button1.TabIndex = 14;
            button1.Text = "Assign grid to map";
            button1.UseVisualStyleBackColor = true;
            button1.Click += button1_Click;
            // 
            // btn_SetStart
            // 
            btn_SetStart.Location = new Point(1144, 372);
            btn_SetStart.Name = "btn_SetStart";
            btn_SetStart.Size = new Size(75, 23);
            btn_SetStart.TabIndex = 15;
            btn_SetStart.Text = "SetStart";
            btn_SetStart.UseVisualStyleBackColor = true;
            btn_SetStart.Click += btn_SetStart_Click;
            // 
            // btn_SetEnd
            // 
            btn_SetEnd.Location = new Point(1147, 401);
            btn_SetEnd.Name = "btn_SetEnd";
            btn_SetEnd.Size = new Size(71, 22);
            btn_SetEnd.TabIndex = 16;
            btn_SetEnd.Text = "SetEnd";
            btn_SetEnd.UseVisualStyleBackColor = true;
            btn_SetEnd.Click += btn_SetEnd_Click;
            // 
            // btn_ExecuteRoute
            // 
            btn_ExecuteRoute.Location = new Point(1147, 465);
            btn_ExecuteRoute.Name = "btn_ExecuteRoute";
            btn_ExecuteRoute.Size = new Size(75, 23);
            btn_ExecuteRoute.TabIndex = 17;
            btn_ExecuteRoute.Text = "ExecuteRoute";
            btn_ExecuteRoute.UseVisualStyleBackColor = true;
            btn_ExecuteRoute.Click += btn_ExecuteRoute_Click;
            // 
            // btnSetServo
            // 
            btnSetServo.Location = new Point(969, 332);
            btnSetServo.Name = "btnSetServo";
            btnSetServo.Size = new Size(87, 23);
            btnSetServo.TabIndex = 24;
            btnSetServo.Text = "load Default";
            btnSetServo.UseVisualStyleBackColor = true;
            btnSetServo.Click += btnSetServo_Click;
            // 
            // txtBoxServo
            // 
            txtBoxServo.Location = new Point(969, 309);
            txtBoxServo.Name = "txtBoxServo";
            txtBoxServo.Size = new Size(61, 23);
            txtBoxServo.TabIndex = 25;
            // 
            // button2
            // 
            button2.Location = new Point(1147, 545);
            button2.Name = "button2";
            button2.Size = new Size(75, 23);
            button2.TabIndex = 26;
            button2.Text = "button2";
            button2.UseVisualStyleBackColor = true;
            button2.Click += button2_Click;
            // 
            // btnCalib
            // 
            btnCalib.Location = new Point(1144, 429);
            btnCalib.Name = "btnCalib";
            btnCalib.Size = new Size(75, 23);
            btnCalib.TabIndex = 27;
            btnCalib.Text = "Calibrate";
            btnCalib.UseVisualStyleBackColor = true;
            btnCalib.Click += btnCalib_Click;
            // 
            // btnRelayTest
            // 
            btnRelayTest.Location = new Point(1122, 280);
            btnRelayTest.Name = "btnRelayTest";
            btnRelayTest.Size = new Size(99, 23);
            btnRelayTest.TabIndex = 28;
            btnRelayTest.Text = "save default";
            btnRelayTest.UseVisualStyleBackColor = true;
            btnRelayTest.Click += btnRelayTest_Click;
            // 
            // button3
            // 
            button3.Location = new Point(1147, 505);
            button3.Name = "button3";
            button3.Size = new Size(75, 23);
            button3.TabIndex = 29;
            button3.Text = "button3";
            button3.UseVisualStyleBackColor = true;
            button3.Click += button3_Click;
            // 
            // btnArmDown
            // 
            btnArmDown.Location = new Point(977, 169);
            btnArmDown.Name = "btnArmDown";
            btnArmDown.Size = new Size(75, 23);
            btnArmDown.TabIndex = 30;
            btnArmDown.Text = "ArmDown";
            btnArmDown.UseVisualStyleBackColor = true;
            btnArmDown.Click += btnArmDown_Click;
            // 
            // btnArmUp
            // 
            btnArmUp.Location = new Point(1079, 169);
            btnArmUp.Name = "btnArmUp";
            btnArmUp.Size = new Size(75, 23);
            btnArmUp.TabIndex = 31;
            btnArmUp.Text = "ArmUp";
            btnArmUp.UseVisualStyleBackColor = true;
            btnArmUp.Click += btnArmUp_Click;
            // 
            // btnGrab
            // 
            btnGrab.Location = new Point(978, 213);
            btnGrab.Name = "btnGrab";
            btnGrab.Size = new Size(75, 23);
            btnGrab.TabIndex = 32;
            btnGrab.Text = "Grab";
            btnGrab.UseVisualStyleBackColor = true;
            btnGrab.Click += btnGrab_Click;
            // 
            // btnRelease
            // 
            btnRelease.Location = new Point(1079, 213);
            btnRelease.Name = "btnRelease";
            btnRelease.Size = new Size(75, 23);
            btnRelease.TabIndex = 33;
            btnRelease.Text = "Release";
            btnRelease.UseVisualStyleBackColor = true;
            btnRelease.Click += btnRelease_Click;
            // 
            // txtBoxWeight
            // 
            txtBoxWeight.Location = new Point(977, 58);
            txtBoxWeight.Name = "txtBoxWeight";
            txtBoxWeight.Size = new Size(67, 23);
            txtBoxWeight.TabIndex = 34;
            // 
            // btnExplode
            // 
            btnExplode.Location = new Point(973, 115);
            btnExplode.Name = "btnExplode";
            btnExplode.Size = new Size(75, 23);
            btnExplode.TabIndex = 35;
            btnExplode.Text = "Explode";
            btnExplode.UseVisualStyleBackColor = true;
            btnExplode.Click += btnExplode_Click;
            // 
            // btnSendImage
            // 
            btnSendImage.Location = new Point(1143, 591);
            btnSendImage.Name = "btnSendImage";
            btnSendImage.Size = new Size(75, 23);
            btnSendImage.TabIndex = 36;
            btnSendImage.Text = "SendImage";
            btnSendImage.UseVisualStyleBackColor = true;
            btnSendImage.Click += btnSendImage_Click;
            // 
            // btnLoadTwo
            // 
            btnLoadTwo.Location = new Point(985, 24);
            btnLoadTwo.Name = "btnLoadTwo";
            btnLoadTwo.Size = new Size(75, 23);
            btnLoadTwo.TabIndex = 37;
            btnLoadTwo.Text = "DualLoad";
            btnLoadTwo.UseVisualStyleBackColor = true;
            btnLoadTwo.Click += btnLoadTwo_Click;
            // 
            // button4
            // 
            button4.Location = new Point(855, 86);
            button4.Name = "button4";
            button4.Size = new Size(143, 23);
            button4.TabIndex = 38;
            button4.Text = "Simulated mapping";
            button4.UseVisualStyleBackColor = true;
            button4.Click += button4_Click;
            // 
            // btnSwitchView
            // 
            btnSwitchView.Location = new Point(879, 144);
            btnSwitchView.Name = "btnSwitchView";
            btnSwitchView.Size = new Size(105, 23);
            btnSwitchView.TabIndex = 39;
            btnSwitchView.Text = "ShowOtherView";
            btnSwitchView.UseVisualStyleBackColor = true;
            btnSwitchView.Click += btnSwitchView_Click;
            // 
            // btnScanView
            // 
            btnScanView.Location = new Point(873, 199);
            btnScanView.Name = "btnScanView";
            btnScanView.Size = new Size(75, 23);
            btnScanView.TabIndex = 40;
            btnScanView.Text = "ShowScan";
            btnScanView.UseVisualStyleBackColor = true;
            btnScanView.Click += btnScanView_Click;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1223, 638);
            Controls.Add(btnScanView);
            Controls.Add(btnSwitchView);
            Controls.Add(button4);
            Controls.Add(btnLoadTwo);
            Controls.Add(btnSendImage);
            Controls.Add(btnExplode);
            Controls.Add(txtBoxWeight);
            Controls.Add(btnRelease);
            Controls.Add(btnGrab);
            Controls.Add(btnArmUp);
            Controls.Add(btnArmDown);
            Controls.Add(button3);
            Controls.Add(btnRelayTest);
            Controls.Add(btnCalib);
            Controls.Add(button2);
            Controls.Add(txtBoxServo);
            Controls.Add(btnSetServo);
            Controls.Add(btn_ExecuteRoute);
            Controls.Add(btn_SetEnd);
            Controls.Add(btn_SetStart);
            Controls.Add(button1);
            Controls.Add(btn_ConnectionButton);
            Controls.Add(btn_ControlRobot);
            Controls.Add(btn_ConvertLoadedToOccupancyGrid);
            Controls.Add(btn_SaveImg);
            Controls.Add(txtBox_TextOutput);
            Controls.Add(btn_ManualRotation);
            Controls.Add(btn_LoadImageAsMap);
            Controls.Add(btn_CreateNewImage);
            Controls.Add(pBox_Area);
            Name = "Form1";
            Text = "Form1";
            FormClosed += Form1_FormClosed;
            KeyDown += Form1_KeyDown;
            KeyUp += Form1_KeyUp;
            ((System.ComponentModel.ISupportInitialize)pBox_Area).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private PictureBox pBox_Area;
        private Button btn_CreateNewImage;
        private Button btn_LoadImageAsMap;
        private Button btn_ManualRotation;
        private TextBox txtBox_TextOutput;
        private Button btn_SaveImg;
        private Button btn_ConvertLoadedToOccupancyGrid;
        private Button btn_ControlRobot;
        private Button btn_ConnectionButton;
        private Button button1;
        private Button btn_SetStart;
        private Button btn_SetEnd;
        private Button btn_ExecuteRoute;
        private Button btnSetServo;
        private TextBox txtBoxServo;
        private Button button2;
        private Button btnCalib;
        private Button btnRelayTest;
        private Button button3;
        private Button btnArmDown;
        private Button btnArmUp;
        private Button btnGrab;
        private Button btnRelease;
        private TextBox txtBoxWeight;
        private Button btnExplode;
        private Button btnSendImage;
        private Button btnLoadTwo;
        private Button button4;
        private Button btnSwitchView;
        private Button btnScanView;
    }
}
