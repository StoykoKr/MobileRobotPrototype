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
            btnLessPWM = new Button();
            btnMorePWM = new Button();
            btnCheckCurrentPWM = new Button();
            btnStop = new Button();
            btnDir = new Button();
            btnMQTT = new Button();
            btnSetServo = new Button();
            txtBoxServo = new TextBox();
            button2 = new Button();
            ((System.ComponentModel.ISupportInitialize)pBox_Area).BeginInit();
            SuspendLayout();
            // 
            // pBox_Area
            // 
            pBox_Area.Location = new Point(43, 28);
            pBox_Area.Name = "pBox_Area";
            pBox_Area.Size = new Size(970, 586);
            pBox_Area.TabIndex = 0;
            pBox_Area.TabStop = false;
            pBox_Area.Click += pBox_Area_Click;
            pBox_Area.Paint += pBox_Area_Paint;
            pBox_Area.MouseDown += pBox_Area_MouseDown;
            pBox_Area.MouseMove += pBox_Area_MouseMove;
            // 
            // btn_CreateNewImage
            // 
            btn_CreateNewImage.Location = new Point(1057, 57);
            btn_CreateNewImage.Name = "btn_CreateNewImage";
            btn_CreateNewImage.Size = new Size(118, 23);
            btn_CreateNewImage.TabIndex = 1;
            btn_CreateNewImage.Text = "Start new scan";
            btn_CreateNewImage.UseVisualStyleBackColor = true;
            btn_CreateNewImage.Click += btn_CreateNewImage_Click;
            // 
            // btn_LoadImageAsMap
            // 
            btn_LoadImageAsMap.Location = new Point(1044, 28);
            btn_LoadImageAsMap.Name = "btn_LoadImageAsMap";
            btn_LoadImageAsMap.Size = new Size(156, 23);
            btn_LoadImageAsMap.TabIndex = 2;
            btn_LoadImageAsMap.Text = "Load an existing map";
            btn_LoadImageAsMap.UseVisualStyleBackColor = true;
            btn_LoadImageAsMap.Click += btn_LoadImageAsMap_Click;
            // 
            // btn_ManualRotation
            // 
            btn_ManualRotation.Location = new Point(1093, 290);
            btn_ManualRotation.Name = "btn_ManualRotation";
            btn_ManualRotation.Size = new Size(118, 23);
            btn_ManualRotation.TabIndex = 4;
            btn_ManualRotation.Text = "Calculate route";
            btn_ManualRotation.UseVisualStyleBackColor = true;
            btn_ManualRotation.Click += btn_ManualRotation_Click;
            // 
            // txtBox_TextOutput
            // 
            txtBox_TextOutput.Location = new Point(1037, 348);
            txtBox_TextOutput.Multiline = true;
            txtBox_TextOutput.Name = "txtBox_TextOutput";
            txtBox_TextOutput.Size = new Size(161, 178);
            txtBox_TextOutput.TabIndex = 5;
            // 
            // btn_SaveImg
            // 
            btn_SaveImg.Location = new Point(1019, 545);
            btn_SaveImg.Name = "btn_SaveImg";
            btn_SaveImg.Size = new Size(118, 23);
            btn_SaveImg.TabIndex = 6;
            btn_SaveImg.Text = "Save as raw map";
            btn_SaveImg.UseVisualStyleBackColor = true;
            btn_SaveImg.Click += btn_SaveImg_Click;
            // 
            // btn_ConvertLoadedToOccupancyGrid
            // 
            btn_ConvertLoadedToOccupancyGrid.Location = new Point(1044, 580);
            btn_ConvertLoadedToOccupancyGrid.Name = "btn_ConvertLoadedToOccupancyGrid";
            btn_ConvertLoadedToOccupancyGrid.Size = new Size(152, 46);
            btn_ConvertLoadedToOccupancyGrid.TabIndex = 11;
            btn_ConvertLoadedToOccupancyGrid.Text = "Convert loaded map to occupancy grid";
            btn_ConvertLoadedToOccupancyGrid.UseVisualStyleBackColor = true;
            btn_ConvertLoadedToOccupancyGrid.Click += btn_ConvertLoadedToOccupancyGrid_Click;
            // 
            // btn_ControlRobot
            // 
            btn_ControlRobot.Location = new Point(1037, 115);
            btn_ControlRobot.Name = "btn_ControlRobot";
            btn_ControlRobot.Size = new Size(164, 23);
            btn_ControlRobot.TabIndex = 12;
            btn_ControlRobot.Text = "Manual robot control";
            btn_ControlRobot.UseVisualStyleBackColor = true;
            btn_ControlRobot.Click += btn_ControlRobot_Click;
            // 
            // btn_ConnectionButton
            // 
            btn_ConnectionButton.Location = new Point(1044, 86);
            btn_ConnectionButton.Name = "btn_ConnectionButton";
            btn_ConnectionButton.Size = new Size(139, 23);
            btn_ConnectionButton.TabIndex = 13;
            btn_ConnectionButton.Text = "WaitForConnection";
            btn_ConnectionButton.UseVisualStyleBackColor = true;
            btn_ConnectionButton.Click += btn_ConnectionButton_Click;
            // 
            // button1
            // 
            button1.Location = new Point(1044, 233);
            button1.Name = "button1";
            button1.Size = new Size(139, 23);
            button1.TabIndex = 14;
            button1.Text = "Assign grid to map";
            button1.UseVisualStyleBackColor = true;
            button1.Click += button1_Click;
            // 
            // btn_SetStart
            // 
            btn_SetStart.Location = new Point(1019, 261);
            btn_SetStart.Name = "btn_SetStart";
            btn_SetStart.Size = new Size(75, 23);
            btn_SetStart.TabIndex = 15;
            btn_SetStart.Text = "SetStart";
            btn_SetStart.UseVisualStyleBackColor = true;
            btn_SetStart.Click += btn_SetStart_Click;
            // 
            // btn_SetEnd
            // 
            btn_SetEnd.Location = new Point(1147, 262);
            btn_SetEnd.Name = "btn_SetEnd";
            btn_SetEnd.Size = new Size(71, 22);
            btn_SetEnd.TabIndex = 16;
            btn_SetEnd.Text = "SetEnd";
            btn_SetEnd.UseVisualStyleBackColor = true;
            btn_SetEnd.Click += btn_SetEnd_Click;
            // 
            // btn_ExecuteRoute
            // 
            btn_ExecuteRoute.Location = new Point(1116, 319);
            btn_ExecuteRoute.Name = "btn_ExecuteRoute";
            btn_ExecuteRoute.Size = new Size(95, 23);
            btn_ExecuteRoute.TabIndex = 17;
            btn_ExecuteRoute.Text = "ExecuteRoute";
            btn_ExecuteRoute.UseVisualStyleBackColor = true;
            btn_ExecuteRoute.Click += btn_ExecuteRoute_Click;
            // 
            // btnLessPWM
            // 
            btnLessPWM.Location = new Point(1019, 144);
            btnLessPWM.Name = "btnLessPWM";
            btnLessPWM.Size = new Size(75, 23);
            btnLessPWM.TabIndex = 18;
            btnLessPWM.Text = "LessPWM";
            btnLessPWM.UseVisualStyleBackColor = true;
            btnLessPWM.Click += btnLessPWM_Click;
            // 
            // btnMorePWM
            // 
            btnMorePWM.Location = new Point(1147, 144);
            btnMorePWM.Name = "btnMorePWM";
            btnMorePWM.Size = new Size(75, 23);
            btnMorePWM.TabIndex = 19;
            btnMorePWM.Text = "MorePWM";
            btnMorePWM.UseVisualStyleBackColor = true;
            btnMorePWM.Click += btnMorePWM_Click;
            // 
            // btnCheckCurrentPWM
            // 
            btnCheckCurrentPWM.Location = new Point(1068, 173);
            btnCheckCurrentPWM.Name = "btnCheckCurrentPWM";
            btnCheckCurrentPWM.Size = new Size(115, 23);
            btnCheckCurrentPWM.TabIndex = 20;
            btnCheckCurrentPWM.Text = "Current PWM";
            btnCheckCurrentPWM.UseVisualStyleBackColor = true;
            btnCheckCurrentPWM.Click += btnCheckCurrentPWM_Click;
            // 
            // btnStop
            // 
            btnStop.Location = new Point(1019, 202);
            btnStop.Name = "btnStop";
            btnStop.Size = new Size(75, 23);
            btnStop.TabIndex = 21;
            btnStop.Text = "stop";
            btnStop.UseVisualStyleBackColor = true;
            btnStop.Click += btnStop_Click;
            // 
            // btnDir
            // 
            btnDir.Location = new Point(1147, 202);
            btnDir.Name = "btnDir";
            btnDir.Size = new Size(75, 23);
            btnDir.TabIndex = 22;
            btnDir.Text = "Dirction";
            btnDir.UseVisualStyleBackColor = true;
            btnDir.Click += btnDir_Click;
            // 
            // btnMQTT
            // 
            btnMQTT.Location = new Point(1147, 545);
            btnMQTT.Name = "btnMQTT";
            btnMQTT.Size = new Size(75, 23);
            btnMQTT.TabIndex = 23;
            btnMQTT.Text = "MQTT";
            btnMQTT.UseVisualStyleBackColor = true;
            btnMQTT.Click += btnMQTT_Click;
            // 
            // btnSetServo
            // 
            btnSetServo.Location = new Point(1019, 319);
            btnSetServo.Name = "btnSetServo";
            btnSetServo.Size = new Size(75, 23);
            btnSetServo.TabIndex = 24;
            btnSetServo.Text = "SetServo^";
            btnSetServo.UseVisualStyleBackColor = true;
            btnSetServo.Click += btnSetServo_Click;
            // 
            // txtBoxServo
            // 
            txtBoxServo.Location = new Point(1019, 291);
            txtBoxServo.Name = "txtBoxServo";
            txtBoxServo.Size = new Size(61, 23);
            txtBoxServo.TabIndex = 25;
            // 
            // button2
            // 
            button2.Location = new Point(1082, 204);
            button2.Name = "button2";
            button2.Size = new Size(75, 23);
            button2.TabIndex = 26;
            button2.Text = "button2";
            button2.UseVisualStyleBackColor = true;
            button2.Click += button2_Click;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1223, 638);
            Controls.Add(button2);
            Controls.Add(txtBoxServo);
            Controls.Add(btnSetServo);
            Controls.Add(btnMQTT);
            Controls.Add(btnDir);
            Controls.Add(btnStop);
            Controls.Add(btnCheckCurrentPWM);
            Controls.Add(btnMorePWM);
            Controls.Add(btnLessPWM);
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
        private Button btnLessPWM;
        private Button btnMorePWM;
        private Button btnCheckCurrentPWM;
        private Button btnStop;
        private Button btnDir;
        private Button btnMQTT;
        private Button btnSetServo;
        private TextBox txtBoxServo;
        private Button button2;
    }
}
