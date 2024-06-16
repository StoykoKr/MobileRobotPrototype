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
            btn_ManualPosition = new Button();
            btn_ManualRotation = new Button();
            txtBox_TextOutput = new TextBox();
            btn_SaveImg = new Button();
            btn_ConvertLoadedToOccupancyGrid = new Button();
            btn_ControlRobot = new Button();
            btn_ConnectionButton = new Button();
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
            // btn_ManualPosition
            // 
            btn_ManualPosition.Location = new Point(1035, 86);
            btn_ManualPosition.Name = "btn_ManualPosition";
            btn_ManualPosition.Size = new Size(154, 23);
            btn_ManualPosition.TabIndex = 3;
            btn_ManualPosition.Text = "Assign position manually";
            btn_ManualPosition.UseVisualStyleBackColor = true;
            btn_ManualPosition.Click += btn_ManualPosition_Click;
            // 
            // btn_ManualRotation
            // 
            btn_ManualRotation.Location = new Point(1031, 115);
            btn_ManualRotation.Name = "btn_ManualRotation";
            btn_ManualRotation.Size = new Size(180, 23);
            btn_ManualRotation.TabIndex = 4;
            btn_ManualRotation.Text = "Assign rotation manually";
            btn_ManualRotation.UseVisualStyleBackColor = true;
            // 
            // txtBox_TextOutput
            // 
            txtBox_TextOutput.Location = new Point(1037, 227);
            txtBox_TextOutput.Multiline = true;
            txtBox_TextOutput.Name = "txtBox_TextOutput";
            txtBox_TextOutput.Size = new Size(161, 299);
            txtBox_TextOutput.TabIndex = 5;
            // 
            // btn_SaveImg
            // 
            btn_SaveImg.Location = new Point(1057, 541);
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
            btn_ControlRobot.Location = new Point(1057, 184);
            btn_ControlRobot.Name = "btn_ControlRobot";
            btn_ControlRobot.Size = new Size(120, 23);
            btn_ControlRobot.TabIndex = 12;
            btn_ControlRobot.Text = "Control robot";
            btn_ControlRobot.UseVisualStyleBackColor = true;
            btn_ControlRobot.Click += btn_ControlRobot_Click;
            // 
            // btn_ConnectionButton
            // 
            btn_ConnectionButton.Location = new Point(1050, 147);
            btn_ConnectionButton.Name = "btn_ConnectionButton";
            btn_ConnectionButton.Size = new Size(139, 23);
            btn_ConnectionButton.TabIndex = 13;
            btn_ConnectionButton.Text = "WaitForConnection";
            btn_ConnectionButton.UseVisualStyleBackColor = true;
            btn_ConnectionButton.Click += btn_ConnectionButton_Click;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1223, 638);
            Controls.Add(btn_ConnectionButton);
            Controls.Add(btn_ControlRobot);
            Controls.Add(btn_ConvertLoadedToOccupancyGrid);
            Controls.Add(btn_SaveImg);
            Controls.Add(txtBox_TextOutput);
            Controls.Add(btn_ManualRotation);
            Controls.Add(btn_ManualPosition);
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
        private Button btn_ManualPosition;
        private Button btn_ManualRotation;
        private TextBox txtBox_TextOutput;
        private Button btn_SaveImg;
        private Button btn_ConvertLoadedToOccupancyGrid;
        private Button btn_ControlRobot;
        private Button btn_ConnectionButton;
    }
}
