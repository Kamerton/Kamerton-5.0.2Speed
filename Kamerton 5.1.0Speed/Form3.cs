using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using KamertonTest;

namespace KamertonTest
{
    public partial class Form3 : Form
    {
        //Событие для передачи данных
      //  public event EventHandler<UserEventArgs> sendDataFromFormEvent3;
      //  public event EventHandler<UserEventArgs> sendDataFromFormEventS3;
        private int loc_x = 0;
        private int loc_y = 0;
        private int Form_Height;
        private int Form_Width;
        private Oscilloscope oscilloscope1 = Oscilloscope.CreateScope("Oscilloscope/Oscilloscope_settings.ini", "");
        public Form3()
        {
            InitializeComponent();
        }

        private void Form3_Load(object sender, EventArgs e)
        {
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;//убираем рамки 
            this.WindowState = FormWindowState.Maximized;//и только потом расширяем форму на весь экран 

            Form_Height = this.Height; //считываем нашей размеры формы 
            Form_Width = this.Width;

            this.WindowState = FormWindowState.Normal; //и не забываем вернуть форму в исходное положение 
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Sizable;//возвращаем рамки 
        }

        private void button1_Click(object sender, EventArgs e)
        {
            this.Close();
        }

  

        private void button55_Click(object sender, EventArgs e)
        {
            oscilloscope1.ShowScope();
            oscilloscope1.UpdateScope();
            oscilloscope1.CellSize = 10;
            oscilloscope1.VerticalOffset1 = -500;
            oscilloscope1.AmplitudeScale1 = 50;
            oscilloscope1.Left = (Form_Width / 2) - 475;
            oscilloscope1.Top = (Form_Height / 2) - 305;
        }

        private void button36_Click(object sender, EventArgs e)
        {
            oscilloscope1.HideScope();
        }

        private void Form3_LocationChanged(object sender, EventArgs e)
        {
            loc_x = this.Location.X;
            loc_y = this.Location.Y;
            oscilloscope1.Left = 25 + loc_x;
            oscilloscope1.Top = 60 + loc_y;
        }
    }
}