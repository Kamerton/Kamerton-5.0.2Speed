using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Printing;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.IO.Ports;
using System.Globalization;
using System.Threading;
using KamertonTest;
using System.Linq;
using FieldTalk.Modbus.Master;
using System.Reflection;
using System.Text.RegularExpressions;

namespace KamertonTest
    {
    public partial class Form2 : Form
        {
            //Событие для передачи данных
            public event EventHandler<UserEventArgs> sendDataFromFormEvent2;
            public event EventHandler<UserEventArgs> sendDataFromFormEventS2;
          //  int sampTrigThreshold = 0;
            private int loc_x = 0;
            private int loc_y = 0;
            private int Form_Height;
            private int Form_Width;
            ushort[] table_porog    = new ushort[50];
            bool end_run0           = false;
            bool oscill_run         = false;
            byte sampTrigThreshold1 = 30;
            byte sampTrigThreshold2 = 30;

            SerialPort ComPort2 = new SerialPort(File.ReadAllText("set_rs232.txt"), 115200, Parity.None, 8, StopBits.One);
            private Oscilloscope oscilloscope2 = Oscilloscope.CreateScope("Oscilloscope/Oscilloscope_settings.ini", "");

           // bool trackChange      = false;
    
            #region Variables and objects

            /// <summary>
            /// Timer to update terminal textbox at fixed interval.
            /// </summary>
          //  private System.Windows.Forms.Timer formUpdateTimer = new System.Windows.Forms.Timer();

            /// <summary>
            /// SerialPort object.
            /// </summary>
           // private SerialPort serialPort = new SerialPort();

            /// <summary>
            /// Sample counter to calculate performance statics.
            /// </summary>
            private SampleCounter sampleCounter = new SampleCounter();

            /// <summary>
            /// TextBoxBuffer containing text printed to terminal.
            /// </summary>
            private TextBoxBuffer textBoxBuffer = new TextBoxBuffer(4096);

            /// <summary>
            /// ASCII buffer for decoding CSVs in serial stream.
            /// </summary>
            private string asciiBuf = "";

            /// <summary>
            /// Oscilloscope channel values decoded from serial stream.
            /// </summary>
            private float[] channels = new float[9] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

            /// <summary>
            /// Oscilloscope for channels 1, 2 and 3.
            /// </summary>
          //  private Oscilloscope oscilloscope123 = Oscilloscope.CreateScope("Oscilloscope/Oscilloscope_settings.ini", "");

             /// <summary>
            /// CSV file writer.
            /// </summary>
          //  private CsvFileWriter csvFileWriter = null;

            #endregion
        
                 
        public Form2()
            {
               CallBackMy2.callbackEventHandler = new CallBackMy2.callbackEvent(this.Reload);

               InitializeComponent();
            }

        private void Form2_Load(object sender, EventArgs e)
            {
                ComPort2.Open();
                label7.Visible          = false;
                refresh.BackColor       = Color.LightGreen;
                MTT_save.BackColor      = Color.LightGreen;
                comboBox2.SelectedIndex = 2;
                ComPort2.DataReceived  += new SerialDataReceivedEventHandler(ComPort2_DataReceived);
                this.FormBorderStyle    = System.Windows.Forms.FormBorderStyle.None;//убираем рамки 
                this.WindowState        = FormWindowState.Maximized;//и только потом расширяем форму на весь экран 
                Form_Height             = this.Height; //считываем размеры нашей формы 
                Form_Width              = this.Width;
                this.WindowState        = FormWindowState.Normal; //и не забываем вернуть форму в исходное положение 
                this.FormBorderStyle    = System.Windows.Forms.FormBorderStyle.Sizable;//возвращаем рамки 
                loc_x                   = (Form_Width / 2) - 485;
                loc_y                   = (Form_Height / 2) - 320;
                set_start_level_res();
                oscilloscope_start();
                CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 29, true);
            }

        private void button1_Click(object sender, EventArgs e)
            {
                oscilloscope2.HideScope();
                CallBackMy.callbackEventHandler("Передаваемые данные.", 9, 2, 0, true);   //Отключить питание
                ComPort2.Close();
                CallBackMy.callbackEventHandler("Передаваемые данные.", 10, 0, 0, true);  //Включить ком порт 2
                this.Close();
             }
       
        #region oscilloscope

        private void button55_Click(object sender, EventArgs e)          //  Кнопка Включить осциллограф
        {
            oscill_run = true;
            CallBackMy.callbackEventHandler("Передаваемые данные.", 9, 1, 0, true);
            oscilloscope_start();
        }

        private void button36_Click(object sender, EventArgs e)          //  Кнопка Отключить осциллограф
        {
            oscill_run = false;
            label7.Visible = false;
            CallBackMy.callbackEventHandler("Передаваемые данные.", 9, 2, 0, true);
        }

        private void Form2_LocationChanged(object sender, EventArgs e)   // Премещать осциллограф
        {
            loc_x = this.Location.X;
            loc_y = this.Location.Y;
            loc_x += 20;
            loc_y += 45;

            if (loc_x < 20)
            {
                loc_x = 20;
            }

            else if (loc_x >= Form_Width)
            {
                loc_x = Form_Width;
            }
            if (loc_y < 56) loc_y = 56;

            else if (loc_y > Form_Height)
            {
                loc_y = Form_Height;
            }

            oscilloscope2.Left = loc_x;
            oscilloscope2.Top = loc_y;
        }
        private void oscilloscope_start()
        {
            oscilloscope2.Caption = "Гарнитура инструктора";
            oscilloscope2.ShowScope();
            oscilloscope2.UpdateScope();
            oscilloscope2.ClearScope();
            oscilloscope2.CellSize = 10;
            oscilloscope2.VerticalOffset1 = -500;
            oscilloscope2.AmplitudeScale1 = 50;
            oscilloscope2.Left = loc_x;
            oscilloscope2.Top = loc_y;

        }
        private void ComPort2_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                      // Get bytes from serial port
                    int bytesToRead = ComPort2.BytesToRead;
                    byte[] readBuffer = new byte[bytesToRead];
                    ComPort2.Read(readBuffer, 0, bytesToRead);

                    // Process each byte
                    foreach (byte b in readBuffer)
                    {
                        // Parse character to textBoxBuffer
                        if ((b < 0x20 || b > 0x7F) && b != '\r')    // replace non-printable characters with '.'
                        {
                            textBoxBuffer.Put(".");
                        }
                        else if (b == '\r')     // replace carriage return with '↵' and valid new line
                        {
                            textBoxBuffer.Put("↵" + Environment.NewLine);
                        }
                        else
                        {
                            textBoxBuffer.Put(((char)b).ToString());
                        }

                        // Extract CSVs and parse to Oscilloscope
                        if (asciiBuf.Length > 128)
                        {
                            asciiBuf = "";  // prevent memory leak
                        }
                        if ((char)b == '\r')
                        {
                            // Split string to comma separated variables (ignore non numerical characters)
                            string[] csvs = (new Regex(@"[^0-9\-,.]")).Replace(asciiBuf, "").Split(',');

                            // Extract each CSV as oscilloscope channel 
                            int channelIndex = 0;
                            foreach (string csv in csvs)
                            {
                                if (csv != "" && channelIndex < 9)
                                {
                                    channels[channelIndex++] = float.Parse(csv, CultureInfo.InvariantCulture);
                                }
                            }

                            // Update oscilloscopes if channel values changed
                            if (channelIndex > 0)
                            {
                                oscilloscope2.AddScopeData(channels[0], channels[1], channels[2]);
                                sampleCounter.Increment();
                            }
                            asciiBuf = "";
                        }
                        else
                        {
                            asciiBuf += (char)b;
                        }
                    }


            }

            catch { }
        }

        #endregion

        
         void Reload(string param, int par1, int par2, ushort par3,bool end_run)
             {

                 switch (par1)                                                // Определить № теста
                 {

                     default:
                     case 0:
                         end_run0 = end_run;
                         break;
                     case 1:
                          table_porog[par2] = par3;            //
                         break;
                     case 2:
                         set_info_reg( param,  par1, par2,  par3, end_run);
                         break;
                     case 3:
                         //
                         break;
                     case 4:
                         //
                         break;
                     case 5:
                         //
                         break;
                     case 6:
                         //
                         break;
                     case 7:
                         //
                         break;


                 }

                // end_run0 = end_run;
                // table_porog[par2] = par3;
             }

         private void set_info_reg(string param, int par1, int par2, ushort par3, bool end_run)
         {

             switch (par2)                                                // Определить № теста
             {

                 default:
      
                 case 1:

                     if (end_run == false) // 30041  флаг подключения гарнитуры инструктора 2 наушниками
                     {
                         label109.BackColor = Color.Red;
                         label109.Text = "0";
                     }
                     else
                     {
                         label109.BackColor = Color.Lime;
                         label109.Text = "1";
                     }

                     break;
                 case 2:
                     if (end_run == false) // 30042  флаг подключения гарнитуры инструктора
                     {
                         label110.BackColor = Color.Red;
                         label110.Text = "0";
                     }
                     else
                     {
                         label110.BackColor = Color.Lime;
                         label110.Text = "1";
                     }

                    break;

             }

         }

         private void refresh_Click(object sender, EventArgs e)
             {
                 if (oscill_run == false)
                 {
                     refresh.Enabled = false;
                     refresh.BackColor = Color.LightSalmon;
                     refresh.Text = "Загрузка";
                     refresh.Refresh();
                     end_run0 = false;
                     CallBackMy.callbackEventHandler("Передаваемые данные.", 1, 0, 0, false);     // Запрос данных порогов инструктора

                     for (int i = 0; i < 38; i++)
                     {
                         CallBackMy.callbackEventHandler("Передаваемые данные.", 2, i, 0, true);
                     }

                     while (!end_run0) { };
                     save_tab_instruk();
                     refresh.Enabled = true;
                     refresh.BackColor = Color.LightGreen;
                     refresh.Text = "Обновить";
                     refresh.Refresh();
                     set_start_level_res();
                 }
                 else
                 {
                     label7.Visible = true;
                 }
                          }
         private void save_tab_instruk()
         {
             button2.Text = table_porog[0].ToString();
             button3.Text = table_porog[1].ToString();
             textBox102.Text = table_porog[2].ToString();
             textBox103.Text = table_porog[3].ToString();
             textBox104.Text = table_porog[4].ToString();
             textBox105.Text = table_porog[5].ToString();
             textBox106.Text = table_porog[6].ToString();
             textBox107.Text = table_porog[7].ToString();
             textBox108.Text = table_porog[8].ToString();
             textBox109.Text = table_porog[9].ToString();
             textBox110.Text = table_porog[10].ToString();
             textBox111.Text = table_porog[11].ToString();
             textBox112.Text = table_porog[12].ToString();
             textBox113.Text = table_porog[13].ToString();
             textBox114.Text = table_porog[14].ToString();
             textBox115.Text = table_porog[15].ToString();
             textBox116.Text = table_porog[16].ToString();
             textBox117.Text = table_porog[17].ToString();
             textBox118.Text = table_porog[18].ToString();
             textBox119.Text = table_porog[19].ToString();

             textBox120.Text = table_porog[20].ToString();
             textBox121.Text = table_porog[21].ToString();
             textBox122.Text = table_porog[22].ToString();
             textBox123.Text = table_porog[23].ToString();
             textBox124.Text = table_porog[24].ToString();
             textBox125.Text = table_porog[25].ToString();
             textBox126.Text = table_porog[26].ToString();
             textBox127.Text = table_porog[27].ToString();
             textBox128.Text = table_porog[28].ToString();
             textBox129.Text = table_porog[29].ToString();
             textBox130.Text = table_porog[30].ToString();
             textBox131.Text = table_porog[31].ToString();
             textBox132.Text = table_porog[32].ToString();
             textBox133.Text = table_porog[33].ToString();
             textBox134.Text = table_porog[34].ToString();
             textBox135.Text = table_porog[34].ToString();
             textBox136.Text = table_porog[36].ToString();
             textBox137.Text = table_porog[37].ToString();
          }

         private void MTT_save_Click(object sender, EventArgs e)
         {
             if (oscill_run == false)
             { 

             CallBackMy.callbackEventHandler("Передаваемые данные.", 3, 1, 0, true);   // Сообщение о выполнения  записи таблицы порогов
             MTT_save.Enabled = false;
             MTT_save.BackColor = Color.LightSalmon;
             MTT_save.Text = "Загрузка";
             MTT_save.Refresh();
             //Создаем таблицу
             DataTable dt_Form2 = new DataTable("InstruktorDataTable");

             //Создаем столбцы
             DataColumn noise_min = new DataColumn("мин.", typeof(ushort));
             DataColumn noise_min1 = new DataColumn("минC.", typeof(ushort));
             // DataColumn noise_max = new DataColumn("шумы макс.", typeof(string));

             dt_Form2.Columns.AddRange(new DataColumn[] 
                {noise_min,
                 noise_min1});

             //Формируем строку из данных в текстовых полях
             DataRow row     = dt_Form2.NewRow();
             row[noise_min]  = ushort.Parse(textBox102.Text);
             row[noise_min1] = ushort.Parse(textBox103.Text);
             dt_Form2.Rows.Add(row);
             DataRow row1     = dt_Form2.NewRow();
             row1[noise_min]  = ushort.Parse(textBox104.Text);
             row1[noise_min1] = ushort.Parse(textBox105.Text);
             dt_Form2.Rows.Add(row1);
             DataRow row2     = dt_Form2.NewRow();
             row2[noise_min]  = ushort.Parse(textBox106.Text);
             row2[noise_min1] = ushort.Parse(textBox107.Text);
             dt_Form2.Rows.Add(row2);
             DataRow row3 = dt_Form2.NewRow();
             row3[noise_min] = ushort.Parse(textBox108.Text);
             row3[noise_min1] = ushort.Parse(textBox109.Text);
             dt_Form2.Rows.Add(row3);
             DataRow row4 = dt_Form2.NewRow();
             row4[noise_min] = ushort.Parse(textBox110.Text);
             row4[noise_min1] = ushort.Parse(textBox111.Text);
             dt_Form2.Rows.Add(row4);
             DataRow row5 = dt_Form2.NewRow();
             row5[noise_min] = ushort.Parse(textBox112.Text);
             row5[noise_min1] = ushort.Parse(textBox113.Text);
             dt_Form2.Rows.Add(row5);
             DataRow row6 = dt_Form2.NewRow();
             row6[noise_min] = ushort.Parse(textBox114.Text);
             row6[noise_min1] = ushort.Parse(textBox115.Text);
             dt_Form2.Rows.Add(row6);
             DataRow row7 = dt_Form2.NewRow();
             row7[noise_min] = ushort.Parse(textBox116.Text);
             row7[noise_min1] = ushort.Parse(textBox117.Text);
             dt_Form2.Rows.Add(row7);
             DataRow row8 = dt_Form2.NewRow();
             row8[noise_min] = ushort.Parse(textBox115.Text);
             row8[noise_min1] = ushort.Parse(textBox119.Text);
             dt_Form2.Rows.Add(row8);

             //Генерируем событие с именованным аргументом
             //в класс аргумента передаем созданную таблицу
             if (sendDataFromFormEvent2 != null) sendDataFromFormEvent2(this, new UserEventArgs(dt_Form2)); // Отправить данные в основную форму


             //Создаем таблицу
             DataTable dt_Form2S = new DataTable("InstruktorDataTable");

             //Создаем столбцы
             DataColumn noise_minS = new DataColumn("мин.", typeof(ushort));
             DataColumn noise_maxS = new DataColumn("макс.", typeof(ushort));

             dt_Form2S.Columns.AddRange(new DataColumn[] {
                noise_minS,
                noise_maxS});


             //Формируем строку из данных в текстовых полях
             DataRow rowS = dt_Form2S.NewRow();
             rowS[noise_minS] = ushort.Parse(textBox120.Text);
             rowS[noise_maxS] = ushort.Parse(textBox121.Text);
             dt_Form2S.Rows.Add(rowS);
             DataRow row1S = dt_Form2S.NewRow();
             row1S[noise_minS] = ushort.Parse(textBox122.Text);
             row1S[noise_maxS] = ushort.Parse(textBox123.Text);
             dt_Form2S.Rows.Add(row1S);
             DataRow row2S = dt_Form2S.NewRow();
             row2S[noise_minS] = ushort.Parse(textBox124.Text);
             row2S[noise_maxS] = ushort.Parse(textBox125.Text);
             dt_Form2S.Rows.Add(row2S);
             DataRow row3S = dt_Form2S.NewRow();
             row3S[noise_minS] = ushort.Parse(textBox126.Text);
             row3S[noise_maxS] = ushort.Parse(textBox127.Text);
             dt_Form2S.Rows.Add(row3S);
             DataRow row4S = dt_Form2S.NewRow();
             row4S[noise_minS] = ushort.Parse(textBox128.Text);
             row4S[noise_maxS] = ushort.Parse(textBox129.Text);
             dt_Form2S.Rows.Add(row4S);
             DataRow row5S = dt_Form2S.NewRow();
             row5S[noise_minS] = ushort.Parse(textBox130.Text);
             row5S[noise_maxS] = ushort.Parse(textBox131.Text);
             dt_Form2S.Rows.Add(row5S);
             DataRow row6S = dt_Form2S.NewRow();
             row6S[noise_minS] = ushort.Parse(textBox132.Text);
             row6S[noise_maxS] = ushort.Parse(textBox133.Text);
             dt_Form2S.Rows.Add(row6S);
             DataRow row7S = dt_Form2S.NewRow();
             row7S[noise_minS] = ushort.Parse(textBox134.Text);
             row7S[noise_maxS] = ushort.Parse(textBox135.Text);
             dt_Form2S.Rows.Add(row7S);
             DataRow row8S = dt_Form2S.NewRow();
             row8S[noise_minS] = ushort.Parse(textBox136.Text);
             row8S[noise_maxS] = ushort.Parse(textBox137.Text);
             dt_Form2S.Rows.Add(row8S);


             //Генерируем событие с именованным аргументом
             //в класс аргумента передаем созданную таблицу
             if (sendDataFromFormEventS2 != null) sendDataFromFormEventS2(this, new UserEventArgs(dt_Form2S));

             MTT_save.Enabled = true;
             MTT_save.BackColor = Color.LightGreen;
             MTT_save.Text = "Сохранить";
             MTT_save.Refresh();
             oscilloscope2.HideScope();
             ComPort2.Close();
             this.Close();

             }
            else
            {
                label7.Visible = true;
            }

         }

         private void radioButton3_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 0, 0, true);
         }

         private void radioButton4_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 1, 1, true);
         }

         private void radioButton5_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 2, 2, true);
         }

         private void radioButton6_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 3, 3, true);
         }

         private void radioButton7_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 4, 4, true);
         }

         private void radioButton8_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 5, 5, true);
         }

         private void radioButton9_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 6, 6, true);
         }

         private void radioButton10_CheckedChanged(object sender, EventArgs e)
         {
             CallBackMy.callbackEventHandler("Передаваемые данные.", 4, 7, 7, true);
         }

         private void radioButton1_CheckedChanged(object sender, EventArgs e)
         {
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 0, 0, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void radioButton2_CheckedChanged(object sender, EventArgs e)
         {
             if (!oscill_run)
              {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 1, 1, true);
              }
             else
             {
                 label7.Visible = true;
             }
         }

         private void radioButton11_CheckedChanged(object sender, EventArgs e)
         {
            if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 2, 2, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void radioButton12_CheckedChanged(object sender, EventArgs e)
         {
             if (!oscill_run)
             {
                  CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 3, 3, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void radioButton13_CheckedChanged(object sender, EventArgs e)
         {
             if (!oscill_run)
             {
                  CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 4, 4, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void radioButton14_CheckedChanged(object sender, EventArgs e)
         {
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 5, 5, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void radioButton16_CheckedChanged(object sender, EventArgs e)
         {
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 6, 6, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void radioButton15_CheckedChanged(object sender, EventArgs e)
         {
             if (!oscill_run)
             {
                  CallBackMy.callbackEventHandler("Передаваемые данные.", 5, 7, 7, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button2_Click(object sender, EventArgs e)       // Кнопка уровня входа№1
         {
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 6, 1, ushort.Parse(label11.Text), true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button3_Click(object sender, EventArgs e)       // Кнопка уровня входа№2
         {
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 6, 2, ushort.Parse(label12.Text), true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void cTrigLevel1_Scroll(object sender, EventArgs e)  // Ползунок №1
         {
             sampTrigThreshold1 = Convert.ToByte(cTrigLevel1.Value);
             // trackChange1 = true;
             label10.Text = string.Format("{0:0}%", sampTrigThreshold1 / 2.55);
             label10.Refresh();
             label11.Text = string.Format("{0:0}", sampTrigThreshold1);
             label11.Refresh();
             CallBackMy.callbackEventHandler("Передаваемые данные.", 7, 1, sampTrigThreshold1, true);
         }

         private void cTrigLevel2_Scroll(object sender, EventArgs e)  // Ползунок №2
         {
             sampTrigThreshold2 = Convert.ToByte(cTrigLevel2.Value);
             // trackChange2 = true;
             label13.Text = string.Format("{0:0}%", sampTrigThreshold2 / 2.55);
             label13.Refresh();
             label12.Text = string.Format("{0:0}", sampTrigThreshold2);
             label12.Refresh();
             CallBackMy.callbackEventHandler("Передаваемые данные.", 7, 2, sampTrigThreshold2, true);
         }

         private void set_start_level_res()
         {
             cTrigLevel1.Value = ushort.Parse(button2.Text);
             cTrigLevel2.Value = ushort.Parse(button3.Text);
             cTrigLevel1.Update();
             cTrigLevel2.Update();
             sampTrigThreshold1 = Convert.ToByte(cTrigLevel1.Value);
             label10.Text = string.Format("{0:0}%", sampTrigThreshold1 / 2.55);
             label10.Refresh();
             label11.Text = string.Format("{0:0}", sampTrigThreshold1);
             label11.Refresh();
             sampTrigThreshold2 = Convert.ToByte(cTrigLevel2.Value);
             label13.Text = string.Format("{0:0}%", sampTrigThreshold2 / 2.55);
             label13.Refresh();
             label12.Text = string.Format("{0:0}", sampTrigThreshold2);
             label12.Refresh();
             CallBackMy.callbackEventHandler("Передаваемые данные.", 7, 2, sampTrigThreshold2, true);
         }

         private void button35_Click(object sender, EventArgs e)      // Подключить канал и включить измерение
         {
             /*
        КОМАНДА 34   
       * comboBox3
       * regBank.get(40040)
       0 - FrontL    A10
       1 - FrontR    A11
       2 - LineL     A8  channel
       3 - LineR     A9
       4 - mag/radio A3
       5 - mag/phone A4
       6 - ГГС       A7
       7 - Радио1    A5
       8 - Радио2    A6
       9 - W         A12
     */

             ushort osc_channel = 0;
             switch (comboBox2.SelectedIndex)
             {
                 default:
                 case 0:
                     osc_channel = 10;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода FrontL ";
                     break;
                 case 1:
                     osc_channel = 11;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода FrontR ";
                     break;
                 case 2:
                     osc_channel = 8;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода LineL ";
                     break;
                 case 3:
                     osc_channel = 9;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода LineR ";
                     break;
                 case 4:
                     osc_channel = 3;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода mag/radio ";
                     break;
                 case 5:
                     osc_channel = 4;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода mag/phone ";
                     break;
                 case 6:
                     osc_channel = 7;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода ГГС ";
                     break;
                 case 7:
                     osc_channel = 5;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода Радио1 ";
                     break;
                 case 8:
                     osc_channel = 6;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода Радио2 ";
                     break;
                 case 9:
                     osc_channel = 12;
                     oscilloscope2.Caption = "Гарнитура инструктора. Контроль выхода W ";
                     break;
             }

             oscill_run = true;
             CallBackMy.callbackEventHandler("Передаваемые данные.", 8, 1, osc_channel, true);
             CallBackMy.callbackEventHandler("Передаваемые данные.", 9, 1, 0, true);
             oscilloscope2.ShowScope();
             oscilloscope2.UpdateScope();
             oscilloscope2.ClearScope();
             oscilloscope2.CellSize = 10;
             oscilloscope2.VerticalOffset1 = -500;
             oscilloscope2.AmplitudeScale1 = 50;
             oscilloscope2.Left = loc_x;
             oscilloscope2.Top = loc_y;

         }

         private void button20_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 27, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button21_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 27, false);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button22_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 29, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button23_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 29, false);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button24_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 2, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button25_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 2, false);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button26_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 5, true);
             }
             else
             {
                 label7.Visible = true;
             }
         }

         private void button27_Click(object sender, EventArgs e)
         {  
             if (!oscill_run)
             {
                 CallBackMy.callbackEventHandler("Передаваемые данные.", 11, 0, 5, false);
             }
             else
             {
                 label7.Visible = true;
             }
         }

    
        }

        public static class CallBackMy2
        {
            public delegate void callbackEvent(string what, int par1, int par2, ushort par3, bool end_run);
            public static callbackEvent callbackEventHandler;
        }
    }
