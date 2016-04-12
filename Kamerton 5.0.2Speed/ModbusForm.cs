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

// Функции протокола MODBUS

//  res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);                  // 01 Считать бит  false или true по адресу 00000 - 09999
//  res = myProtocol.writeCoil(slave, startCoil, true);                               // 05   Записать бит false или true по адресу 00000 - 09999
//  res = myProtocol.forceMultipleCoils(slave, startCoil, coilVals, numCoils);        // 15 (0F) Записать бит false или true  по адресу 0-9999 
//  res = myProtocol.readInputDiscretes(slave, startCoil, coilArr, numCoils);         // 02  Считать бит  0 или 1 по адресу 10000 - 19999
//  res = myProtocol.readInputRegisters(slave, startRdReg, readVals, numRdRegs);      // 04  Считать бит  0 или 1 по адресу 30000 - 39999
//  res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);   // 03  Считать число из регистров по адресу  40000 -49999
//  res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs); // 16 (10Hex)Записать в регистры число по адресу 40000 -49999
//  res = myProtocol.writeSingleRegister(int slaveAddr,int regAddr, ushort regVal)    // 06  Записать в регистр число по адресу 40000 -49999


namespace KamertonTest
{

	public partial class Form1 : Form
		{
		
	   #region Variables and objects     
		// Осциллограф
		/// <summary>
		/// Timer to update terminal textbox at fixed interval.
		/// </summary>
		//private System.Windows.Forms.Timer formUpdateTimer = new System.Windows.Forms.Timer();

		/// <summary>
		/// SerialPort object.
		/// </summary>
		//private SerialPort serialPort = new SerialPort();

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
		private float[] channels = new float[9] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

		/// <summary>
		/// Oscilloscope for channels 1, 2 and 3.
		/// </summary>
		//private Oscilloscope oscilloscope123 = Oscilloscope.CreateScope("Oscilloscope/Oscilloscope_settings.ini", "");

		/// <summary>
		/// Oscilloscope for channels 4, 5 and 6.
		/// </summary>
	   // private Oscilloscope oscilloscope456 = Oscilloscope.CreateScope("Oscilloscope/Oscilloscope_settings.ini", "");

		/// <summary>
		/// Oscilloscope for channels 7, 8 and 9.
		/// </summary>
	   // private Oscilloscope oscilloscope789 = Oscilloscope.CreateScope("Oscilloscope/Oscilloscope_settings.ini", "");

		/// <summary>
		/// CSV file writer.
		/// </summary>
		//private CsvFileWriter csvFileWriter = null;

		#endregion

		#region Set_variables
	  	    private Oscilloscope oscilloscope1 = Oscilloscope.CreateScope("Oscilloscope/Oscilloscope_settings.ini", "");

			private MbusMasterFunctions myProtocol;
			private int slave;
			private int startCoil;
			private int numCoils;
			private int startWrReg;
			private int numRdRegs;
			private int startRdReg;
			private int res;
			private int loc_x             = 0;
			private int loc_y             = 0;
			bool All_Table_Read           = false;                                   // Признак выполнения заполнения таблицы
			bool Insrt_Table_Read         = false;                                   // Признак выполнения заполнения таблицы
			bool Disp_Table_Read          = false;                                   // Признак выполнения заполнения таблицы
			bool Mtt_Table_Read           = false;                                   // Признак выполнения заполнения таблицы
			bool Mic_Table_Read           = false;                                   // Признак выполнения заполнения таблицы
			bool Ggs_Table_Read           = false;                                   // Признак выполнения заполнения таблицы
			bool Rad1_Table_Read          = false;                                   // Признак выполнения заполнения таблицы
			bool Rad2_Table_Read          = false;                                   // Признак выполнения заполнения таблицы
			bool Button_All_Test_Stop     = false;                                   // Признак для управления кнопкой "Стоп"
			bool All_Test_run             = false;                                   // Признак выполнения общего теста 
			bool Test_MODBUS              = false;                                   // Признак выполнения контроля связи MODBUS
			bool MODBUS_SUCCESS           = false;                                   // Признак исправности связи MODBUS
			bool Com2_SUCCESS             = false;                                   // Признак исправности связи Com2
			bool serial_connect_ok        = false;                                   // Признак успешного подключения serial_connect
			bool serial2_connect_ok       = false;                                   // Признак успешного подключения serial2_connect
			bool byte_set_ready           = false;                                   // Признак готовности информации о состоянии регистров Аудио-1
			bool Status_Rele_ready        = false;                                   // Признак готовности информации о состоянии реле 
			bool byte_set_run             = false;                                   // Признак выполнения программы сканирования регистров
			bool Serial_COM2              = false;                                   // Признак какой серийный порт подключен USB 0  или RS232
			bool Serial_COM2_Enable       = false;                                   // Признак наличия серийного КОМ порта
			bool Oscilloscope_run         = false;                                   // Признак передачи информации в осциллограф
            bool radio_on                 = false;                                   // Признак включения радиопередачи
            bool radio_off                = false;                                   // Признак выключения радиопередачи
            bool ggs_mute_on              = false;                                   // Признак включения ggs_mute
            bool ggs_mute_off             = false;                                   // Признак выключения ggs_mute
            bool set_display              = false;                                   // Признак уровня яркости
            bool set_sound_level          = false;                                   // Признак установки уровня сигнала
            bool  Message_show            = true;                                    // Разрешить показ сообщения 
            bool disp_mks                 = true;                                    // Признак вывода мкс или вольт
            short tempK                   = 0;
            int tempK_lev                 = 0;
            string s1_disp                = "";                                      //
            string s2_disp                = "";                                      //  
            ushort[] porog_min_all        = new ushort[200];
			ushort[] porog_max_all        = new ushort[200];
			ushort[] readVals_all         = new ushort[200];
			ushort[] readVals_byte        = new ushort[145];
			ushort[] readVolt_all         = new ushort[200];
			ushort[] table_porog          = new ushort[300];
			bool[] coilArr_all            = new bool[200];
			bool[] coil_Button            = new bool[64];
			bool[] coilArr                = new bool[64];
			string binaryResult           = "";
			bool[] Dec_bin                = new bool[64];
			bool[] coilArr_Status_Rele    = new bool[64];
			bool[] coil_Status_Rele1_8    = new bool[10];
			bool[] coil_Status_Rele9_16   = new bool[10];
			bool[] coil_Status_Rele17_24  = new bool[10];
			bool[] coil_Status_Rele25_32  = new bool[10];
			bool[] coilArrFlag            = new bool[4];
			double s12_Power              = 0;
			double s12_GGS                = 0;
			double s12_Radio1             = 0;
			double s12_Radio2             = 0;
			double s12_Led_Mic            = 0;
			int decimalNum;
			Int64 binaryHolder;
			int Tab_index                 = 0;
			string fileName               = "";
			private int num_module_audio1 = 0;
			private int[] test_step       = new int[40];
			private int TestN;
			private int TestStep;
			private int TestRepeatCount;
			private int MaxTestCount = 0;
			private int Form_Height;
			private int Form_Width;
			float temp_disp;
			bool list_files                     = false;
			bool read_file                      = false;
			static string folderName            = @"C:\Audio log";
			static string folderFormatName      = @"C:\Program Files\SDA\SD Formatter\SDFormatter.exe";
			static string folderNotepadName     = @"C:\Program Files\Notepad++\Notepad++.exe";
			string pathString                   = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
			string pathStringSD                 = System.IO.Path.Combine(folderName, "SD");
			string pathStringHDD                = "";
			string pathStringINI                = System.IO.Path.Combine(folderName, "INI");
            string s_txt7                       = "";
			string s_txt8                       = "";
			string s_txt9                       = "";
			string s_txt48                      = "";
			string s_txt80                      = "";
			//string s_txt11                    = "";
			string data                         = "";
		 //   string data1                      = "";
			byte sampTrigThreshold1             = 0;
			byte sampTrigThreshold2             = 0;
          //  internal SaveFileDialog SaveFileDialog1;
			private BackgroundWorker _bw_modbus_test;
			private BackgroundWorker _bw_All_test;
			private BackgroundWorker _bw_All_test1;
			private BackgroundWorker _bw_set_byte;
			SerialPort currentPort = new SerialPort(File.ReadAllText("set_MODBUS_port.txt"), 57600, Parity.None, 8, StopBits.One);
			SerialPort ComPort2    = new SerialPort(File.ReadAllText("set_rs232.txt"), 115200, Parity.None, 8, StopBits.One);
		//	public event EventHandler<UserEventArgs> sendDataFromFormEvent_instr;     //Таблица , проверить применение





		#endregion
		public Form1()
		{
			InitializeComponent();
			_bw_modbus_test = new BackgroundWorker();
			_bw_modbus_test.WorkerSupportsCancellation = true;
			_bw_modbus_test.WorkerReportsProgress = true;
			_bw_modbus_test.DoWork += new DoWorkEventHandler(Bw_DoWork_MODBUS);
			_bw_modbus_test.RunWorkerCompleted += new RunWorkerCompletedEventHandler(Bw_RunWorkerCompleted);
			_bw_modbus_test.ProgressChanged += new ProgressChangedEventHandler(Bw_ProgressChanged);

			_bw_All_test = new BackgroundWorker();
			_bw_All_test.WorkerSupportsCancellation = true;
			_bw_All_test.WorkerReportsProgress = true;
			_bw_All_test.DoWork += new DoWorkEventHandler(Bw_DoWorkAll_Test);
			_bw_All_test.RunWorkerCompleted += new RunWorkerCompletedEventHandler(Bw_DoWorkAll_TestRunWorkerCompleted);
			_bw_All_test.ProgressChanged += new ProgressChangedEventHandler(Bw_DoWorkAll_TestProgressChanged);

			_bw_All_test1 = new BackgroundWorker();
			_bw_All_test1.WorkerSupportsCancellation = true;
			_bw_All_test1.WorkerReportsProgress = true;
			_bw_All_test1.DoWork += new DoWorkEventHandler(Bw_DoWorkAll_Test1);
			_bw_All_test1.RunWorkerCompleted += new RunWorkerCompletedEventHandler(Bw_DoWorkAll_Test1RunWorkerCompleted);
			_bw_All_test1.ProgressChanged += new ProgressChangedEventHandler(Bw_DoWorkAll_Test1ProgressChanged);

			_bw_set_byte = new BackgroundWorker();
			_bw_set_byte.WorkerSupportsCancellation = true;
			_bw_set_byte.WorkerReportsProgress = true;
			_bw_set_byte.DoWork += new DoWorkEventHandler(Bw_set_byte);
			_bw_set_byte.RunWorkerCompleted += new RunWorkerCompletedEventHandler(Bw_set_byteCompleted);
			_bw_set_byte.ProgressChanged += new ProgressChangedEventHandler(Bw_set_byteProgressChanged);

			CallBackMy.callbackEventHandler = new CallBackMy.callbackEvent(this.Reload);
			LoadListboxes();
		}

		private void Form1_Load(object sender, EventArgs e)
		{
			ToolTip1.SetToolTip(txtPollDelay, "Задержка в миллисекундах между двумя последовательными операциями Modbus, 0 для отключения");
			ToolTip1.SetToolTip(cmbRetry, "Сколько раз повторить операцию, если в первый раз не принят?");
			ToolTip1.SetToolTip(cmbSerialProtocol, "Выбор протокола COM: ASCII или RTU");
		
			cmbComPort.SelectedIndex        = 0;
			cmbParity.SelectedIndex         = 0;
			cmbStopBits.SelectedIndex       = 0;
			cmbDataBits.SelectedIndex       = 0;
			cmbBaudRate.SelectedIndex       = 5;
			cmbSerialProtocol.SelectedIndex = 0;
			cmbRetry.SelectedIndex          = 2;
	   
			cmbComPort2.SelectedIndex       = 0;
			cmbBaudRate2.SelectedIndex      = 12;
			cmbDataBits2.SelectedIndex      = 0;
			cmbParity2.SelectedIndex        = 0;
			cmbDataBits2.SelectedIndex      = 0;
			cmbStopBits2.SelectedIndex      = 0;

			ComPort2.Handshake           = Handshake.None;
			ComPort2.DataReceived       += new SerialDataReceivedEventHandler(ComPort2_DataReceived);
			//ComPort2.DataReceived     += new SerialDataReceivedEventHandler(sp_DataReceived);
			ComPort2.ReadTimeout         = 500;
			ComPort2.WriteTimeout        = 500;
			ComPort2.RtsEnable           = true;
			ComPort2.DtrEnable           = true;
			comboBox4.SelectedIndex      = 0;
			comboBox2.SelectedIndex      = 0;
			radioButton1.Checked         = true;                                        // Одноразовая проверка
		   // radioButton4.Checked       = true;                                        // Выбор частоты звукового сигнала
			serviceSet();                                                               // Выбор многократной проверки
			tabControl1.Selected += new TabControlEventHandler(TabControl1_Selected);   // Переключение вкладок
			button11.BackColor    = Color.Lime;
			button9.BackColor     = Color.LightSalmon;
			button9.Enabled       = false;
			label54.Visible       = false;
			checkBox1.Checked     = true;
			label87.Visible       = false;
			label148.Visible      = false;
			label138.Visible      = false;
			cTrigLevel1.Value     = 128;
			cTrigLevel2.Value     = 128;
			serial_connect();
			serial_connect2();
           // tempK_lev = short.Parse(textBox5.Text, CultureInfo.CurrentCulture)*5;                    // Установка уровня входного сигнала
			this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;//убираем рамки 
			this.WindowState     = FormWindowState.Maximized;//и только потом расширяем форму на весь экран 
			Form_Height          = this.Height; //считываем размеры нашей формы 
			Form_Width           = this.Width;
			textBox9.Text        = Form_Width.ToString() + " : " + Form_Height.ToString();//выводим результаты на label1 
			this.WindowState     = FormWindowState.Normal; //и не забываем вернуть форму в исходное положение 
			this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Sizable;//возвращаем рамки 

			loc_x = (Form_Width / 2) - 475;
			loc_y = (Form_Height / 2) - 295;
            timerVisualAll_Test.Stop();
 		}
    //    MemoryStream userInput = new MemoryStream();

		private void TabControl1_Selected(object sender, TabControlEventArgs e)
		{
			switch (e.TabPageIndex)
			{
				case 0:
			 
					if (Tab_index == 2)
					{
						Tab_index = 0;
						timer_byte_set.Stop();
						stop_bw_set_byte();
						stop_DoWorkAll_Test1();
						Thread.Sleep(300);
						while (byte_set_run) { };
						Thread.Sleep(100);
						startCoil = 8;                                                       // Управление питанием платы "Камертон"
						res = myProtocol.writeCoil(slave, startCoil, false);                 // Выключить питание платы "Камертон"
						Thread.Sleep(1000);
						start_test_modbus1();                                                // Включить сканирование MODBUS
					}
				   else
					{
					 
						if (!Button_All_Test_Stop)                                          // Если не запущено тестирование - выключить питание
						{
							button11.BackColor = Color.Lime;                                // 
							if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
							{
								//Thread.Sleep(1500);
								//timerPowerOff.Start();
								//Thread.Sleep(1500);                       
								startCoil = 8;                                              // Управление питанием платы "Камертон"
								res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
								Thread.Sleep(400);
							}

							if (!Test_MODBUS)                                               // если не запущено сканирование MODBUS
							{
								
								res = myProtocol.readCoils(slave, 12, coilArr, 1);          // Поверить что подключено USB или RS232
								if (coilArr[0] == false)
								{                                                           // Отобразить состояние кнопок USB или RS232
									button_USB0.Enabled = false;
									button_RS232.Enabled = true;
									button_RS232.BackColor = Color.LightSalmon;
									button_USB0.BackColor = Color.LightGreen;
									Serial_COM2 = false;
									label19.Text = ("USB 2");                               // Признак включения RS232
									label148.Visible = true;
									label138.Visible = false;

								}
								else
								{
									button_RS232.Enabled = false;
									button_USB0.Enabled = true;
									button_USB0.BackColor = Color.LightSalmon;
									button_RS232.BackColor = Color.LightGreen;
									Serial_COM2 = true;
									label19.Text = ("RS-232");                              // Признак включения RS232
									label148.Visible = false;
									label138.Visible = true;
								}
								
								start_test_modbus1();                                       // Включить сканирование MODBUS если оно не включено
							}
							else
							{
								// Ничего не нужно делать, все уже запущено

							}

						}
					}
						  
					break;
				case 1:

					if (Tab_index == 2)                                                    // Если включен режим set_byte - отключить
					{
						Tab_index = 1;
						timer_byte_set.Stop();
						stop_bw_set_byte();
						Thread.Sleep(300);
						while (byte_set_run) { };
						if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
						{
							//Thread.Sleep(1500);
							//timerPowerOff.Start();
							//Thread.Sleep(1500);                       
							startCoil = 8;                                              // Управление питанием платы "Камертон"
							res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
							Thread.Sleep(400);
						}
						button1.BackColor   = Color.LightGreen;
						button94.BackColor  = Color.LightGreen;
						button95.BackColor  = Color.LightGreen;
						button96.BackColor  = Color.LightGreen;
						button97.BackColor  = Color.LightGreen;
						button98.BackColor  = Color.LightGreen;
						button99.BackColor  = Color.LightGreen;
						button100.BackColor = Color.LightGreen;
						button101.BackColor = Color.LightGreen;
						Thread.Sleep(100);
					}
					else
					{
						if (!Button_All_Test_Stop)
						{
							button11.BackColor = Color.Lime;
							if (coilArr_Status_Rele[7] == false)
							{
								timer1.Stop();
								stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
								Thread.Sleep(1500);
								if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
								{
									//Thread.Sleep(1500);
									//timerPowerOff.Start();
									//Thread.Sleep(1500);                       
									startCoil = 8;                                              // Управление питанием платы "Камертон"
									res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
									Thread.Sleep(400);
								}
								button1.BackColor   = Color.LightGreen;
								button94.BackColor  = Color.LightGreen;
								button95.BackColor  = Color.LightGreen;
								button96.BackColor  = Color.LightGreen;
								button97.BackColor  = Color.LightGreen;
								button98.BackColor  = Color.LightGreen;
								button99.BackColor  = Color.LightGreen;
								button100.BackColor = Color.LightGreen;
								button101.BackColor = Color.LightGreen;
 							}

							if(!Test_MODBUS)
								{
									timer1.Stop();
									stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
									Thread.Sleep(1500);
									if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
									{
										//Thread.Sleep(1500);
										//timerPowerOff.Start();
										//Thread.Sleep(1500);                       
										startCoil = 8;                                              // Управление питанием платы "Камертон"
										res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
										Thread.Sleep(400);
									}

									button1.BackColor = Color.LightGreen;
									button95.BackColor = Color.LightGreen;
									button96.BackColor = Color.LightGreen;
									button97.BackColor = Color.LightGreen;
									button98.BackColor = Color.LightGreen;
									button99.BackColor = Color.LightGreen;
									button100.BackColor = Color.LightGreen;
									button101.BackColor = Color.LightGreen;
								}
						}
					}
					break;

				case 2:
					Tab_index = 2;
					if (!Button_All_Test_Stop)
					{
						timer1.Stop();
						stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
						Thread.Sleep(100);
						startCoil = 8;                                                                   // Управление питанием платы Аудио-1
						Thread.Sleep(1000);
						start_bw_set_byte();
					}

					break;

				case 3:

				//	button_Find_Com2.Enabled = false;
					if (Tab_index == 2)
					{
						Tab_index = 3;
						timer_byte_set.Stop();
						stop_bw_set_byte();
						Thread.Sleep(300);
						while (byte_set_run) { };
						if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
						{
							//Thread.Sleep(1500);
							//timerPowerOff.Start();
							//Thread.Sleep(1500);                       
							startCoil = 8;                                              // Управление питанием платы "Камертон"
							res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
							Thread.Sleep(400);
						}                                                             // Включить сканирование MODBUS 
						toolStripStatusLabel8.Text = ("Установка параметров подключения Камертон 5.0");

					}
	
					else
					{
						if (!Button_All_Test_Stop)
						{
							button11.BackColor = Color.Lime;
							if (coilArr_Status_Rele[7] == false)
							{
							timer1.Stop();
							stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
							Thread.Sleep(1000);
							if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
							{
								//Thread.Sleep(1500);
								//timerPowerOff.Start();
								//Thread.Sleep(1500);                       
								startCoil = 8;                                              // Управление питанием платы "Камертон"
								res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
								Thread.Sleep(400);
							}
								res = myProtocol.readCoils(slave, 12, coilArr, 1);            // 01 Считать бит false или true по адресу 12
								if(coilArr[0] == false)
									{
									button_USB0.Enabled = false;
									button_RS232.Enabled = true;
									button_RS232.BackColor = Color.LightSalmon;
									button_USB0.BackColor = Color.LightGreen;
									Serial_COM2 = false;
									label19.Text = ("USB 2");
									}
								else
									{
									button_RS232.Enabled = false;
									button_USB0.Enabled = true;
									button_USB0.BackColor = Color.LightSalmon;
									button_RS232.BackColor = Color.LightGreen;
									Serial_COM2 = true;
									label19.Text = ("RS-232");                         // Признак включения RS232
									}
	 
							}
							toolStripStatusLabel8.Text = ("Установка параметров подключения Камертон 5.0");

							if (!Test_MODBUS)
							{
								if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
								{
									//Thread.Sleep(1500);
									//timerPowerOff.Start();
									//Thread.Sleep(1500);                       
									startCoil = 8;                                              // Управление питанием платы "Камертон"
									res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
									Thread.Sleep(400);
								}                                                      // Включить сканирование MODBUS если оно не включено
							}
						}
					}
					

					break;
				case 4:


				  
					if (Tab_index == 2)
					{
						Tab_index = 4;
						timer_byte_set.Stop();
						stop_bw_set_byte();
						Thread.Sleep(300);
						while (byte_set_run) { };
						timer1.Stop();
						Thread.Sleep(1500);
						if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
						{
							//Thread.Sleep(1500);
							//timerPowerOff.Start();
							//Thread.Sleep(1500);                       
							startCoil = 8;                                              // Управление питанием платы "Камертон"
							res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
							Thread.Sleep(400);
						}
						toolStripStatusLabel8.Text = ("Меню просмотра файлов");
					}
					else
					{
						if (!Button_All_Test_Stop)
						{
							button11.BackColor = Color.Lime;
							stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
							Thread.Sleep(500);
							timer1.Stop();
							toolStripStatusLabel8.Text = ("Меню просмотра файлов");
							if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
							{
								//Thread.Sleep(1500);
								//timerPowerOff.Start();
								//Thread.Sleep(1500);                       
								startCoil = 8;                                              // Управление питанием платы "Камертон"
								res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
								Thread.Sleep(400);
							}

						}
						else
						{
							timer1.Stop();
							stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
							Thread.Sleep(500);
							Thread.Sleep(1500);
							if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
							{
								//Thread.Sleep(1500);
								//timerPowerOff.Start();
								//Thread.Sleep(1500);                       
								startCoil = 8;                                              // Управление питанием платы "Камертон"
								res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
								Thread.Sleep(400);
							}

						}

					}

					break;

				case 5:

					if(Tab_index == 2)
						{
							Tab_index = 5;
							timer_byte_set.Stop();
							stop_bw_set_byte();
							Thread.Sleep(300);
							while(byte_set_run) { };
							Thread.Sleep(100);
							if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
							{
								//Thread.Sleep(1500);
								//timerPowerOff.Start();
								//Thread.Sleep(1500);                       
								startCoil = 8;                                              // Управление питанием платы "Камертон"
								res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
								Thread.Sleep(400);
							}
							toolStripStatusLabel8.Text = ("Осциллограф");
						}

					else
						{
						if(!Button_All_Test_Stop)
							{
							button11.BackColor = Color.Lime;
							if(coilArr_Status_Rele[7] == false)
								{
								timer1.Stop();
								stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
								Thread.Sleep(1000);
								if (coilArr_Status_Rele[7] != false)                            // Проверить, может уже отключено питание. Нет - отключить                
								{
									//Thread.Sleep(1500);
									//timerPowerOff.Start();
									//Thread.Sleep(1500);                       
									startCoil = 8;                                              // Управление питанием платы "Камертон"
									res = myProtocol.writeCoil(slave, startCoil, false);        // Выключить питание платы "Камертон"
									Thread.Sleep(400);
								}    
								}
							toolStripStatusLabel8.Text = ("Осциллограф");

							if(!Test_MODBUS)
								{
								//start_test_modbus1();                                                         // Включить сканирование MODBUS если оно не включено
								}
							}
						}

					break;

				default:
					break;
			}
		}
	
		private void serviceSet()
		{
			checkBoxSenAll.Checked = true;
		}

		#region Serial port1

		private void LoadListboxes()
			{
				cmbComPort.Items.Clear();
				cmbComPort2.Items.Clear();
				ComPort2.PortName = File.ReadAllText("set_rs232.txt");
				foreach (string PortName in System.IO.Ports.SerialPort.GetPortNames())
					{
						cmbComPort.Items.Add(PortName);
						cmbComPort2.Items.Add(PortName);

						if (PortName == ComPort2.PortName)
						{
						 //   MessageBox.Show("COM порт №2 найден", "", MessageBoxButtons.OK, MessageBoxIcon.Hand);
							Serial_COM2_Enable = true;
						}
					}
				cmbComPort.SelectedIndex  = 0;
				cmbComPort2.SelectedIndex = 0;
			}
		private void serial_connect()
		{
			if ((myProtocol == null))
			{
				try
				{
					if ((cmbSerialProtocol.SelectedIndex == 0))
						myProtocol = new MbusRtuMasterProtocol(); // RTU
					else
						myProtocol = new MbusAsciiMasterProtocol(); // ASCII
				}
				catch (OutOfMemoryException ex)
				{
					lblResult.Text = ("Не удалось создать экземпляр класса серийного протокола! Ошибка была " + ex.Message);
					return;
				}
			}
			else // already instantiated, close protocol, reinstantiate
			{
				if (myProtocol.isOpen()) myProtocol.closeProtocol();
				myProtocol = null;
				try
				{
					if ((cmbSerialProtocol.SelectedIndex == 0))
						myProtocol = new MbusRtuMasterProtocol(); // RTU
					else
						myProtocol = new MbusAsciiMasterProtocol(); // ASCII
				}
				catch (OutOfMemoryException ex)
				{
					lblResult.Text = ("Не удалось создать экземпляр класса серийного протокола! Ошибка была " + ex.Message);
					return;
				}
			}

			short[] readVals = new short[125];
			int retryCnt;
			int pollDelay;
			int timeOut;
			int baudRate;
			int parity;
			int dataBits;
			int stopBits;
			int res;
			try
			{
				retryCnt = int.Parse(cmbRetry.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				retryCnt = 0;
			}
			try
			{
				pollDelay = int.Parse(txtPollDelay.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				pollDelay = 0;
			}
			try
			{
				timeOut = int.Parse(txtTimeout.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				timeOut = 1000;
			}
			try
			{
				baudRate = int.Parse(cmbBaudRate.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				baudRate = 57600;
			}
			switch (cmbParity.SelectedIndex)
			{
				default:
				case 0:
					parity = MbusSerialMasterProtocol.SER_PARITY_NONE;
					break;
				case 1:
					parity = MbusSerialMasterProtocol.SER_PARITY_EVEN;
					break;
				case 2:
					parity = MbusSerialMasterProtocol.SER_PARITY_ODD;
					break;
			}
			switch (cmbDataBits.SelectedIndex)
			{
				default:
				case 0:
					dataBits = MbusSerialMasterProtocol.SER_DATABITS_8;
					break;
				case 1:
					dataBits = MbusSerialMasterProtocol.SER_DATABITS_7;
					break;
			}
			switch (cmbStopBits.SelectedIndex)
			{
				default:
				case 0:
					stopBits = MbusSerialMasterProtocol.SER_STOPBITS_1;
					break;
				case 1:
					stopBits = MbusSerialMasterProtocol.SER_STOPBITS_2;
					break;
			}
			myProtocol.timeout = timeOut;
			myProtocol.retryCnt = retryCnt;
			myProtocol.pollDelay = pollDelay;
			cmbComPort.Text = currentPort.PortName;
			res = ((MbusSerialMasterProtocol)(myProtocol)).openProtocol(cmbComPort.Text, baudRate, dataBits, stopBits, parity);
			if ((res == BusProtocolErrors.FTALK_SUCCESS))
			{
				lblResult.Text = ("Последовательный порт Serial Modbus USB 1 успешно открыт с параметрами:\r\n "
						 + (cmbComPort.Text + (", "
						 + (baudRate + (" baud, "
						 + (dataBits + (" data bits, "
						 + (stopBits + (" stop bits, parity " + parity + "\r\n\r\n")))))))));

				toolStripStatusLabel3.Text = (cmbComPort.Text + (", " + (baudRate + (" baud |"))));
				cmdOpenSerial.Enabled = false;                                                    // Отключить кнопку "Открыть сериал"
				Close_Serial.Enabled = true;                                                      // Включить кнопку "Закрыть сериал"  
				serial_connect_ok = true;                                                         // Признак успешного подключения serial_connect
				start_test_modbus1();                                                             // Включить сканирование MODBUS
			}
			else
			{
				serial_connect_ok = false;                                                   // Признак успешного подключения serial_connect
				stop_test_modbus1();                                                         // Остановить сканирование  MODBUS 
				lblResult.Text = ("Не удалось открыть протокол, ошибка была:\r\n " + BusProtocolErrors.getBusProtocolErrorText(res));
				toolStripStatusLabel3.Text = ("СОМ порт не подключен |");
			}
		}
		private void Listbox_check_Click(object sender, EventArgs e)          // Кнопка "USB1" Обновить список файлов
		{
			LoadListboxes();
		}
		private void cmdOpenSerial_Click(object sender, EventArgs e)
		{
			stop_test_modbus1();
			File.WriteAllText("set_MODBUS_port.txt", cmbComPort.SelectedItem.ToString(), Encoding.GetEncoding("UTF-8"));
			if (File.Exists("set_MODBUS_port.txt"))
			{
				SerialPort currentPort = new SerialPort(File.ReadAllText("set_MODBUS_port.txt"), 57600, Parity.None, 8, StopBits.One);
				lblResult.Text += currentPort.PortName;
			}
			else
			{
				SerialPort currentPort = new SerialPort("COM2", 57600, Parity.None, 8, StopBits.One);
				lblResult.Text += currentPort.PortName;
			}

			if ((myProtocol == null))
			{
				try
				{
					if ((cmbSerialProtocol.SelectedIndex == 0))
						myProtocol = new MbusRtuMasterProtocol(); // RTU
					else
						myProtocol = new MbusAsciiMasterProtocol(); // ASCII
				}
				catch (OutOfMemoryException ex)
				{
					lblResult.Text = (" Ошибка была" + ex.Message + "\r\n");
					lblResult.Text += ("Не удалось создать экземпляр класса серийного протокола!" + ex.Message);
					return;
				}
			}
			else // already instantiated, close protocol, reinstantiate
			{
				if (myProtocol.isOpen()) myProtocol.closeProtocol();
				myProtocol = null;
				try
				{
					if ((cmbSerialProtocol.SelectedIndex == 0))
						myProtocol = new MbusRtuMasterProtocol(); // RTU
					else
						myProtocol = new MbusAsciiMasterProtocol(); // ASCII
				}
				catch (OutOfMemoryException ex)
				{
					lblResult.Text = (" Ошибка была" + ex.Message + "\r\n");
					lblResult.Text += ("Не удалось создать экземпляр класса серийного протокола!" + ex.Message);
					return;
				}
			}
			// Здесь мы настроим протокол
			int retryCnt;
			int pollDelay;
			int timeOut;
			int baudRate;
			int parity;
			int dataBits;
			int stopBits;
			int res;
			try
			{
				retryCnt = int.Parse(cmbRetry.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				retryCnt = 2;
			}
			try
			{
				pollDelay = int.Parse(txtPollDelay.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				pollDelay = 10;
			}
			try
			{
				timeOut = int.Parse(txtTimeout.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				timeOut = 1000;
			}
			try
			{
				baudRate = int.Parse(cmbBaudRate.Text, CultureInfo.CurrentCulture);
			}
			catch (Exception)
			{
				baudRate = 115200;
			}
			switch (cmbParity.SelectedIndex)
			{
				default:
				case 0:
					parity = MbusSerialMasterProtocol.SER_PARITY_NONE;
					break;
				case 1:
					parity = MbusSerialMasterProtocol.SER_PARITY_EVEN;
					break;
				case 2:
					parity = MbusSerialMasterProtocol.SER_PARITY_ODD;
					break;
			}
			switch (cmbDataBits.SelectedIndex)
			{
				default:
				case 0:
					dataBits = MbusSerialMasterProtocol.SER_DATABITS_8;
					break;
				case 1:
					dataBits = MbusSerialMasterProtocol.SER_DATABITS_7;
					break;
			}
			switch (cmbStopBits.SelectedIndex)
			{
				default:
				case 0:
					stopBits = MbusSerialMasterProtocol.SER_STOPBITS_1;
					break;
				case 1:
					stopBits = MbusSerialMasterProtocol.SER_STOPBITS_2;
					break;
			}
			myProtocol.timeout = timeOut;
			myProtocol.retryCnt = retryCnt;
			myProtocol.pollDelay = pollDelay;
			// Примечание: В следующем варианте требуется как объект myProtocol объявлен
			// Как суперкласс MbusSerialMasterProtocol. Таким образом myProtocol может
			// Представляют различные типы протоколов.
			res = ((MbusSerialMasterProtocol)(myProtocol)).openProtocol(File.ReadAllText("set_MODBUS_port.txt"), baudRate, dataBits, stopBits, parity);
			//res = ((MbusSerialMasterProtocol)(myProtocol)).openProtocol(cmbComPort.Text, baudRate, dataBits, stopBits, parity);
			if ((res == BusProtocolErrors.FTALK_SUCCESS))
			{
				lblResult.Text = ("Последовательный порт успешно открыт с параметрами:\r\n"
						 + (cmbComPort.Text + (", "
						 + (baudRate + (" baud, "
						 + (dataBits + (" data bits, "
						 + (stopBits + (" stop bits, parity " + parity)))))))));
				Close_Serial.Enabled = true;
				toolStripStatusLabel3.Text = (cmbComPort.Text + (", " + (baudRate + (" baud |"))));
				serial_connect_ok = true;                                                        // Признак успешного подключения serial_connect
				cmdOpenSerial.Enabled = false;                                                   // Отключить кнопку "Открыть сериал"
				Close_Serial.Enabled = true;                                                     // Включить кнопку "Закрыть сериал"  
				start_test_modbus1();
			}
			else
			{
				serial_connect_ok = false;                                                   // Признак успешного подключения serial_connect
				stop_test_modbus1();                                                         // Остановить сканирование  MODBUS 
				lblResult.Text = (" Ошибка была: " + BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n");
				lblResult.Text = ("Не удалось открыть протокол!");
			}
		}
		private void Close_Serial_Click(object sender, EventArgs e)
		{
			if ((myProtocol != null))
			{
				// Close protocol and serial port
				stop_test_modbus1();                                                             // Остановить сканирование  MODBUS 
				timer1.Stop();
				myProtocol.closeProtocol();
				lblResult.Text = "Протокол закрыт";
				lblResult.Refresh();
				toolStripStatusLabel3.Text = ("");
				toolStripStatusLabel1.Text = "  MODBUS ЗАКРЫТ   ";
				toolStripStatusLabel1.BackColor = Color.Red;
				toolStripStatusLabel3.Text = ("");
				statusStrip1.Refresh();
				serial_connect_ok = false;                                                       // Признак успешного подключения serial_connect
				cmdOpenSerial.Enabled = true;                                                    // Включить кнопку "Открыть сериал"
				Close_Serial.Enabled = false;                                                    // Отключить кнопку "Закрыть сериал"  
			}
		}

		#endregion

		#region Serial port2
		private void serial_connect2()
			{

				if (Serial_COM2_Enable)
				{

					// Переключить КОМ порт на USB или RS232
					// ...............

					// ...............

					// ComPort2.PortName = File.ReadAllText("set_rs232.txt");

					string temp;
					//  ComPort2.PortName = ((string)cmbComPort2.SelectedItem);     // Отключил, прочитал из файла
					ComPort2.Parity = (Parity)Enum.Parse(typeof(Parity), (string)cmbParity2.SelectedItem);
					temp = ((string)cmbBaudRate2.SelectedItem);
					temp = temp.ToString();
					ComPort2.BaudRate = (int.Parse(temp));
					ComPort2.StopBits = ((StopBits)Enum.Parse(typeof(StopBits), (string)cmbStopBits2.SelectedItem));
					temp = ((string)cmbDataBits2.SelectedItem);
					temp = temp.ToString();
					ComPort2.DataBits = (int.Parse(temp));

					if (ComPort2.PortName == null)
					{
						//return false;
					}

					// Open serial port
					CloseSerialPort2();
					try
					{
						ComPort2.Open();
						Open_ComPort2.Enabled = false;                   // Кнопка 
						Close_Serial2.Enabled = true;                    // Кнопка  

						lblResult.Text += ("Serial №2  USB 2 или RS 232 успешно открыт с параметрами:\r\n "
							 + (ComPort2.PortName + (", "
							 + (ComPort2.BaudRate + (" baud, "
							 + (ComPort2.DataBits + (" data bits, "
							 + (ComPort2.StopBits + (" stop bits, parity " + ComPort2.Parity + "\r\n")))))))));

						toolStripStatusLabel5.Text = (" Serial №2 = " + ComPort2.PortName + (", " + (ComPort2.BaudRate + (" baud |"))));
						label_COM2.Text = (ComPort2.PortName);
						// sampleCounter.Reset();
						toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5 УСТАНОВЛЕНА !");     // Обработка ошибки.
						toolStripStatusLabel4.BackColor = Color.Lime;

						if (Serial_COM2)
						{
							label19.Text = ("RS-232");
						}
						else
						{
							label19.Text = ("USB 2");
						}
						serial2_connect_ok = true;
					 
					}
					catch (InvalidOperationException)
					{
					   // MessageBox.Show("COM порт №2 не найден", "Error", MessageBoxButtons.OK, MessageBoxIcon.Hand);
						serial2_connect_ok = false;
						toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
						toolStripStatusLabel4.BackColor = Color.Red;
					}
				}
				else
				{
					MessageBox.Show("COM порт №2 не найден", "Error", MessageBoxButtons.OK, MessageBoxIcon.Hand);
					serial2_connect_ok = false;
					toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
					toolStripStatusLabel4.BackColor = Color.Red;
				}



			}
		private void CloseSerialPort2()
		{
			try
			{
				ComPort2.Close();
				Com2_SUCCESS = false;
				toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
				toolStripStatusLabel4.BackColor = Color.Red;
				serial2_connect_ok = false;                                                        // Признак успешного подключения serial_connect
			}
			catch { }
		   // this.Text = Assembly.GetExecutingAssembly().GetName().Name + " (Port Closed)";
		}
		private void SendSerialPort2(char c)
		{
			try
			{
				ComPort2.Write(new char[] { c }, 0, 1);
			}
			catch { }
		}

		private void button_USB0_Click(object sender, EventArgs e)
		{
			startWrReg             = 120;                                                                // 
			res                    = myProtocol.writeSingleRegister(slave, startWrReg, 42);                     // Реле переключения USB 
			button_USB0.Enabled    = false;
			button_RS232.Enabled   = true;
			startWrReg = 120;                                                                // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 36);                     // Программный сброс
			Thread.Sleep(500);
			system_ready();
			//startWrReg = 120;                                                                // 
			//res = myProtocol.writeSingleRegister(slave, startWrReg, 28);                     // Очистить буфер serial2 
			button_RS232.BackColor = Color.LightSalmon;
			button_USB0.BackColor = Color.LightGreen;
			Serial_COM2 = false;
			label19.Text = ("USB 2");
			label148.Visible = true;
			label138.Visible = false;

		}
		private void button_RS232_Click(object sender, EventArgs e)
		{
			startWrReg             = 120;                                                                // 
			res                    = myProtocol.writeSingleRegister(slave, startWrReg, 43);                     // Реле переключения RS232  
			button_RS232.Enabled   = false;
			button_USB0.Enabled    = true;
			startWrReg = 120;                                                                // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 36);                     // Программный сброс
			Thread.Sleep(500);
			system_ready();
			//startWrReg = 120;                                                                // 
			//res = myProtocol.writeSingleRegister(slave, startWrReg, 28);                     // Очистить буфер serial2 
			button_USB0.BackColor = Color.LightSalmon;
			button_RS232.BackColor = Color.LightGreen;
			Serial_COM2 = true;
			label19.Text = ("RS-232");
			label148.Visible = false;
			label138.Visible = true;
		}

		private void Open_ComPort2_Click(object sender, EventArgs e)
		{

			// Переключить КОМ порт на USB или RS232
			// ...............

			// ...............
			string temp;
			ComPort2.PortName = ((string)cmbComPort2.SelectedItem);
			ComPort2.Parity = (Parity)Enum.Parse(typeof(Parity), (string)cmbParity2.SelectedItem);
			temp = ((string)cmbBaudRate2.SelectedItem);
			temp = temp.ToString();
			ComPort2.BaudRate = (int.Parse(temp));
			ComPort2.StopBits = ((StopBits)Enum.Parse(typeof(StopBits), (string)cmbStopBits2.SelectedItem));
			temp = ((string)cmbDataBits2.SelectedItem);
			temp = temp.ToString();
			ComPort2.DataBits = (int.Parse(temp));
			try
			{
				ComPort2.Open();
				Open_ComPort2.Enabled = false;                            // Кнопка
				Close_Serial2.Enabled = true;                             // Кнопка
				lblResult.Text = ("Последовательный порт Serial №2  USB 0 или RS 232 успешно открыт с параметрами:\r\n"
			  + (ComPort2.PortName + (", "
			  + (ComPort2.BaudRate + (" baud, "
			  + (ComPort2.DataBits + (" data bits, "
			  + (ComPort2.StopBits + (" stop bits, parity " + ComPort2.Parity + "\r\n")))))))));

				toolStripStatusLabel5.Text = (" COM №2 = " + ComPort2.PortName + (", " + (ComPort2.BaudRate + (" baud |"))));
				label_COM2.Text = (ComPort2.PortName);
				toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5 УСТАНОВЛЕНА !");     // Обработка ошибки.
				toolStripStatusLabel4.BackColor = Color.Lime;
				File.WriteAllText("set_rs232.txt", cmbComPort2.SelectedItem.ToString(), Encoding.GetEncoding("UTF-8"));

				//if(Serial_COM2)
				//    {
				//    label19.Text = ("RS-232");
				//    }
				//else
				//    {
				//    label19.Text = ("USB 2");
				//    }
				 serial2_connect_ok = true;
				
			}
			catch (InvalidOperationException)
			{
				serial2_connect_ok = false;
				toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
				toolStripStatusLabel4.BackColor = Color.Red;
			}
	
		}
		private void Close_Serial2_Click(object sender, EventArgs e)
		{
			CloseSerialPort2();
			Open_ComPort2.Enabled           = true;                                                 // Включить кнопку "Открыть сериал"
			Close_Serial2.Enabled           = false;
			lblResult.Text                  = "COM Порт № 2 закрыт";
			toolStripStatusLabel5.Text      = ("| COM №2 закрыт");
			toolStripStatusLabel4.Text      = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
			toolStripStatusLabel4.BackColor = Color.Red;
			serial2_connect_ok              = false;                                                // Признак успешного подключения serial_connect
			Open_ComPort2.Enabled           = true;                                                 // Включить кнопку "Открыть сериал"
			// Close_Serial.Enabled         = false;  
			lblResult.Refresh();
		}
		private void button_Find_Com2_Click(object sender, EventArgs e)
		{
			ComPort2.Close();
			button_Find_Com2.Enabled = false;
			lblResult.Text = "";
			lblResult.Refresh();
			//stop_test_modbus1();                                                           // Отключить сканирование MODBUS 
			startCoil = 39;
			res = myProtocol.writeCoil(slave, startCoil, false);
			Thread.Sleep(400);
			startWrReg = 120;                                                                // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 36);                     // Программный сброс
			Thread.Sleep(500);
			system_ready();
			startWrReg = 120;                                                                // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 28);                     // Очистить буфер serial2 
			Thread.Sleep(100);
			startWrReg = 120;                                                                // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 29);                     // Поиск КОМ 2 Автоматически  
			Thread.Sleep(200);
			SetComPort2();
			Thread.Sleep(400);
			//timer_param_set.Start();
		}

		private void SetComPort2()
			{
			  try
				{
				if(ComPort2.IsOpen) ComPort2.Close();
				string[] ports = SerialPort.GetPortNames();//creat array with all port names on computer
				foreach(string port in ports)//for every port (string) from the ports array.
					{
					ComPort2 = new SerialPort(port, 115200, Parity.None, 8, StopBits.One);
					// richTextBox2.Text += (port);//debuging
					// MessageBox.Show(port);//debuging
					if(DetectArduino())//check is the arduino is on this current port
						{
						  // richTextBox2.Text += ("   Arduino Found!!! \r\n");//debuging
						  lblResult.Text = ("Обнаружен Serial  " + ComPort2.PortName + "\r\n");
						  // MessageBox.Show("Arduino Found!!!");//debuging
						  ComPort2.Open();
						  Open_ComPort2.Enabled = false;                          // Кнопка
						  Close_Serial2.Enabled = true;                           // Кнопка 
						  lblResult.Text += ("Serial №2  USB 2 или RS 232 успешно открыт с параметрами:\r\n "
							   + (ComPort2.PortName + (", "
							   + (ComPort2.BaudRate + (" baud, "
							   + (ComPort2.DataBits + (" data bits, "
							   + (ComPort2.StopBits + (" stop bits, parity " + ComPort2.Parity + "\r\n")))))))));

						  toolStripStatusLabel5.Text = (" Serial №2 = " + ComPort2.PortName + (", " + (ComPort2.BaudRate + (" baud |"))));
						  label_COM2.Text = (ComPort2.PortName);
						  toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5 УСТАНОВЛЕНА !");     // Обработка ошибки.
						  toolStripStatusLabel4.BackColor = Color.Lime;
						  File.WriteAllText("set_rs232.txt", cmbComPort2.SelectedItem.ToString(), Encoding.GetEncoding("UTF-8"));
						  if (Serial_COM2)
						  {
							  label19.Text = ("RS-232");
						  }
						  else
						  {
							  label19.Text = ("USB 2");
						  }
						  serial2_connect_ok = true;
						  Close_Serial.BackColor = Color.White;
						  Close_Serial.Refresh();
						  label87.Visible = true;
						break;
						}
					else
						{
							lblResult.Text = "Serial № 2 не найден \r\n";
							toolStripStatusLabel5.Text = ("Serial №2  не подключен |");
							lblResult.Text += ("Serial  №2  не подключен \r\n");
							toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
							toolStripStatusLabel4.BackColor = Color.Red;
						}
					}
				}
			catch(Exception e)
				{
				 // richTextBox2.Text += ("Error!");//show error message is something happend
				 // MessageBox.Show("Error!");//show error message is something happend
				}
			button_Find_Com2.Enabled = true;

			}
		private bool DetectArduino()
			{
			try
				{
				//The below setting are for the Hello handshake
				byte[] buffer = new byte[5];
				buffer[0] = Convert.ToByte(16);//start of message
				buffer[1] = Convert.ToByte(128);//command to tell arduino to say hi
				buffer[2] = Convert.ToByte(0);//empty command
				buffer[3] = Convert.ToByte(0);//empty command
				buffer[4] = Convert.ToByte(4);//empty command

				int intReturnASCII = 0;
				char charReturnValue = (Char)intReturnASCII;

				ComPort2.Open();//open com port connection
				ComPort2.Write(buffer, 0, 5);//send the message from the buffer array
				Thread.Sleep(1000);//wait, give time for replay
				int count =  ComPort2.BytesToRead;//get lenght of replay message
				string returnMessage = "";
				while(count > 0)
					{
						intReturnASCII = ComPort2.ReadByte();
						returnMessage = returnMessage + Convert.ToChar(intReturnASCII);//creat message letter by letter
						// richTextBox2.Text += (returnMessage);//debuging
						// MessageBox.Show(returnMessage);//debuging
						count--;
					}

				if(returnMessage.Contains("HELLO FROM SERIAL2"))//The Arduino said hello
					{
					ComPort2.Close();//close the serial com port connection
				   // richTextBox2.Text += ("  " + returnMessage + "  ");//debuging
					return true;
					}
				else//device was either not an arduino, or it didn't say hello
					{
					ComPort2.Close();//close the serial com port connection
					return false;
					}
				}
			catch(Exception e)
				{
					//richTextBox2.Text += ("Error, Device not connected \r\n");
					//MessageBox.Show("Error, Device not connected");
					return false;
				}
			}
		private bool DetectArduino1()
		{
			try
			{
				//The below setting are for the Hello handshake
				byte[] buffer = new byte[5];
				buffer[0] = Convert.ToByte(16);//start of message
				buffer[1] = Convert.ToByte(128);//command to tell arduino to say hi
				buffer[2] = Convert.ToByte(0);//empty command
				buffer[3] = Convert.ToByte(0);//empty command
				buffer[4] = Convert.ToByte(4);//empty command

				int intReturnASCII = 0;
				char charReturnValue = (Char)intReturnASCII;

			   // ComPort2.Open();//open com port connection
				ComPort2.Write(buffer, 0, 5);//send the message from the buffer array
				Thread.Sleep(1000);//wait, give time for replay
				int count = ComPort2.BytesToRead;//get lenght of replay message
				string returnMessage = "";
				while (count > 0)
				{
					intReturnASCII = ComPort2.ReadByte();
					returnMessage = returnMessage + Convert.ToChar(intReturnASCII);//creat message letter by letter
					// richTextBox2.Text += (returnMessage);//debuging
					// MessageBox.Show(returnMessage);//debuging
					count--;
				}

				if (returnMessage.Contains("HELLO FROM SERIAL2"))//The Arduino said hello
				{
				   // ComPort2.Close();//close the serial com port connection
					richTextBox2.Text += ("  " + returnMessage + "  ");//debuging
					return true;
				}
				else//device was either not an arduino, or it didn't say hello
				{
				   // ComPort2.Close();//close the serial com port connection
					return false;
				}
			}
			catch (Exception e)
			{
				//richTextBox2.Text += ("Error, Device not connected \r\n");
				//MessageBox.Show("Error, Device not connected");
				return false;
			}
		}
		
		private void TestComPort2()
		{
			startWrReg = 120;                                                                // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 28);                     // Очистить буфер serial2 
			Thread.Sleep(100);
			startWrReg = 120;                                                                // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 29);                     // Поиск КОМ 2 Автоматически  
			Thread.Sleep(200);
			try
			{
				//if (ComPort2.IsOpen) ComPort2.Close();
				//string[] ports = SerialPort.GetPortNames();//creat array with all port names on computer
				//foreach (string port in ports)//for every port (string) from the ports array.
				//{
				//    ComPort2 = new SerialPort(port, 115200, Parity.None, 8, StopBits.One);
				//    // richTextBox2.Text += (port);//debuging
				//    // MessageBox.Show(port);//debuging
					if (DetectArduino1())//check is the arduino is on this current port
					{
						// richTextBox2.Text += ("   Arduino Found!!! \r\n");//debuging
						lblResult.Text = ("Обнаружен Serial  " + ComPort2.PortName + "\r\n");
						// MessageBox.Show("Arduino Found!!!");//debuging
					  //  ComPort2.Open();
						//Open_ComPort2.Enabled = false;                          // Кнопка
						//Close_Serial2.Enabled = true;                           // Кнопка 
						lblResult.Text += ("Serial №2  USB 2 или RS 232 успешно открыт с параметрами:\r\n "
							 + (ComPort2.PortName + (", "
							 + (ComPort2.BaudRate + (" baud, "
							 + (ComPort2.DataBits + (" data bits, "
							 + (ComPort2.StopBits + (" stop bits, parity " + ComPort2.Parity + "\r\n")))))))));

						toolStripStatusLabel5.Text = (" Serial №2 = " + ComPort2.PortName + (", " + (ComPort2.BaudRate + (" baud |"))));
						label_COM2.Text = (ComPort2.PortName);
						toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5 УСТАНОВЛЕНА !");     // Обработка ошибки.
						toolStripStatusLabel4.BackColor = Color.Lime;

						File.WriteAllText("set_rs232.txt", cmbComPort2.SelectedItem.ToString(), Encoding.GetEncoding("UTF-8"));
						if (Serial_COM2)
						{
							label19.Text = ("RS-232");
						}
						else
						{
							label19.Text = ("USB 2");
						}
						serial2_connect_ok = true;
						Com2_SUCCESS = true;
	 
					}
					else
					{
						lblResult.Text = "Serial № 2 не найден \r\n";
						toolStripStatusLabel5.Text = ("Serial №2  не подключен xx|");
						lblResult.Text += ("Serial  №2  не подключен xx\r\n");
						toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
						toolStripStatusLabel4.BackColor = Color.Red;
					}
				//}
			}
			catch (Exception e)
			{
				// richTextBox2.Text += ("Error!");//show error message is something happend
				// MessageBox.Show("Error!");//show error message is something happend
			}
		}

		private delegate void SetTextDeleg(string text);                   //             
		//void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
		//{

		//    if (Oscilloscope_run == false)
		//    {
		//        Thread.Sleep(10);
		//        data = "";
		//        data = ComPort2.ReadLine();
		//        //  Привлечение делегата на потоке UI, и отправка данных, которые
		//        //  были приняты привлеченным методом.
		//        //  ---- Метод "si_DataReceived" будет выполнен в потоке UI,
		//        //  который позволит заполнить текстовое поле TextBox.
		//        this.BeginInvoke(new SetTextDeleg(si_DataReceived), new object[] { data });
		//    }

		//}
		private void si_DataReceived(string data)
		{
			progressBar3.Value = 100;

			if (read_file == true)
			{
				textBox48.Text += (data.Trim() + "\r\n");
				progressBar3.Increment(1);
			}
			else
			{
		
				textBox48.Text += (data.Trim() + " \r\n");
				if (list_files == true)
				{
					comboBox1.Items.Add(data.Trim());
					comboBox1.SelectedIndex = comboBox1.Items.Count - 1;
				}
			}
			progressBar3.Value = 0;
		}
		
		private void ComPort2_DataReceived(object sender, SerialDataReceivedEventArgs e)
				{
					try
					{
						if (Oscilloscope_run == false)
						{
							
							if (!ComPort2.IsOpen) ComPort2.Open();
						   // Thread.Sleep(150);
						   data = "";
						   // data = ComPort2.ReadExisting();//
							data = ComPort2.ReadLine();
							//  Привлечение делегата на потоке UI, и отправка данных, которые
							//  были приняты привлеченным методом.
							//  ---- Метод "si_DataReceived" будет выполнен в потоке UI,
							//  который позволит заполнить текстовое поле TextBox.
						   this.BeginInvoke(new SetTextDeleg(si_DataReceived), new object[] { data });
 
						}
						else
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
									oscilloscope1.AddScopeData(channels[0], channels[1], channels[2]);
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

					}


					catch { }
				}
				
		#endregion

		private void file_fakt_namber()
		{
			short[] readVals = new short[20];
			int slave;
			int startRdReg;
			int numRdRegs;
			slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			startRdReg = 112;                                                         // 40112 Адрес ячеек с именем текущего файла
			numRdRegs = 4;
			res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
			if ((res == BusProtocolErrors.FTALK_SUCCESS))
			{
				toolStripStatusLabel1.Text = "    MODBUS ON    ";
				toolStripStatusLabel1.BackColor = Color.Lime;
				textBox9.Text += "Текущий номер файла   -   ";
				textBox9.Text += (readVals[0]);
				if (readVals[1] < 10)
				{
					textBox9.Text += ("0" + readVals[1]);
				}
				else
				{
					textBox9.Text += (readVals[1]);
				}
				if (readVals[2] < 10)
				{
					textBox9.Text += ("0" + readVals[2]);
				}
				else
				{
					textBox9.Text += (readVals[2]);
				}
				if (readVals[3] < 10)
				{
					textBox9.Text += ("0" + readVals[3] + ".KAM" + "\r\n");
				}
				else
				{
					textBox9.Text += (readVals[3] + ".KAM" + "\r\n");
				}
			}

			else
			{
				toolStripStatusLabel1.Text = "    MODBUS ERROR (8) ";
				toolStripStatusLabel1.BackColor = Color.Red;
				toolStripStatusLabel4.Text = ("Ошибка получения имени файла!");  // Обработка ошибки.
				toolStripStatusLabel4.ForeColor = Color.Red;
				Thread.Sleep(100);
			}
			test_end1();
		}
 
		#region timer all
		private void timer_byte_set_Tick(object sender, EventArgs e)
		{

			int i;

			//*************************  Получить данные состояния модуля Камертон ************************************

			if ((myProtocol != null))
			{


				//  Напряжение питания на разъемах

				label45.Text  = string.Format("{0:0.00}", s12_Power, CultureInfo.CurrentCulture);
				label46.Text  = string.Format("{0:0.00}", s12_GGS, CultureInfo.CurrentCulture);
				label47.Text  = string.Format("{0:0.00}", s12_Radio1, CultureInfo.CurrentCulture);
				label48.Text  = string.Format("{0:0.00}", s12_Radio2, CultureInfo.CurrentCulture);
				label108.Text = string.Format("{0:0.00}", s12_Led_Mic, CultureInfo.CurrentCulture);

				//*************************** Вывод состояния битов Камертона *****************************************

				label30.Text = "";
				label31.Text = "";
				label32.Text = "";

				label33.Text = "";
				label34.Text = "";
				label35.Text = "";
				label36.Text = "";

                if (ggs_mute_on == false)                                                       // Признак выключения радиопередачи
                {
                    button83.Enabled = true;
                    groupBox28.Enabled = true;
                }
                if (ggs_mute_off == false)                                                       // Признак выключения радиопередачи
                {
                    button90.Enabled = true;
                    groupBox28.Enabled = true;
                }

               if(radio_on == false)                                                       // Признак выключения радиопередачи
               {
                   button92.Enabled   = true;
                   groupBox28.Enabled = true;
               }
               if (radio_off == false)                                                       // Признак выключения радиопередачи
               {
                   button102.Enabled  = true;
                   groupBox28.Enabled = true;
               }

               if (set_sound_level == false)
                {
                   button24.Enabled = true;
                }

               if (set_display == false)
               {

                   label43.Text = (s1_disp);
                   if (disp_mks)
                   {
                       label41.Text = (s2_disp);
                       label42.Text = "мкс";
                   }
                   else
                   {
                       label41.Text = "3,3";
                       label42.Text = "вольта";
                   }

                   button25.Enabled = true;
               }

				// ***** Обновить информацию из  модуля Аудио-1 *****************
				if (byte_set_ready)
				{
					for (int bite_x = 0; bite_x < 7; bite_x++)
					{

						decimalNum = readVals_byte[bite_x];
						while (decimalNum > 0)
						{
							binaryHolder = decimalNum % 2;
							binaryResult += binaryHolder;
							decimalNum = decimalNum / 2;
						}

						int len_str = binaryResult.Length;

						while (len_str < 8)
						{
							binaryResult += 0;
							len_str++;
						}

						//****************** Перемена битов ***************************
						//binaryArray = binaryResult.ToCharArray();
						//Array.Reverse(binaryArray);
						//binaryResult = new string(binaryArray);
						//*************************************************************
						for (i = 0; i < 8; i++)                         // 
						{
							if (binaryResult[i] == '1')
							{
								Dec_bin[i + (8 * bite_x)] = true;
							}
							else
							{
								Dec_bin[i + (8 * bite_x)] = false;
							}
						}
						binaryResult = "";
					}
				}
				//---------------------------------------------------------------------------
				//******************** Отобразить состояние регистров модуля Аудио-1 ****************
				for (i = 7; i >= 0; i--)
				{
					if (Dec_bin[i] == true)
					{
						label30.Text += ("1" + "  ");
					}
					else
					{
						label30.Text += ("0" + "  ");
					}
					if (Dec_bin[i + 8] == true)
					{
						label31.Text += ("1" + "  ");
					}
					else
					{
						label31.Text += ("0" + "  ");
					}


					if (Dec_bin[i + 16] == true)
					{
						label32.Text += ("1" + "  ");
					}
					else
					{
						label32.Text += ("0" + "  ");
					}

					if (Dec_bin[i + 24] == true)
					{
						label33.Text += ("1" + "  ");
					}
					else
					{
						label33.Text += ("0" + "  ");
					}

					if (Dec_bin[i + 32] == true)
					{
						label34.Text += ("1" + "  ");
					}
					else
					{
						label34.Text += ("0" + "  ");
					}

					if (Dec_bin[i + 40] == true)
					{
						label35.Text += ("1" + "  ");
					}
					else
					{
						label35.Text += ("0" + "  ");
					}
					if (Dec_bin[i + 48] == true)
					{
						label36.Text += ("1" + "  ");
					}
					else
					{
						label36.Text += ("0" + "  ");
					}
				}
				//-----------------------------------------------------------------------------------
				//***************** Отобразить состояние регистров Аудио-1 рядом с кнопками управления *****************

				if (Status_Rele_ready)
				{
					if (Dec_bin[24] == false) // 30024 флаг подключения ГГ Радио2
					{
						//label103.BackColor = Color.Red;
						//label103.Text = "0";
					}
					else
					{
						//label103.BackColor = Color.Lime;
						//label103.Text = "1";
					}
					if (Dec_bin[25] == false) // 30025 флаг подключения ГГ Радио1
					{
						//label104.BackColor = Color.Red;
						//label104.Text = "0";
					}
					else
					{
						//label104.BackColor = Color.Lime;
						//label104.Text = "1";
					}

					if (Dec_bin[26] == false) // 30026 флаг подключения трубки
					{
						label105.BackColor = Color.Red;
						label105.Text = "0";
					}
					else
					{
						label105.BackColor = Color.Lime;
						label105.Text = "1";
					}

					if (Dec_bin[27] == false)   // 30027 флаг подключения ручной тангенты
					{
						label106.BackColor = Color.Red;
						label106.Text = "0";
					}
					else
					{
						label106.BackColor = Color.Lime;
						label106.Text = "1";
					}

					if (Dec_bin[28] == false)  // 30028 флаг подключения педали
					{
						label107.BackColor = Color.Red;
						label107.Text = "0";
					}
					else
					{
						label107.BackColor = Color.Lime;
						label107.Text = "1";
					}

					if (Dec_bin[40] == false) // 30040  флаг подключения магнитофона
					{
						//label108.BackColor = Color.Red;
						//label108.Text = "0";
					}
					else
					{
						//label108.BackColor = Color.Lime;
						//label108.Text = "1";
					}

					if (Dec_bin[41] == false) // 30041  флаг подключения гарнитуры инструктора 2 наушниками
					{
						label109.BackColor = Color.Red;
						label109.Text = "0";
					}
					else
					{
						label109.BackColor = Color.Lime;
						label109.Text = "1";
					}

					if (Dec_bin[42] == false) // 30042  флаг подключения гарнитуры инструктора
					{
						label110.BackColor = Color.Red;
						label110.Text = "0";
					}
					else
					{
						label110.BackColor = Color.Lime;
						label110.Text = "1";
					}

					if (Dec_bin[43] == false) // 30043  флаг подключения гарнитуры диспетчера с 2 наушниками
					{
						label111.BackColor = Color.Red;
						label111.Text = "0";
					}
					else
					{
						label111.BackColor = Color.Lime;
						label111.Text = "1";
					}

					if (Dec_bin[44] == false) // 30044  флаг подключения гарнитуры диспетчера
					{
						label112.BackColor = Color.Red;
						label112.Text = "0";
					}
					else
					{
						label112.BackColor = Color.Lime;
						label112.Text = "1";
					}

					if (Dec_bin[45] == false) // 30045  флаг подключения микрофона XS1 - 6 Sence
					{
						label113.BackColor = Color.Red;
						label113.Text = "0";
					}
					else
					{
						label113.BackColor = Color.Lime;
						label113.Text = "1";
					}

					if (Dec_bin[46] == false) //  30046  флаг подключения ГГС
					{
						//label115.BackColor = Color.Red;
						//label115.Text = "0";
					}
					else
					{
						//label115.BackColor = Color.Lime;
						//label115.Text = "1";
					}


					if (Dec_bin[52] == false) // 30052   флаг выключения ГГС (Mute)
					{
						label144.BackColor = Color.Red;
						label144.Text = "0";
					}
					else
					{
						label144.BackColor = Color.Lime;
						label144.Text = "1";
					}

					if (Dec_bin[53] == false) // 30053   флаг радиопередачи
					{
						label143.BackColor = Color.Red;
						label143.Text = "0";
					}
					else
					{
						label143.BackColor = Color.Lime;
						label143.Text = "1";
					}

					if (Dec_bin[54] == false) // 30054   флаг управления микрофонами гарнитур
					{
						label142.BackColor = Color.Red;
						label142.Text = "0";
					}
					else
					{
						label142.BackColor = Color.Lime;
						label142.Text = "1";
					}


					if (Status_Rele_ready)
					{


						//----------------------------------------------------------------
						//******************** Отобразить состояние кнопок управления, полученное из Камертон 5.0 ********************


						if (coilArr_Status_Rele[0] == true)                              //   Реле RL0  Звук MIC1P
						{
							button37.BackColor = Color.Lime;
							button48.BackColor = Color.White;
						}
						else
						{
							button48.BackColor = Color.Red;
							button37.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[1] == true)                              //   Реле RL1  Звук MIC2P
						{
							button40.BackColor = Color.Lime;
							button53.BackColor = Color.White;
						}
						else
						{
							button53.BackColor = Color.Red;
							button40.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[2] == true)                              //   Реле RL2 Звук MIC3P
						{
							button44.BackColor = Color.Lime;
							button79.BackColor = Color.White;
						}
						else
						{
							button79.BackColor = Color.Red;
							button44.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[3] == true)                              //   Реле RL3 Звук LFE
						{
							button49.BackColor = Color.Lime;
							button66.BackColor = Color.White;
						}
						else
						{
							button66.BackColor = Color.Red;
							button49.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[4] == true)                              //   Реле RL4  Микрофон инструктора
						{
							button38.BackColor = Color.Lime;
							button52.BackColor = Color.White;
						}
						else
						{
							button52.BackColor = Color.Red;
							button38.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[5] == true)                              //   Реле RL5  Звук FR L, FR R
						{
							button71.BackColor = Color.Lime;
							button47.BackColor = Color.White;
						}
						else
						{
							button47.BackColor = Color.Red;
							button71.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[6] == true)                              //   Реле RL6  Звук CENTER
						{
							button69.BackColor = Color.Lime;
							button42.BackColor = Color.White;
						}
						else
						{
							button42.BackColor = Color.Red;
							button69.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[7] == true)                              //   Реле RL7  Питание 12 вольт
						{
							button51.BackColor = Color.Lime;
							button45.BackColor = Color.White;
						}
						else
						{
							button45.BackColor = Color.Red;
							button51.BackColor = Color.White;
						}


						//   startCoil = 9;  //  regBank.add(00009-16);   Отображение соостояния реле 9-16
						//   numCoils = 8;

						if (coilArr_Status_Rele[8] == true)                              //   Реле RL8 Звук на микрофон regBank.add(9)
						{
							button46.BackColor = Color.Lime;
							button50.BackColor = Color.White;
						}
						else
						{
							button50.BackColor = Color.Red;
							button46.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[9] == true)                              //  Реле RL9  XP1 10 regBank.add(10) Микрофон диспетчера
						{
							button27.BackColor = Color.Lime;
							button30.BackColor = Color.White;
						}
						else
						{
							button30.BackColor = Color.Red;
							button27.BackColor = Color.White;
						}

						if (coilArr_Status_Rele[10] == true)                              //    Реле RL10 regBank.add(11) Импульс (высоковольтный источник питания)
						{
							button2.BackColor = Color.Lime;
							button81.BackColor = Color.White;
						}
						else
						{
							button81.BackColor = Color.Red;
							button2.BackColor = Color.White;
						}

						if (coilArr_Status_Rele[11] == true)                                //   Свободен regBank.add(12)
						{
							//button27.BackColor = Color.Lime;
							//button30.BackColor = Color.White;
						}
						else
						{
							//button30.BackColor = Color.Red;
							//button27.BackColor = Color.White;
						}


						if (coilArr_Status_Rele[12] == true)                             // XP8 - 2  Sence Танг н. regBank.add(13)
						{
							button59.BackColor = Color.Lime;
							button74.BackColor = Color.White;
						}
						else
						{
							button74.BackColor = Color.Red;
							button59.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[13] == true)                             //XP8 - 1  PTT Танг н. regBank.add(14)
						{
							button39.BackColor = Color.Lime;
							button41.BackColor = Color.White;
						}
						else
						{
							button41.BackColor = Color.Red;
							button39.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[14] == true)                             // XS1 - 5   PTT Мик  regBank.add(15)
						{
							button7.BackColor = Color.Lime;
							button18.BackColor = Color.White;
						}
						else
						{
							button18.BackColor = Color.Red;
							button7.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[15] == true)                             // XS1 - 6 Sence Мик. regBank.add(16)
						{
							button33.BackColor = Color.Lime;
							button34.BackColor = Color.White;
						}
						else
						{
							button34.BackColor = Color.Red;
							button33.BackColor = Color.White;
						}

						//   startCoil = 17;  //  regBank.add(00017-24);   Отображение соостояния реле 17-24
						//   numCoils = 8;

						if (coilArr_Status_Rele[16] == true)                             // XP7 4 PTT2 Танг. р.  regBank.add(17)
						{
							button14.BackColor = Color.Lime;
							button29.BackColor = Color.White;
						}
						else
						{
							button29.BackColor = Color.Red;
							button14.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[17] == true)                             // XP1 - 20  HangUp  DCD regBank.add(18)
						{
							button19.BackColor = Color.Lime;
							button26.BackColor = Color.White;
						}
						else
						{
							button26.BackColor = Color.Red;
							button19.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[18] == true)                             // J8-11     XP7 2 Sence  Танг. р. regBank.add(19)
						{
							button58.BackColor = Color.Lime;
							button72.BackColor = Color.White;
						}
						else
						{
							button72.BackColor = Color.Red;
							button58.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[19] == true)                             //  XP7 1 PTT1 Танг. р.  regBank.add(20)
						{
							button10.BackColor = Color.Lime;
							button23.BackColor = Color.White;
						}
						else
						{
							button23.BackColor = Color.Red;
							button10.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[20] == true)                             // XP2-2    Sence "Маг." 
						{
							//button60.BackColor = Color.Lime;
							//button76.BackColor = Color.White;
						}
						else
						{
							//button76.BackColor = Color.Red;
							//button60.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[21] == true)                             // XP5-3    Sence "ГГC."
						{
							//button35.BackColor = Color.Lime;
							//button36.BackColor = Color.White;
						}
						else
						{
							//button36.BackColor = Color.Red;
							//button35.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[22] == true)                             // XP3-3    Sence "ГГ-Радио1."
						{
							//button56.BackColor = Color.Lime;
							//button68.BackColor = Color.White;
						}
						else
						{
							//button68.BackColor = Color.Red;
							//button56.BackColor = Color.White;
						}
						if (coilArr_Status_Rele[23] == true)                             // XP4-3    Sence "ГГ-Радио2."
						{
							//button55.BackColor = Color.Lime;
							//button67.BackColor = Color.White;
						}
						else
						{
							//button67.BackColor = Color.Red;
							//button55.BackColor = Color.White;
						}


						//  startCoil = 25;  //  regBank.add(00001-12);   Отображение соостояния реле 25-32
						//  numCoils = 8;


						if (coilArr_Status_Rele[24] == false)                            // XP1- 19 HaSs      Сенсор  подключения трубки       
						{
							button57.BackColor = Color.Lime;
							button70.BackColor = Color.White;
						}
						else
						{
							button70.BackColor = Color.Red;
							button57.BackColor = Color.White;
						}

						if (coilArr_Status_Rele[25] == true)                             // XP1- 17 HaSPTT    CTS DSR вкл. 
						{
							button16.BackColor = Color.Lime;
							button20.BackColor = Color.White;
						}
						else
						{
							button20.BackColor = Color.Red;
							button16.BackColor = Color.White;
						}



						if (coilArr_Status_Rele[26] == true)                             // XP1- 16 HeS2Rs    флаг подключения гарнитуры инструктора с 2 наушниками
						{
							button61.BackColor = Color.Lime;
							button78.BackColor = Color.White;
						}
						else
						{
							button78.BackColor = Color.Red;
							button61.BackColor = Color.White;
						}


						if (coilArr_Status_Rele[27] == true)                             // XP1- 15 HeS2PTT   CTS вкл
						{
							button28.BackColor = Color.Lime;
							button17.BackColor = Color.White;
						}
						else
						{
							button17.BackColor = Color.Red;
							button28.BackColor = Color.White;
						}

						if (coilArr_Status_Rele[28] == true)                             //    XP1- 13 HeS2Ls    флаг подключения гарнитуры инструктора 
						{
							button62.BackColor = Color.Lime;
							button77.BackColor = Color.White;
						}
						else
						{
							button77.BackColor = Color.Red;
							button62.BackColor = Color.White;
						}

						if (coilArr_Status_Rele[29] == true)                             //    XP1- 6  HeS1PTT   CTS вкл
						{
							button8.BackColor = Color.Lime;
							button22.BackColor = Color.White;
						}
						else
						{
							button22.BackColor = Color.Red;
							button8.BackColor = Color.White;
						}

						if (coilArr_Status_Rele[30] == true)                             //   XP1- 5  HeS1Rs    Флаг подкючения гарнитуры диспетчера с 2 наушниками
						{
							button63.BackColor = Color.Lime;
							button75.BackColor = Color.White;
						}
						else
						{
							button75.BackColor = Color.Red;
							button63.BackColor = Color.White;
						}

						if (coilArr_Status_Rele[31] == true)                             //    XP1- 1  HeS1Ls    Флаг подкючения гарнитуры диспетчера
						{
							button64.BackColor = Color.Lime;
							button73.BackColor = Color.White;
						}
						else
						{
							button73.BackColor = Color.Red;
							button64.BackColor = Color.White;
						}

					}


					//---------------------------------------------------------

					//*******************************************************
					if (coilArrFlag[0] == false) // бит CTS - 1x81 
					{
						label156.BackColor = Color.Red;
						label156.Text = "1";
					}
					else
					{
						label156.BackColor = Color.Lime;
						label156.Text = "0";
					}
					if (coilArrFlag[1] == false) // бит DSR - 1x82  
					{
						label155.BackColor = Color.Red;
						label155.Text = "1";
					}
					else
					{
						label155.BackColor = Color.Lime;
						label155.Text = "0";
					}
					if (coilArrFlag[2] == false) // // бит DCD -  1x83
					{
						label152.BackColor = Color.Red;
						label152.Text = "1";
					}
					else
					{
						label152.BackColor = Color.Lime;
						label152.Text = "0";
					}
				}

				if (MODBUS_SUCCESS)
				{
					toolStripStatusLabel1.BackColor = Color.Lime;
					if(serial2_connect_ok == true)
						{
							Com2_SUCCESS = true;
						}
				}
				else
				{
					toolStripStatusLabel1.BackColor = Color.Red;
				}
				if(Com2_SUCCESS)
				{
				   toolStripStatusLabel4.BackColor = Color.Lime;
				   toolStripStatusLabel4.ForeColor = Color.Black;
				}
				else
				{
					toolStripStatusLabel4.ForeColor = Color.Red;
					toolStripStatusLabel4.BackColor = Color.White;
				}

			}

			else
			{
				Com2_SUCCESS                    = false;
				toolStripStatusLabel4.Text      = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
				toolStripStatusLabel4.ForeColor = Color.Red;
				//  Thread.Sleep(100);
			}
		}
		private void timer1_Tick(object sender, EventArgs e)
			{
                //textBox7.Text += s_txt7;
               // textBox8.Text += s_txt8;
                //textBox9.Text += s_txt9;
                //textBox48.Text += s_txt48;
                //s_txt7 = "";
                //s_txt8 = "";
                //s_txt9 = "";
                //s_txt48 = "";

			    if(All_Test_run)
				    {
				    progressBar4.Visible = true;               //  Отобразить прогрессбар (верхний) выполнения теста
                    start_DoWorkAll_Test1();                   //  Запустить  прогрессбар (верхний)выполнения теста
				    }
			    else
				    {
				    progressBar4.Visible = false;                 //  Запретить отображение прогрессбар выполнения теста
				    stop_DoWorkAll_Test1();                       //  Остановить прогрессбар выполнения теста
				    }
			}
/*
        private void timer_s_txt7_Tick(object sender, EventArgs e)
        {
            //textBox7.Text += s_txt7;
            //s_txt7 = "";
            //textBox7.Refresh();
            timer_s_txt7.Stop();
        }
        private void timer_s_txt8_Tick(object sender, EventArgs e)
        {
            //textBox8.Text += s_txt8;
            //s_txt8 = "";
            //textBox8.Refresh();
            timer_s_txt8.Stop();
        }
        private void timer_s_txt9_Tick(object sender, EventArgs e)
        {
            //textBox9.Text += s_txt9;
            //s_txt9 = "";
            //textBox9.Refresh();
            timer_s_txt9.Stop();
        }
        private void timer_s_txt48_Tick(object sender, EventArgs e)
        {
            //textBox48.Text += s_txt48;
            //s_txt48 = "";
            //textBox48.Refresh();
            timer_s_txt48.Stop();
        }
*/
        private void timer_param_set_Tick(object sender, EventArgs e)
			{
			timer1.Start();
			start_test_modbus1();                                                        // Включить сканирование MODBUS если оно не включено
			Thread.Sleep(400);
			timer_param_set.Stop();
			}
		private void timerVisualAll_Test_Tick(object sender, EventArgs e)
			{

                if (s_txt7.Length != 0)
                {
                    textBox7.Text += s_txt7;
                    textBox7.SelectionStart = textBox7.Text.Length;
                    textBox7.ScrollToCaret();
                    textBox7.Refresh();
                    s_txt7 = "";
                }
                if (s_txt8.Length != 0)
                {
                    textBox8.Text += s_txt8;
                    textBox8.SelectionStart = textBox8.Text.Length;
                    textBox8.ScrollToCaret();
                    textBox8.Refresh();
                    s_txt8 = "";
                }

                if (s_txt9.Length != 0)
                {
                    textBox9.Text += s_txt9;
                    textBox9.SelectionStart = textBox9.Text.Length;
                    textBox9.ScrollToCaret();
                    textBox9.Refresh();
                    s_txt9 = "";
                }
                if (s_txt48.Length != 0)
                {
                    textBox48.Text += s_txt48;
                    textBox48.SelectionStart = textBox48.Text.Length;
                    textBox48.ScrollToCaret();
                    textBox48.Refresh();
                    s_txt48 = "";
                }


              //  if (s_txt7 != "") textBox7.Text += s_txt7;
              //  if (s_txt8 != "") textBox8.Text += s_txt8;
              //  if (s_txt9 != "") textBox9.Text += s_txt9;
              //  if (s_txt48 != "") textBox48.Text += s_txt48;

              //  textBox7.SelectionStart = textBox7.Text.Length;
              //  textBox7.ScrollToCaret();
              //  textBox8.SelectionStart = textBox8.Text.Length;
              //  textBox8.ScrollToCaret();
              //  textBox9.SelectionStart = textBox9.Text.Length;
              //  textBox9.ScrollToCaret();
              //  textBox48.SelectionStart = textBox48.Text.Length;
              //  textBox48.ScrollToCaret();
              //  s_txt7 = "";
              //  s_txt8 = "";
              //  s_txt9 = "";
              //  s_txt48 = "";

			// Запуск индикации выполнения теста
			if(All_Test_run)
				{
				progressBar4.Visible = true;               //  Отобразить прогрессбар выполнения теста
				start_DoWorkAll_Test1();                   //  Запустить  прогрессбар выполнения теста
				}
			else
				{
				progressBar4.Visible = false;                 //  Запретить отображение прогрессбар выполнения теста
				stop_DoWorkAll_Test1();                       //  Остановить прогрессбар выполнения теста
				}
			}
		private void timer_param_set2_Tick(object sender, EventArgs e)
			{
			  start_bw_set_byte();
			  Thread.Sleep(300);
			  timer_param_set2.Stop();
			}
		private void timerPowerOff_Tick(object sender, EventArgs e)
			{
 			    startCoil = 8;                                                     // Управление питанием платы "Камертон"
			    res = myProtocol.writeCoil(slave, startCoil, false);               // Выключить питание платы "Камертон"
			    Thread.Sleep(200);
			    timerPowerOff.Stop();
			}
		private void timerCloseForm_Tick(object sender, EventArgs e)
		{
		  
		}

		#endregion
	  
		private void button3_Click(object sender, EventArgs e)                         // Записать системное время
		{
			//   time_system_save();
			ushort[] writeVals = new ushort[20];
			bool[] coilVals = new bool[2];
			int startWrReg;
			int numWrRegs;                                                                         //
			int numVal = -1;

			if ((myProtocol != null))
			{
				stop_test_modbus1();                                                               // Отключить сканирование MODBUS 
				Thread.Sleep(400);
				string command = label80.Text;
				numVal = Convert.ToInt32(command.Substring(0, 2), CultureInfo.CurrentCulture);
				writeVals[0] = (ushort)numVal;                                                     // 
				numVal = Convert.ToInt32(command.Substring(3, 2), CultureInfo.CurrentCulture);
				writeVals[1] = (ushort)numVal;                                                     // 
				numVal = Convert.ToInt32(command.Substring(6, 4), CultureInfo.CurrentCulture);
				writeVals[2] = (ushort)numVal;                                                     // 
				numVal = Convert.ToInt32(command.Substring(11, 2), CultureInfo.CurrentCulture);
				writeVals[3] = (ushort)numVal;                                                     // 
				numVal = Convert.ToInt32(command.Substring(14, 2), CultureInfo.CurrentCulture);
				writeVals[4] = (ushort)numVal;                                                     // 
				startWrReg = 52;                                                                   // 40052 Установка времени в контроллере
				numWrRegs = 6;                                                                     //
				res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				startWrReg = 120;
				res = myProtocol.writeSingleRegister(slave, startWrReg, 14);                       // Записать системное время
				Thread.Sleep(400);
				timer_param_set.Start();

			}
			else
			{
				Com2_SUCCESS                    = false;
				toolStripStatusLabel4.Text      = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
				toolStripStatusLabel4.ForeColor = Color.Red;
				Thread.Sleep(100);
			}
		}
		private void button4_Click(object sender, EventArgs e)                         // Записать пользовательское время
		{

			ushort[] writeVals = new ushort[20];
			bool[] coilVals = new bool[2];
			int slave;                           //
			int res;
			int startWrReg;
			int numWrRegs;   //

			slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			int numVal = -1;
			if ((myProtocol != null))
			{
				stop_test_modbus1();                                                             // Отключить сканирование MODBUS 
				Thread.Sleep(400);
				string command = dateTimePicker1.Value.ToString("ddMMyyyyHHmmss", CultureInfo.CurrentCulture);
				numVal = Convert.ToInt32(command.Substring(0, 2), CultureInfo.CurrentCulture);
				writeVals[0] = (ushort)numVal;   // 
				numVal = Convert.ToInt32(command.Substring(2, 2), CultureInfo.CurrentCulture);
				writeVals[1] = (ushort)numVal;   // 
				numVal = Convert.ToInt32(command.Substring(4, 4), CultureInfo.CurrentCulture);
				writeVals[2] = (ushort)numVal;   // 
				numVal = Convert.ToInt32(command.Substring(8, 2), CultureInfo.CurrentCulture);
				writeVals[3] = (ushort)numVal;   // 
				numVal = Convert.ToInt32(command.Substring(10, 2), CultureInfo.CurrentCulture);
				writeVals[4] = (ushort)numVal;   // 
				startWrReg = 52;
				numWrRegs = 6;   //
				res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				startWrReg = 120;
				res = myProtocol.writeSingleRegister(slave, startWrReg, 14);                       // Записать новое время пользователя
				Thread.Sleep(400);
				timer_param_set.Start();
			}
			else
			{
				Com2_SUCCESS                    = false;
				toolStripStatusLabel4.Text      = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");    // Обработка ошибки.
				toolStripStatusLabel4.ForeColor = Color.Red;
				Thread.Sleep(100);
			}
		}

		#region Button test
   
		//****************  Включение реле ************************************************
		private void button37_Click(object sender, EventArgs e)                      // ВКЛ Реле RL0
		{
			coil_Button[1] = true;
		}
		private void button48_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL0
		{
			coil_Button[1] = false;
		}

		private void button40_Click(object sender, EventArgs e)                      // ВКЛ Реле RL1
		{
			coil_Button[2] = true;
		}
		private void button53_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL1
		{
			coil_Button[2] = false;
		}

		private void button44_Click(object sender, EventArgs e)                      // ВКЛ Реле RL2
		{
			coil_Button[3] = true;
		}
		private void button79_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL2
		{
			coil_Button[3] = false;
		}

		private void button49_Click(object sender, EventArgs e)                      // ВКЛ Реле RL3
		{
			coil_Button[4] = true;
		}
		private void button66_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL3
		{
			coil_Button[4] = false;
		}

		private void button38_Click(object sender, EventArgs e)                      // ВКЛ Реле RL4
		{
			coil_Button[5] = true;
		}
		private void button52_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL4
		{
			coil_Button[5] = false;
		}

		private void button71_Click(object sender, EventArgs e)                      // ВКЛ Реле RL5
		{
			coil_Button[6] = true;
		}
		private void button47_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL5
		{
			coil_Button[6] = false;
		}

		private void button69_Click(object sender, EventArgs e)                      // ВКЛ Реле RL6
		{
			coil_Button[7] = true;
		}
		private void button42_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL6
		{
			coil_Button[7] = false;
		}

		private void button51_Click(object sender, EventArgs e)                      // ВКЛ Реле RL7
		{
			coil_Button[8] = true;
		}
		private void button45_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL7
		{
			coil_Button[8] = false;
		}

		private void button46_Click(object sender, EventArgs e)                      // ВКЛ Реле RL8
		{
			coil_Button[9] = true;
		}
		private void button50_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL8
		{
			coil_Button[9] = false;
		}

		private void button27_Click(object sender, EventArgs e)                      // ВКЛ Реле RL9
		{
			coil_Button[10] = true;
		}
		private void button30_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL9
		{
			coil_Button[10] = false; // Микрофон диспетчера отключить
		}

		private void button2_Click(object sender, EventArgs e)                       // ВКЛ Реле RL10
		{
			coil_Button[11] = true;
		}
		private void button81_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL10
		{
			coil_Button[11] = false; // Питание высоковольтное отключить
		}

		//**********************************************************************************************


		/*

				private void button55_Click(object sender, EventArgs e)                      // Отключено, Применяется для измерения напряжения
				{
					coil_Button[24] = true;                                                        // Управление сенсорами
				}
				private void button67_Click(object sender, EventArgs e)                      // Отключено, Применяется для измерения напряжения
				{
					coil_Button[24] = false;                                                        // Управление сенсорами
				 
				}

				private void button56_Click(object sender, EventArgs e)                      // Отключено, Применяется для измерения напряжения
				{
					coil_Button[23] = true;                                                  // Управление сенсорами
				  
				}
				private void button68_Click(object sender, EventArgs e)                      // Отключено, Применяется для измерения напряжения
				{
				   coil_Button[23] = false;  
				}
		*/

		private void button57_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ подключения трубки    XP1- 19 HaSs  
		{
			coil_Button[25] = false;
		}
		private void button70_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ подключения трубки    XP1- 19 HaSs  
		{
			coil_Button[25] = true;
		}

		private void button58_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор  Танг.р 
		{
			coil_Button[19] = true; // Управление сенсорами
		}
		private void button72_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор Танг.р 
		{
			coil_Button[19] = false;
		}

		private void button59_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор Танг.н
		{
			coil_Button[13] = true; // Управление сенсорами
		}
		private void button74_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор Танг.н
		{
			coil_Button[13] = false;
		}

		private void button60_Click(object sender, EventArgs e)              // Отключено, Применяется для измерения напряжения 
		{
			// coil_Button[21] = true; // Управление сенсорами
		}

		private void button76_Click(object sender, EventArgs e)              // Отключено, Применяется для измерения напряжения 
		{
			// coil_Button[21] = false;
		}

		private void button61_Click(object sender, EventArgs e)                      // 7 Кнопка  ВКЛ   XP1- 16 HeS2Rs Сенсор подключения гарнитуры инструктора с 2 наушниками
		{
			coil_Button[27] = true;
		}
		private void button78_Click(object sender, EventArgs e)                      // 7 Кнопка  ОТКЛ  XP1- 16 HeS2Rs  Сенсор подключения гарнитуры инструктора с 2 наушниками
		{
			coil_Button[27] = false;
		}

		private void button62_Click(object sender, EventArgs e)                      // ВКЛ XP1- 13 HeS2Ls Кнопка  ВКЛ флаг подключения гарнитуры инструктора 
		{
			coil_Button[29] = true;

		}
		private void button77_Click(object sender, EventArgs e)                      // ОТКЛ XP1- 13 HeS2Ls Кнопка  ОТКЛ флаг подключения гарнитуры инструктора  
		{
			coil_Button[29] = false;
		}

		private void button63_Click(object sender, EventArgs e)                      // XP1- 5  HeS1Rs Кнопка  ВКЛ Флаг подключения гарнитуры диспетчера с 2 наушниками
		{
			coil_Button[31] = true;
		}
		private void button75_Click(object sender, EventArgs e)                      // XP1- 5  HeS1Rs Кнопка  ОТКЛ Флаг подключения гарнитуры диспетчера с 2 наушниками
		{
			coil_Button[31] = false;
		}

		private void button64_Click(object sender, EventArgs e)                      // XP1- 1  HeS1Ls  Кнопка  ВКЛ  Флаг подключения гарнитуры диспетчера
		{
			coil_Button[32] = true;
		}
		private void button73_Click(object sender, EventArgs e)                      // XP1- 1  HeS1Ls  Кнопка  ОТКЛ Флаг подключения гарнитуры диспетчера
		{
			coil_Button[32] = false;
		}

		private void button33_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор Мик.
		{
			coil_Button[16] = true;
		}
		private void button34_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор Мик.
		{
			coil_Button[16] = false;
		}

		//private void button35_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ питание +12 ГГС
		//{
		//    coil_Button[22] = true;
		//}
		//private void button36_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ питание +12 ГГС
		//{
		//    coil_Button[22] = false;
		//}
 
		//********************** Вторая колонка кнопок*****************************


		private void button7_Click(object sender, EventArgs e)                       // ВКл . XP8 - 1  PTT Мик
		{
			coil_Button[15] = true;
		}
		private void button18_Click(object sender, EventArgs e)                      // ОТКЛ XP8 - 1  PTT Мик
		{
			coil_Button[15] = false;
		}

		private void button10_Click(object sender, EventArgs e)                      // ВКЛ   XP7 1 PTT1 Танг. р.
		{
			coil_Button[20] = true;
		}
		private void button23_Click(object sender, EventArgs e)                      // ОТКЛ   XP7 1 PTT1 Танг. р.
		{
			coil_Button[20] = false;
		}

		private void button14_Click(object sender, EventArgs e)                      // ВКЛ  XP7 4 PTT2 Танг. р.
		{
			coil_Button[17] = true;
		}
		private void button29_Click(object sender, EventArgs e)                      // ОТКЛ  XP7 4 PTT2 Танг. р.
		{
			coil_Button[17] = false;
		}

		private void button19_Click(object sender, EventArgs e)                      // ВКЛ XP1 - 20  HangUp  DCD
		{
			coil_Button[18] = true;
		}
		private void button26_Click(object sender, EventArgs e)                      // ОТКЛ XP1 - 20  HangUp  DCD
		{
			coil_Button[18] = false;
		}

		private void button8_Click(object sender, EventArgs e)                       // ВКЛ XP1- 6  HeS1PTT   CTS вкл
		{
			coil_Button[30] = true;
		}
		private void button22_Click(object sender, EventArgs e)                      // ОТКЛ  XP1- 6  HeS1PTT   CTS вкл
		{
			coil_Button[30] = false;
		}

		private void button28_Click(object sender, EventArgs e)                      // ВКЛ  XP1- 15 HeS2PTT   CTS вкл
		{
			coil_Button[28] = true;
		}
		private void button17_Click(object sender, EventArgs e)                      // ОТКЛ  XP1- 15 HeS2PTT   CTS вкл
		{
			coil_Button[28] = false;
		}

		private void button16_Click(object sender, EventArgs e)                      // ВКЛ  XP1- 17 HaSPTT    CTS DSR вкл.
		{
			coil_Button[26] = true;
		}
		private void button20_Click(object sender, EventArgs e)                      // ОТКЛ  XP1- 17 HaSPTT    CTS DSR вкл.
		{
			coil_Button[26] = false;
		}

		private void button39_Click(object sender, EventArgs e)                      // ВКЛ XP8-1 РТТ Танг. н.
		{
			coil_Button[14] = true;
		}
		private void button41_Click(object sender, EventArgs e)                      // ОТКЛ XP8-1  РТТ Танг. н.
		{
			coil_Button[14] = false;
		}

        private void button92_Click(object sender, EventArgs e)                      // Кнопка Радиопередача вкл.
        {
            button92.Enabled   = false;
            groupBox28.Enabled = false;
            radio_on           = true;                                               // Признак включения радиопередачи
        }
        private void button102_Click(object sender, EventArgs e)                     // Кнопка Радиопередача откл.
        {
            button102.Enabled  = false;
            groupBox28.Enabled = false;
            radio_off          = true; 
        }
        private void button83_Click(object sender, EventArgs e)                      // Кнопка    ГГС (mute) вкл.
        {
            button83.Enabled   = false;
            groupBox28.Enabled = false;
            ggs_mute_on        = true;
/*
            stop_bw_set_byte();
            while (byte_set_run) { };
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            if ((myProtocol != null))
            {
                startCoil = 42;                                                     // Регистр 41 управления кнопкой 
                res = myProtocol.writeCoil(slave, startCoil, true);                 // Включить кнопку ГГС (mute) вкл.
                startWrReg = 120;                                                   // 
                res = myProtocol.writeSingleRegister(slave, startWrReg, 53);        // 
                test_end1();
            }
            else
            {
                Com2_SUCCESS = false;
                toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                toolStripStatusLabel4.ForeColor = Color.Red;
                Thread.Sleep(100);
            }
            button83.Enabled = true;
            groupBox28.Enabled = true;
            timer_param_set2.Start();
            */
        }
        private void button90_Click(object sender, EventArgs e)                      // Кнопка    ГГС (mute) откл.
        {
            button90.Enabled   = false;
            groupBox28.Enabled = false;
            ggs_mute_off       = true;
            /*
            stop_bw_set_byte();
            while (byte_set_run) { };
            // Thread.Sleep(1000);
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            if ((myProtocol != null))
            {
                startCoil = 42;                                                     // Регистр 41 управления кнопкой 
                res = myProtocol.writeCoil(slave, startCoil, false);                 // Включить кнопку ГГС (mute) вкл.
                startWrReg = 120;                                                   // 
                res = myProtocol.writeSingleRegister(slave, startWrReg, 53);        // 
                test_end1();
            }
            else
            {
                Com2_SUCCESS = false;
                toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                toolStripStatusLabel4.ForeColor = Color.Red;
                Thread.Sleep(100);
            }
            button90.Enabled = true;
            groupBox28.Enabled = true;
            timer_param_set2.Start();
            */
        }


		//**********************************************************************************************
		#endregion

		#region Test all
		// для вызова тестов необходимо отправить по адресу 120  в контроллер номер теста  (1-23)
   
		private void sensor_off()// 
		{
			ushort[] writeVals = new ushort[2];
			short[] readVals = new short[124];
			bool[] coilArr = new bool[64];
			slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 1); // Отключить все сенсоры
			s_txt7 += ("Команда на отключение сенсоров отправлена" + "\r\n");
 			Thread.Sleep(300);

			// Новый фрагмент чтения регистров 40001-40007

			int startRdReg;
			int numRdRegs;
			int i;
			bool[] coilVals = new bool[64];
			bool[] coilSensor = new bool[64];

			//*************************  Получить данные состояния модуля Камертон ************************************

			Int64 binaryHolder;
			string binaryResult = "";
			int decimalNum;
			bool[] Dec_bin = new bool[64];
			startRdReg = 1;
			numRdRegs = 7;
			res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);    // 03  Считать число из регистров по адресу  40000 -49999
			if ((res == BusProtocolErrors.FTALK_SUCCESS))
			{
				toolStripStatusLabel1.Text = "    MODBUS ON    ";
				MODBUS_SUCCESS = true;

				for (int bite_x = 0; bite_x < 7; bite_x++)
				{
					decimalNum = readVals[bite_x];
					while (decimalNum > 0)
					{
						binaryHolder = decimalNum % 2;
						binaryResult += binaryHolder;
						decimalNum = decimalNum / 2;
					}

					int len_str = binaryResult.Length;

					while (len_str < 8)
					{
						binaryResult += 0;
						len_str++;
					}

					//****************** Перемена битов ***************************
					//binaryArray = binaryResult.ToCharArray();
					//Array.Reverse(binaryArray);
					//binaryResult = new string(binaryArray);
					//*************************************************************

					for (i = 0; i < 8; i++)                         // 
					{
						if (binaryResult[i] == '1')
						{
							Dec_bin[i + (8 * bite_x)] = true;
						}
						else
						{
							Dec_bin[i + (8 * bite_x)] = false;
						}
					}
					binaryResult = "";
				}

			}
		}
		private void sensor_on()
		{
 			ushort[] writeVals = new ushort[2];
			short[] readVals = new short[124];
			bool[] coilArr = new bool[64];
			slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 2); // Включить все сенсоры
			s_txt7 += ("Команда на включение сенсоров отправлена" + "\r\n");
 			Thread.Sleep(300);

			//  фрагмент чтения регистров 40001-40007 состояния "Камертон"

			int startRdReg;
			int numRdRegs;
			int i;
			bool[] coilVals = new bool[64];
			bool[] coilSensor = new bool[64];

			//*************************  Получить данные состояния модуля Камертон ************************************

			Int64 binaryHolder;
			string binaryResult = "";
			int decimalNum;
			bool[] Dec_bin = new bool[64];
			startRdReg = 1;
			numRdRegs = 7;
			res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);    // 03  Считать число из регистров по адресу  40000 -49999
			if ((res == BusProtocolErrors.FTALK_SUCCESS))
			{
				toolStripStatusLabel1.Text = "    MODBUS ON    ";
				MODBUS_SUCCESS = true;

				for (int bite_x = 0; bite_x < 7; bite_x++)
				{
					decimalNum = readVals[bite_x];
					while (decimalNum > 0)
					{
						binaryHolder = decimalNum % 2;
						binaryResult += binaryHolder;
						decimalNum = decimalNum / 2;
					}

					int len_str = binaryResult.Length;

					while (len_str < 8)
					{
						binaryResult += 0;
						len_str++;
					}

					//****************** Перемена битов ***************************
					//binaryArray = binaryResult.ToCharArray();
					//Array.Reverse(binaryArray);
					//binaryResult = new string(binaryArray);
					//*************************************************************

					for (i = 0; i < 8; i++)                         // 
					{
						if (binaryResult[i] == '1')
						{
							Dec_bin[i + (8 * bite_x)] = true;
						}
						else
						{
							Dec_bin[i + (8 * bite_x)] = false;
						}
					}
					binaryResult = "";
				}

			}
		}
		private void test_instruktora()
		{
			startWrReg = 120;                                           // В 40120 ячейке хранится номер теста. Эту ячейку применяет test_switch() Arduino
			res = myProtocol.writeSingleRegister(slave, startWrReg, 3); // Команда на проверку 'Гарнитура Инструктора'
			s_txt7 += ("Команда на проверку 'Гарнитура Инструктора' отправлена" + "\r\n");
			Thread.Sleep(4050);
		}
		private void test_dispetchera()
		{
			startWrReg = 120;                                           // В 40120 ячейке хранится номер теста. Эту ячейку применяет test_switch() Arduino
			res = myProtocol.writeSingleRegister(slave, startWrReg, 4); // Команда на проверку 'Гарнитура Диспетчера'
			s_txt7 += ("Команда на проверку 'Гарнитура Диспетчера' отправлена" + "\r\n");
			Thread.Sleep(450);
		}
		private void test_MTT()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 5);   // Команда на проверку 'МТТ'
			s_txt7 += ("Команда на проверку 'МТТ' отправлена" + "\r\n");
 			Thread.Sleep(450);
		}
		private void test_tangR()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 6); // Команда на проверку 'Тангента ручная'
			s_txt7 += ("Команда на проверку 'Тангента ручная' отправлена" + "\r\n");
 			Thread.Sleep(450);
		}
		private void test_tangN()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 7); // Команда на проверку 'Тангента ножная'
			s_txt7 += ("Команда на проверку 'Тангента ножная' отправлена" + "\r\n");
			Thread.Sleep(450);
		}
		private void testGGS()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 8); // Команда на проверку ГГС 
			s_txt7 += ("Команда на проверку ГГС отправлена" + "\r\n");
			Thread.Sleep(250);
		}
		private void test_GG_Radio1()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 9);   //Команда на проверку 'ГГ-Радио1'
			s_txt7 += ("Команда на проверку 'ГГ-Радио1' отправлена" + "\r\n");
			Thread.Sleep(250);
		}
		private void test_GG_Radio2()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 10); // Команда на проверку 'ГГ-Радио2
			s_txt7 += ("Команда на проверку 'ГГ-Радио2' отправлена" + "\r\n");
			Thread.Sleep(250);
		}
		private void test_mikrophon()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 11);   // Команда на проверку 'Микрофон'
			s_txt7 = ("Команда на проверку 'Микрофон' отправлена" + "\r\n");
			Thread.Sleep(250);
		}
		private void test_power()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 17); // Проверка напряжения питания
			s_txt7 += ("Проверка напряжения питания" + "\r\n");
			Thread.Sleep(250);
		}
		private void test_valueDisp()
		{
			startWrReg = 120;
			res = myProtocol.writeSingleRegister(slave, startWrReg, 19); // Отключить все сенсоры
			s_txt7 += ("Проверка регулировки яркости дисплея" + "\r\n");
			Thread.Sleep(250);
		}
		#endregion

		private void test_end()
		{
			ushort[] waitVals = new ushort[4];
			bool[] coilArr = new bool[4];
			startRdReg = 120;                                                                        //regBank.add(40120)  adr_control_command Адрес передачи комманд на выполнение
			//  0 в регистре означает завершение выполнения фрагмента проверки
			numRdRegs = 2;
			int count = 0;
			do
			{
				res = myProtocol.readMultipleRegisters(slave, startRdReg, waitVals, numRdRegs);      // Ожидание кода подтверждения окончания проверки  Адрес передачи подтверждения 40120

				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = ("    MODBUS ON    " + count);
					MODBUS_SUCCESS = true;
					count++;
				}

				Thread.Sleep(200);
			} while (waitVals[0] != 0);                                                             // Если readVals[0] == 0 , тест завершен
			startCoil = 120;                                                                        // regBank.add(120) Флаг индикации возникновения любой ошибки
			numCoils = 2;
			res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);                        // Проверить Адрес 120  индикации возникновения любой ошибки
			if (coilArr[0] == true) //есть ошибка
			{
				// Обработка ошибки.
				s_txt48 += ("Ошибка! " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\t");
				s_txt8 = ("Ошибка!  " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
 				error_list();
			}
 			s_txt7 += ("Выполнение команды завершено" + "\r\n");
			All_Test_run = false;
		}
		private void test_end1()
		{
			ushort[] waitVals = new ushort[4];
			bool[] coilArr = new bool[2];
			startRdReg = 120;                                    //regBank.add(40120);    // adr_control_command Адрес передачи комманд на выполнение
			//  0 в регистре означает завершение выполнения фрагмента проверки
			numRdRegs = 2;
			do
			{
				res = myProtocol.readMultipleRegisters(slave, 120, waitVals, numRdRegs);  // Ожидание кода подтверждения окончания проверки  Адрес передачи подтверждения 40120

				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = "    MODBUS ON    ";
					MODBUS_SUCCESS = true;
				}

				else
				{
					toolStripStatusLabel1.Text = "    MODBUS ERROR (end1) ";
					toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
					Com2_SUCCESS               = false;
					MODBUS_SUCCESS             = false;
					Thread.Sleep(100);
					return;
				}
				Thread.Sleep(50);
			} while (waitVals[0] != 0);                                     // Если readVals[0] == 0 , тест завершен
		}
		private void system_ready()
		{
			bool[] coilArr = new bool[2];
			startCoil = 39;    // adr_control_command 
			numCoils = 1;
			//  true в регистре означает завершение выполнения программного сброса
			do
			{
				res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);    

				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = "    MODBUS ON    ";
					MODBUS_SUCCESS = true;
				}
				Thread.Sleep(50);
			} while (coilArr[0] == false);                                     // Если coilArr[0] == true , тест завершен
		}

		#region error list
		private void error_list()                                           //  Обработать информацию ошибок и вывети в окно 
		{
			error_list3();                                // Записать информацию регистров  200 - 330 флага индикации возникновения  ошибки

			if (coilArr_all[0] != false)
			{
				error_list_reg(1310, 2, 0, 0);
				s_txt8  += ("Сенсор  трубки не отключился                              \t< = " + readVals_all[0] + ">\r\n");
				s_txt48 += ("Сенсор  трубки не отключился                              \r\n");
 				res      = myProtocol.writeCoil(slave, 200, false);
			}

			if (coilArr_all[1] != false)
			{
				error_list_reg(1314, 2, 1, 1);
				s_txt8  += ("Сенсор Тангента ручная не отключился                      \t< = " + readVals_all[1] + ">\r\n");
				s_txt48 += ("Сенсор Тангента ручная не отключился         \r\n");
                res      = myProtocol.writeCoil(slave, 201, false);
			}

			if (coilArr_all[2] != false)
			{
				error_list_reg(1318, 2, 2, 2);
				s_txt8  += ("Сенсор Тангента ножная не отключился                      \t< = " + readVals_all[2] + ">\r\n");
				s_txt48 += ("Сенсор Тангента ножная не отключился          \r\n");
                res      = myProtocol.writeCoil(slave, 202, false);
			}

			if (coilArr_all[3] != false)
			{
				error_list_reg(1322, 2, 3, 3);
				s_txt8  += ("Сенсор гарнитуры инструктора с 2 наушниками  не отключился\t< = " + readVals_all[3] + ">\r\n");
				s_txt48 += ("Сенсор гарнитуры инструктора с 2 наушниками  не отключился\r\n");
                res      = myProtocol.writeCoil(slave, 203, false);
			}
			if (coilArr_all[4] != false)
			{
				error_list_reg(1326, 2, 4, 4);
				s_txt8  += ("Сенсор гарнитуры инструктора  не отключился               \t< = " + readVals_all[4] + ">\r\n");
				s_txt48 += ("Сенсор гарнитуры инструктора  не отключился               \r\n");
                res      = myProtocol.writeCoil(slave, 204, false);
			}
			if (coilArr_all[5] != false)
			{
				error_list_reg(1330, 2, 5, 5);
				s_txt8  += ("Сенсор диспетчера с 2 наушниками не отключился            \t< = " + readVals_all[5] + ">\r\n");
				s_txt48 += ("Сенсор диспетчера с 2 наушниками не отключился            \r\n");
                res      = myProtocol.writeCoil(slave, 205, false);
			}
			if (coilArr_all[6] != false)
			{
				error_list_reg(1334, 2, 6, 6);
				s_txt8  += ("Сенсор диспетчера не отключился                           \t< = " + readVals_all[6] + ">\r\n");
				s_txt48 += ("Сенсор диспетчера не отключился                           \r\n");
                res      = myProtocol.writeCoil(slave, 206, false);
			}
			if (coilArr_all[7] != false)
			{

				error_list_reg(1338, 2, 7, 7);
				s_txt8  += ("Сенсор Микрофона не отключился                            \t< = " + readVals_all[7] + ">\r\n");
				s_txt48 += ("Сенсор Микрофона не отключился                            \r\n");
                res      = myProtocol.writeCoil(slave, 207, false);
			}
			if (coilArr_all[8] != false)
			{
				error_list_reg(1342, 2, 8, 8);
				s_txt8  += ("Микрофон инструктора не отключился                        \t< = " + readVals_all[8] + ">\r\n");
				s_txt48 += ("Микрофон инструктора не отключился                        \r\n");
                res      = myProtocol.writeCoil(slave, 208, false);
			}
			if (coilArr_all[9] != false)
			{
				error_list_reg(1346, 2, 9, 9);
				s_txt8  += ("Микрофон диспетчера не отключился                         \t< = " + readVals_all[9] + ">\r\n");
				s_txt48 += ("Микрофон диспетчера не отключился                         \r\n");
                res      = myProtocol.writeCoil(slave, 209, false);
			}

			if (coilArr_all[10] != false)
			{
				error_list_reg(1350, 2, 10, 10);
				s_txt8  += ("Сенсор  трубки не включился                               \t\t< = " + readVals_all[10] + ">\r\n");
				s_txt48 += ("Сенсор  трубки не включился                               \r\n");
                res      = myProtocol.writeCoil(slave, 210, false);
			}

			if (coilArr_all[11] != false)
			{
				error_list_reg(1354, 2, 11, 11);
				s_txt8  += ("Сенсор Тангента ручная не включился                       \t< = " + readVals_all[11] + ">\r\n");
				s_txt48 += ("Сенсор Тангента ручная не включился                       \r\n");
                res      = myProtocol.writeCoil(slave, 211, false);
			}

			if (coilArr_all[12] != false)
			{
				error_list_reg(1358, 2, 12, 12);
				s_txt8  += ("Сенсор Тангента ножная не включился                       \t< = " + readVals_all[12] + ">\r\n");
				s_txt48 += ("Сенсор Тангента ножная не включился                       \r\n");
                res      = myProtocol.writeCoil(slave, 212, false);
			}

			if (coilArr_all[13] != false)
			{
				error_list_reg(1362, 2, 13, 13);
				s_txt8  += ("Сенсор гарнитуры инструктора с 2 наушниками  не включился \t< = " + readVals_all[13] + ">\r\n");
				s_txt48 += ("Сенсор гарнитуры инструктора с 2 наушниками  не включился \r\n");
				res      = myProtocol.writeCoil(slave, 213, false);
			}
			if (coilArr_all[14] != false)
			{
				error_list_reg(1366, 2, 14, 14);
				s_txt8  += ("Сенсор гарнитуры инструктора  не включился                \t< = " + readVals_all[14] + ">\r\n");
				s_txt48 += ("Сенсор гарнитуры инструктора  не включился                \r\n");
                res      = myProtocol.writeCoil(slave, 214, false);
			}
			if (coilArr_all[15] != false)
			{
				error_list_reg(1370, 2, 15, 15);
				s_txt8  += ("Сенсор диспетчера с 2 наушниками не включился             \t< = " + readVals_all[15] + ">\r\n");
				s_txt48 += ("Сенсор диспетчера с 2 наушниками не включился             \r\n");
				res      = myProtocol.writeCoil(slave, 215, false);
			}
			if (coilArr_all[16] != false)
			{
				error_list_reg(1374, 2, 16, 16);
				s_txt8  += ("Сенсор диспетчера не включился                            \t< = " + readVals_all[16] + ">\r\n");
				s_txt48 += ("Сенсор диспетчера не включился                            \r\n");
                res      = myProtocol.writeCoil(slave, 216, false);
			}
			if (coilArr_all[17] != false)
			{
				error_list_reg(1378, 2, 17, 17);
				s_txt8  += ("Сенсор Микрофона не включился                             \t< = " + readVals_all[17] + ">\r\n");
				s_txt48 += ("Сенсор Микрофона не включился                             \r\n");
				res      = myProtocol.writeCoil(slave, 217, false);
			}
			if (coilArr_all[18] != false)
			{
				error_list_reg(1382, 2, 18, 18);
				s_txt8  += ("Микрофон инструктора не включился                         \t< = " + readVals_all[18] + ">\r\n");
				s_txt48 += ("Микрофон инструктора не включился                         \r\n");
                res      = myProtocol.writeCoil(slave, 218, false);
			}
			if (coilArr_all[19] != false)
			{
				error_list_reg(1386, 2, 19, 19);
				s_txt8  += ("Микрофон диспетчера не включился                          \t< = " + readVals_all[19] + ">\r\n");
				s_txt48 += ("Микрофон диспетчера не включился                          \r\n");
                res      = myProtocol.writeCoil(slave, 219, false);
			}

			if (coilArr_all[20] != false)
			{
				error_list_reg(1390, 2, 20, 20);
				s_txt8  += ("PTT инструктора не отключился                             \t< = " + readVals_all[20] + ">\r\n");
				s_txt48 += ("PTT инструктора не отключился                             \r\n");
                res      = myProtocol.writeCoil(slave, 220, false);
			}

			if (coilArr_all[21] != false)
			{
				error_list_reg(1394, 2, 21, 21);
				s_txt8  += ("PTT инструктора не включился                              \t\t< = " + readVals_all[21] + ">\r\n");
				s_txt48 += ("PTT инструктора не включился                              \t\r\n");
                res      = myProtocol.writeCoil(slave, 221, false);
			}

			if (coilArr_all[22] != false)
			{
				error_list_reg(1398, 2, 22, 22);
				s_txt8  += ("PTT диспетчера не отключился                              \t< = " + readVals_all[22] + ">\r\n");
				s_txt48 += ("PTT диспетчера не отключился                              \t\r\n");
                res      = myProtocol.writeCoil(slave, 222, false);
			}

			if (coilArr_all[23] != false)
			{
				error_list_reg(1402, 2, 23, 23);
				s_txt8  += ("PTT диспетчера не включился                               \t\t< = " + readVals_all[23] + ">\r\n");
				s_txt48 += ("PTT диспетчера не включился                               \t\r\n");
                res      = myProtocol.writeCoil(slave, 223, false);
			}
			if (coilArr_all[24] != false)
			{
				error_list_reg(1406, 2, 24, 24);
				temp_disp = readVolt_all[24];
				s_txt8   += ("Сигнал инструктора LineL не в норме                     \t< = " + readVals_all[24] + ">  " + temp_disp / 100 + " V \r\n");
				s_txt48  += ("Сигнал инструктора LineL не в норме                     \t = " + temp_disp / 100 + " V \r\n");
                res       = myProtocol.writeCoil(slave, 224, false);
			}
			if (coilArr_all[25] != false)
			{
				error_list_reg(1410, 2, 25, 25);
				temp_disp = readVolt_all[25];
				s_txt8   += ("Сигнал инструктора LineR не в норме                     \t< = " + readVals_all[25] + ">  " + temp_disp / 100 + " V \r\n");
				s_txt48  += ("Сигнал инструктора LineR не в норме                     \t = " + temp_disp / 100 + " V \r\n");
				res       = myProtocol.writeCoil(slave, 225, false);
			}
			if (coilArr_all[26] != false)
			{
				error_list_reg(1414, 2, 26, 26);
				temp_disp = readVolt_all[26];
				s_txt8   += ("Сигнал инструктора на разъеме Маг phone не в норме \t< = " + readVals_all[26] + ">  " + temp_disp / 100 + " V \r\n");
				s_txt48  += ("Сигнал инструктора на разъеме Маг phone не в норме \t = " + temp_disp / 100 + " V \r\n");
                res       = myProtocol.writeCoil(slave, 226, false);
			}
			if (coilArr_all[27] != false)
			{
				error_list_reg(1418, 2, 27, 27);
				temp_disp = readVolt_all[27];
				s_txt8   += ("Сигнал диспетчера LineL не в норме                      \t< = " + readVals_all[27] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Сигнал диспетчера LineL не в норме                      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 227, false);
			}
			if (coilArr_all[28] != false)
			{
				error_list_reg(1422, 2, 28, 28);
				temp_disp = readVolt_all[28];
				s_txt8   += ("Сигнал диспетчера LineR не в норме                      \t< = " + readVals_all[28] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Сигнал диспетчера LineR не в норме                      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 228, false);
			}
			if (coilArr_all[29] != false)
			{
				error_list_reg(1426, 2, 29, 29);
				temp_disp = readVolt_all[29];
				s_txt8   += ("Сигнал диспетчера на разъеме Маг phone не в норме       \t< = " + readVals_all[29] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Сигнал диспетчера на разъеме Маг phone не в норме       \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 229, false);
			}

			if (coilArr_all[30] != false)
			{
				error_list_reg(1430, 2, 30, 30);
				temp_disp = readVolt_all[30];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал FrontL    \tOFF     \t< = " + readVals_all[30] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48  += ("Тест гарнитуры инструктора ** Сигнал FrontL    \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 230, false);
			}

			if (coilArr_all[31] != false)
			{
				error_list_reg(1434, 2, 31, 31);
				temp_disp = readVolt_all[31];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал FrontR    \tOFF     \t< = " + readVals_all[31] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48  += ("Тест гарнитуры инструктора ** Сигнал FrontR    \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 231, false);
			}

			if (coilArr_all[32] != false)
			{
				error_list_reg(1438, 2, 32, 32);
				temp_disp = readVolt_all[32];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал LineL     \tOFF     \t< = " + readVals_all[32] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48 += ("Тест гарнитуры инструктора ** Сигнал LineL      \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 232, false);
			}

			if (coilArr_all[33] != false)
			{
				error_list_reg(1442, 2, 33, 33);
				temp_disp = readVolt_all[33];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал LineR     \tOFF     \t< = " + readVals_all[33] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48  += ("Тест гарнитуры инструктора ** Сигнал LineR     \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 233, false);
			}
			if (coilArr_all[34] != false)
			{
				error_list_reg(1446, 2, 34, 34);
				temp_disp = readVolt_all[34];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал mag radio \tOFF     \t< = " + readVals_all[34] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48  += ("Тест гарнитуры инструктора ** Сигнал mag radio \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 234, false);
			}
			if (coilArr_all[35] != false)
			{
				error_list_reg(1450, 2, 35, 35);
				temp_disp = readVolt_all[35];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал mag phone \tOFF     \t< = " + readVals_all[35] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48  += ("Тест гарнитуры инструктора ** Сигнал mag phone \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 235, false);
			}
			if (coilArr_all[36] != false)
			{
				error_list_reg(1454, 2, 36, 36);
				temp_disp = readVolt_all[36];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал ГГС       \tOFF     \t< = " + readVals_all[36] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры инструктора ** Сигнал ГГС       \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 236, false);
			}
			if (coilArr_all[37] != false)
			{
				error_list_reg(1458, 2, 37, 37);
				temp_disp = readVolt_all[37];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал ГГ Радио1 \tOFF     \t< = " + readVals_all[37] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48  += ("Тест гарнитуры инструктора ** Сигнал ГГ Радио1 \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 237, false);
			}
			if (coilArr_all[38] != false)
			{
				error_list_reg(1462, 2, 38, 38);
				temp_disp = readVolt_all[38];
				s_txt8   += ("Тест гарнитуры инструктора ** Сигнал ГГ Радио2 \tOFF     \t< = " + readVals_all[38] + ">  " + temp_disp / 100 + " V\r\n");
                s_txt48  += ("Тест гарнитуры инструктора ** Сигнал ГГ Радио2 \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 238, false);
			}
			if (coilArr_all[39] != false)
			{
				error_list_reg(1466, 2, 39, 39);
				temp_disp = readVolt_all[39];
				s_txt8   += ("Ошибка! Ток устройства не в норме  < = " + readVals_all[38] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Ошибка! Ток устройства не в норме  < = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 239, false);
			}

			if (coilArr_all[40] != false)
			{
				error_list_reg(1470, 2, 40, 40);
				temp_disp = readVolt_all[40];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал FrontL     \tOFF     \t< = " + readVals_all[40] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал FrontL     \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 240, false);
			}

			if (coilArr_all[41] != false)
			{
				error_list_reg(1474, 2, 41, 41);
				temp_disp = readVolt_all[41];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал FrontR     \tOFF     \t< = " + readVals_all[41] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал FrontR     \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 241, false);
			}

			if (coilArr_all[42] != false)
			{
				error_list_reg(1478, 2, 42, 42);
				temp_disp = readVolt_all[42];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал LineL      \tOFF     \t< = " + readVals_all[42] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал LineL      \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 242, false);
			}

			if (coilArr_all[43] != false)
			{
				error_list_reg(1482, 2, 43, 43);
				temp_disp = readVolt_all[43];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал LineR      \tOFF     \t< = " + readVals_all[43] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал LineR      \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 243, false);
			}
			if (coilArr_all[44] != false)
			{
				error_list_reg(1486, 2, 44, 44);
				temp_disp = readVolt_all[44];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал mag radio  \tOFF     \t< = " + readVals_all[44] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал mag radio  \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 244, false);
			}
			if (coilArr_all[45] != false)
			{
				error_list_reg(1490, 2, 45, 45);
				temp_disp = readVolt_all[45];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал mag phone  \tOFF     \t< = " + readVals_all[45] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал mag phone  \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 245, false);
			}
			if (coilArr_all[46] != false)
			{
				error_list_reg(1494, 2, 46, 46);
				temp_disp = readVolt_all[46];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал ГГС        \tOFF     \t< = " + readVals_all[46] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал ГГС        \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 246, false);
			}
			if (coilArr_all[47] != false)
			{
				error_list_reg(1498, 2, 47, 47);
				temp_disp = readVolt_all[47];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал ГГ Радио1  \tOFF     \t< = " + readVals_all[47] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал ГГ Радио1  \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 247, false);
			}
			if (coilArr_all[48] != false)
			{
				error_list_reg(1502, 2, 48, 48);
				temp_disp = readVolt_all[48];
				s_txt8   += ("Тест гарнитуры диспетчера ** Сигнал ГГ Радио2  \tOFF     \t< = " + readVals_all[48] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест гарнитуры диспетчера ** Сигнал ГГ Радио2  \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 248, false);
			}
			if (coilArr_all[49] != false)
			{
				error_list_reg(1506, 2, 49, 49);
				temp_disp = readVolt_all[49];
				s_txt8   += ("Ошибка! Ток устройства не в норме                        \t< = " + readVals_all[49] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Ошибка! Ток устройства не в норме                        \t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 249, false);
			}

			if (coilArr_all[50] != false)
			{
				error_list_reg(1510, 2, 50, 50);
				temp_disp = readVolt_all[50];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал FrontL             \tOFF     \t< = " + readVals_all[50] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал FrontL             \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 250, false);
			}

			if (coilArr_all[51] != false)
			{
				error_list_reg(1514, 2, 51, 51);
				temp_disp = readVolt_all[51];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал FrontR             \tOFF     \t< = " + readVals_all[51] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал FrontR             \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 251, false);
			}

			if (coilArr_all[52] != false)
			{
				error_list_reg(1518, 2, 52, 52);
				temp_disp = readVolt_all[52];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал LineL              \tOFF     \t< = " + readVals_all[52] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал LineL              \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 252, false);
			}

			if (coilArr_all[53] != false)
			{
				error_list_reg(1522, 2, 53, 53);
				temp_disp = readVolt_all[53];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал LineR              \tOFF     \t< = " + readVals_all[53] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал LineR              \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 253, false);
			}
			if (coilArr_all[54] != false)
			{
				error_list_reg(1526, 2, 54, 54);
				temp_disp = readVolt_all[54];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал mag radio          \tOFF     \t< = " + readVals_all[54] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал mag radio          \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 254, false);
			}
			if (coilArr_all[55] != false)
			{
				error_list_reg(1530, 2, 55, 55);
				temp_disp = readVolt_all[55];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал mag phone          \tOFF     \t< = " + readVals_all[55] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал mag phone          \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 255, false);
			}
			if (coilArr_all[56] != false)
			{
				error_list_reg(1534, 2, 56, 56);
				temp_disp = readVolt_all[56];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал ГГС                \tOFF     \t< = " + readVals_all[56] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал ГГС                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 256, false);
			}
			if (coilArr_all[57] != false)
			{
				error_list_reg(1538, 2, 57, 57);
				temp_disp = readVolt_all[57];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал ГГ Радио1          \tOFF     \t< = " + readVals_all[57] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал ГГ Радио1          \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 257, false);
			}
			if (coilArr_all[58] != false)
			{
				error_list_reg(1542, 2, 58, 58);
				temp_disp = readVolt_all[58];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал ГГ Радио2          \tOFF     \t< = " + readVals_all[58] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал ГГ Радио2          \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 258, false);
			}
			if (coilArr_all[59] != false)
			{
				error_list_reg(1546, 2, 59, 59);
				temp_disp = readVolt_all[59];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал ГГС                \tON      \t< = " + readVals_all[59] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал ГГС                \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 259, false);
			}

			if (coilArr_all[60] != false)
			{
				error_list_reg(1550, 2, 60, 60);
				temp_disp = readVolt_all[60];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал LineL              \tON      \t< = " + readVals_all[60] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал LineL              \tON      \t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 260, false);
			}

			if (coilArr_all[61] != false)
			{
				error_list_reg(1554, 2, 61, 61);
				temp_disp = readVolt_all[61];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал LineR              \tON      \t< = " + readVals_all[61] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал LineR              \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 261, false);
			}

			if (coilArr_all[62] != false)
			{
				error_list_reg(1558, 2, 62, 62);
				temp_disp = readVolt_all[62];
				s_txt8   += ("Тест МТТ (трубки) ** Сигнал Mag phone          \tON      \t< = " + readVals_all[62] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) ** Сигнал Mag phone          \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 262, false);
			}

			if (coilArr_all[63] != false)
			{
				error_list_reg(1562, 2, 63, 63);
				temp_disp = readVolt_all[63];
				s_txt8   += ("Тест МТТ (трубки) PTT    (CTS)                 \tOFF     \t< = " + readVals_all[63] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест МТТ (трубки) PTT    (CTS)                 \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 263, false);
			}
			if (coilArr_all[64] != false)
			{
				error_list_reg(1566, 2, 64, 64);
				s_txt8  += ("Тест микрофона PTT  (CTS)                       \t OFF    \t< = " + readVals_all[64] + ">\r\n");
				s_txt48 += ("Тест микрофона PTT  (CTS)                       \tОтключен\r\n");
                res      = myProtocol.writeCoil(slave, 264, false);
			}
			if (coilArr_all[65] != false)
			{
				error_list_reg(1570, 2, 65, 65);
				s_txt8  += ("Тест МТТ (трубки) PTT    (CTS)                  \tON      \t< = " + readVals_all[65] + ">\r\n");
				s_txt48 += ("Тест МТТ (трубки) PTT    (CTS)                  \tON      \r\n");
                res      = myProtocol.writeCoil(slave, 265, false);
			}
			if (coilArr_all[66] != false)
			{
				error_list_reg(1574, 2, 66, 66);
				s_txt8  += ("Тест микрофона PTT  (CTS)                       \tON      \t< = " + readVals_all[66] + ">\r\n");
				s_txt48 += ("Тест микрофона PTT  (CTS)                       \tON      \r\n");
                res      = myProtocol.writeCoil(slave, 266, false);
			}
			if (coilArr_all[67] != false)
			{
				error_list_reg(1578, 2, 67, 67);
				s_txt8  += ("Тест МТТ (трубки) HangUp (DCD)                  \tOFF     \t< = " + readVals_all[67] + ">\r\n");
				s_txt48 += ("Тест МТТ (трубки) HangUp (DCD)                  \tОтключен\r\n");
                res      = myProtocol.writeCoil(slave, 267, false);
			}
			if (coilArr_all[68] != false)
			{
				error_list_reg(1582, 2, 68, 68);
				s_txt8  += ("Тест МТТ (трубки) HangUp (DCD)                  \tON      \t< = " + readVals_all[68] + ">\r\n");
				s_txt48 += ("Тест МТТ (трубки) HangUp (DCD)                  \tON      \r\n");
                res      = myProtocol.writeCoil(slave, 268, false);
			}
			if (coilArr_all[69] != false)
			{
				error_list_reg(1586, 2, 69, 69);
				temp_disp = readVolt_all[69];        // 
				s_txt8   += ("Длительность импульса регулировки яркости дисплея\t< = " + readVals_all[69] + ">  " + temp_disp + " мкс\r\n");
				s_txt48  += ("Длительность импульса регулировки яркости дисплея\t = " + temp_disp + " мкс\r\n");
                res       = myProtocol.writeCoil(slave, 269, false);
			}

			if (coilArr_all[70] != false)
			{
				error_list_reg(1590, 2, 70, 70);
				s_txt8  += ("Команда PTT1 тангента ручная (CTS)              \tOFF     \t< = " + readVals_all[70] + ">\r\n");
				s_txt48 += ("Команда PTT1 тангента ручная (CTS)              \tОтключен\r\n");
                res      = myProtocol.writeCoil(slave, 270, false);
			}

			if (coilArr_all[71] != false)
			{
				error_list_reg(1594, 2, 71, 71);
				s_txt8  += ("Команда PTT2 тангента ручная (DCR)              \tOFF     \t< = " + readVals_all[71] + ">\r\n");
				s_txt48 += ("Команда PTT2 тангента ручная (DCR)              \tОтключен\r\n");
                res      = myProtocol.writeCoil(slave, 271, false);
			}

			if (coilArr_all[72] != false)
			{
				error_list_reg(1598, 2, 72, 72);
				s_txt8  += ("Команда PTT1 тангента ручная (CTS)              \tON      \t< = " + readVals_all[72] + ">\r\n");
				s_txt48 += ("Команда PTT1 тангента ручная (CTS)              \tON      \r\n");
                res      = myProtocol.writeCoil(slave, 272, false);
			}

			if (coilArr_all[73] != false)
			{
				error_list_reg(1602, 2, 73, 73);
				s_txt8  += ("Команда PTT2 тангента ручная (DCR)              \tON      \t< = " + readVals_all[73] + ">\r\n");
				s_txt48 += ("Команда PTT2 тангента ручная (DCR)              \tON      \r\n");
				res      = myProtocol.writeCoil(slave, 273, false);
			}
			if (coilArr_all[74] != false)
			{
				error_list_reg(1606, 2, 74, 74);
				s_txt8  += ("Команда сенсор тангента ручная                  \tOFF     \t< = " + readVals_all[74] + ">\r\n");
				s_txt48 += ("Команда сенсор тангента ручная                  \tОтключен\r\n");
                res      = myProtocol.writeCoil(slave, 274, false);
			}
			if (coilArr_all[75] != false)
			{
				error_list_reg(1610, 2, 75, 75);
				s_txt8  += ("Команда сенсор тангента ручная                  \tON      \t< = " + readVals_all[75] + ">\r\n");
				s_txt48 += ("Команда сенсор тангента ручная                  \tON      \r\n");
				res      = myProtocol.writeCoil(slave, 275, false);
			}
			if (coilArr_all[76] != false)
			{
				error_list_reg(1614, 2, 76, 76);
				s_txt8 += ("Команда сенсор тангента ножная                  \tOFF     \t< = " + readVals_all[76] + ">\r\n");
				s_txt48 += ("Команда сенсор тангента ножная                  \tОтключен\r\n");
                res = myProtocol.writeCoil(slave, 276, false);
			}
			if (coilArr_all[77] != false)
			{
				error_list_reg(1618, 2, 77, 77);
				s_txt8  += ("Команда сенсор тангента ножная                  \tON      \t< = " + readVals_all[77] + ">\r\n");
				s_txt48 += ("Команда сенсор тангента ножная                  \tON      \r\n");
				res      = myProtocol.writeCoil(slave, 277, false);
			}
			if (coilArr_all[78] != false)
			{
				error_list_reg(1622, 2, 78, 78);
				s_txt8  += ("Команда PTT тангента ножная (CTS)               \tOFF     \t< = " + readVals_all[78] + ">\r\n");
				s_txt48 += ("Команда PTT тангента ножная (CTS)               \tОтключен\r\n");
                res      = myProtocol.writeCoil(slave, 278, false);
			}
			if (coilArr_all[79] != false)
			{
				error_list_reg(1626, 2, 79, 79);
				s_txt8  += ("Команда PTT тангента ножная (CTS)               \tON      \t< = " + readVals_all[79] + ">\r\n");
				s_txt48 += ("Команда PTT тангента ножная (CTS)               \tON      \r\n");
				res      = myProtocol.writeCoil(slave, 279, false);
			}

			if (coilArr_all[80] != false)
			{
				error_list_reg(1630, 2, 80, 80);
				temp_disp = readVolt_all[80];
				s_txt8   += ("Тест ГГС ** Сигнал FrontL                      \tOFF     \t< = " + readVals_all[80] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал FrontL                      \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 280, false);
			}

			if (coilArr_all[81] != false)
			{
				error_list_reg(1634, 2, 81, 81);
				temp_disp = readVolt_all[81];
				s_txt8   += ("Тест ГГС ** Сигнал FrontR                      \tOFF     \t< = " + readVals_all[81] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал FrontR                      \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 281, false);
			}

			if (coilArr_all[82] != false)
			{
				error_list_reg(1638, 2, 82, 82);
				temp_disp = readVolt_all[82];
				s_txt8   += ("Тест ГГС ** Сигнал LineL                       \tOFF     \t< = " + readVals_all[82] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал LineL                       \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 282, false);
			}

			if (coilArr_all[83] != false)
			{
				error_list_reg(1642, 2, 83, 83);
				temp_disp = readVolt_all[83];
				s_txt8   += ("Тест ГГС ** Сигнал LineR                       \tOFF     \t< = " + readVals_all[83] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал LineR                       \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 283, false);
			}
			if (coilArr_all[84] != false)
			{
				error_list_reg(1646, 2, 84, 84);
				temp_disp = readVolt_all[84];
				s_txt8 += ("Тест ГГС ** Сигнал mag radio                   \tOFF     \t< = " + readVals_all[84] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48 += ("Тест ГГС ** Сигнал mag radio                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res = myProtocol.writeCoil(slave, 284, false);
			}
			if (coilArr_all[85] != false)
			{
				error_list_reg(1650, 2, 85, 85);
				temp_disp = readVolt_all[85];
				s_txt8   += ("Тест ГГС ** Сигнал mag phone                   \tOFF     \t< = " + readVals_all[85] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал mag phone                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 285, false);
			}
			if (coilArr_all[86] != false)
			{
				error_list_reg(1654, 2, 86, 86);
				temp_disp = readVolt_all[86];
				s_txt8   += ("Тест ГГС ** Сигнал ГГС                         \tOFF     \t< = " + readVals_all[86] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал ГГС                         \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 286, false);
			}
			if (coilArr_all[87] != false)
			{
				error_list_reg(1658, 2, 87, 87);
				temp_disp = readVolt_all[87];
				s_txt8   += ("Тест ГГС ** Сигнал ГГ Радио1                   \tOFF     \t< = " + readVals_all[87] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал ГГ Радио1                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 287, false);
			}
			if (coilArr_all[88] != false)
			{
				error_list_reg(1662, 2, 88, 88);
				temp_disp = readVolt_all[88];
				s_txt8   += ("Тест ГГС ** Сигнал ГГ Радио2                   \tOFF     \t< = " + readVals_all[88] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал ГГ Радио2                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 288, false);
			}
			if (coilArr_all[89] != false)
			{
				error_list_reg(1666, 2, 89, 89);
				temp_disp = readVolt_all[89];
				s_txt8   += ("Тест ГГС ** Сигнал ГГС                         \t\tON    \t< = " + readVals_all[89] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал ГГС                         \t\tON    \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 289, false);
			}

			if (coilArr_all[90] != false)
			{
				error_list_reg(1670, 2, 90, 90);
				temp_disp = readVolt_all[90];
				s_txt8   += ("Тест ГГС ** Сигнал FrontL                      \tON      \t< = " + readVals_all[90] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал FrontL                      \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 290, false);
			}

			if (coilArr_all[91] != false)
			{
				error_list_reg(1674, 2, 91, 91);
				temp_disp = readVolt_all[91];
				s_txt8   += ("Тест ГГС ** Сигнал FrontR                      \tON      \t< = " + readVals_all[91] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал FrontR                      \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 291, false);
			}

			if (coilArr_all[92] != false)
			{
				error_list_reg(1678, 2, 92, 92);
				temp_disp = readVolt_all[92];
				s_txt8   += ("Тест ГГС ** Сигнал mag phone                   \tON      \t< = " + readVals_all[92] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест ГГС ** Сигнал mag phone                   \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 292, false);
			}

			if (coilArr_all[93] != false)
			{
				error_list_reg(1682, 2, 93, 93);
				temp_disp = readVolt_all[93];
				s_txt8   += ("Напряжение питания модуля не в норме                     \t< = " + readVals_all[93] + ">  " + temp_disp * 2.51 / 100 + " V\r\n");
				s_txt48  += ("Напряжение питания модуля не в норме                     \t = " + temp_disp * 2.51 / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 293, false);
			}
			if (coilArr_all[94] != false)
			{
				error_list_reg(1686, 2, 94, 94);
				temp_disp = readVolt_all[94];
				s_txt8   += ("Напряжение питания Радио1 не в норме                     \t< = " + readVals_all[94] + ">  " + temp_disp * 2.51 / 100 + " V\r\n");
				s_txt48  += ("Напряжение питания Радио1 не в норме                     \t = " + temp_disp * 2.51 / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 294, false);
			}
			if (coilArr_all[95] != false)
			{
				error_list_reg(1690, 2, 95, 95);
				temp_disp = readVolt_all[95];
				s_txt8   += ("Напряжение питания Радио2 не в норме                     \t< = " + readVals_all[95] + ">  " + temp_disp * 2.51 / 100 + " V\r\n");
				s_txt48  += ("Напряжение питания Радио2 не в норме                     \t = " + temp_disp * 2.51 / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 295, false);
			}
			if (coilArr_all[96] != false)
			{
				error_list_reg(1694, 2, 96, 96);
				temp_disp = readVolt_all[96];
				s_txt8   += ("Напряжение питания ГГС  не в норме                       \t< = " + readVals_all[96] + ">  " + temp_disp * 2.51 / 100 + " V\r\n");
				s_txt48  += ("Напряжение питания ГГС  не в норме                       \t = " + temp_disp * 2.51 / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 296, false);
			}
			if (coilArr_all[97] != false)
			{
				error_list_reg(1698, 2, 97, 97);
				temp_disp = readVolt_all[97];
				s_txt8   += ("Напряжение питания светодиода микрофона не в норме       \t< = " + readVals_all[97] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Напряжение питания светодиода микрофона не в норме       \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 297, false);
			}
			if (coilArr_all[98] != false)
			{
				error_list_reg(1702, 2, 98, 98);
				temp_disp = readVolt_all[98];
				s_txt8   += ("Тест микрофона ** Сигнал mag phone             \tON      \t< = " + readVals_all[98] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал mag phone             \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 298, false);
			}
			if (coilArr_all[99] != false)
			{
				error_list_reg(1706, 2, 99, 99);
				temp_disp = readVolt_all[99];
				s_txt8   += ("Тест микрофона ** Сигнал LineL                 \tON      \t< = " + readVals_all[99] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал LineL                 \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 299, false);
			}

			if (coilArr_all[100] != false)
			{
				error_list_reg(1710, 2, 100, 100);
				temp_disp = readVolt_all[100];
				s_txt8   += ("Тест Радио1 ** Сигнал FrontL                   \tOFF     \t< = " + readVals_all[100] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал FrontL                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 300, false);
			}

			if (coilArr_all[101] != false)
			{
				error_list_reg(1714, 2, 101, 101);
				temp_disp = readVolt_all[101];
				s_txt8   += ("Тест Радио1 ** Сигнал FrontR                   \tOFF     \t< = " + readVals_all[101] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал FrontR                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 301, false);
			}

			if (coilArr_all[102] != false)
			{
				error_list_reg(1718, 2, 102, 102);
				temp_disp = readVolt_all[102];
				s_txt8   += ("Тест Радио1 ** Сигнал LineL                    \tOFF     \t< = " + readVals_all[102] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал LineL                    \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 302, false);
			}

			if (coilArr_all[103] != false)
			{
				error_list_reg(1722, 2, 103, 103);
				temp_disp = readVolt_all[103];
				s_txt8   += ("Тест Радио1 ** Сигнал LineR                    \tOFF     \t< = " + readVals_all[103] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал LineR                    \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 303, false);
			}
			if (coilArr_all[104] != false)
			{
				error_list_reg(1726, 2, 104, 104);
				temp_disp = readVolt_all[104];
				s_txt8   += ("Тест Радио1 ** Сигнал mag radio                \tOFF     \t< = " + readVals_all[104] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал mag radio                \tОтключен\t< = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 304, false);
			}
			if (coilArr_all[105] != false)
			{
				error_list_reg(1730, 2, 105, 105);
				temp_disp = readVolt_all[105];
				s_txt8   += ("Тест Радио1 ** Сигнал mag phone                \tOFF     \t< = " + readVals_all[105] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал mag phone                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 305, false);
			}
			if (coilArr_all[106] != false)
			{
				error_list_reg(1734, 2, 106, 106);
				temp_disp = readVolt_all[6];
				s_txt8   += ("Тест Радио1 ** Сигнал ГГС                      \tOFF     \t< = " + readVals_all[106] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал ГГС                      \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 306, false);
			}
			if (coilArr_all[107] != false)
			{
				error_list_reg(1738, 2, 107, 107);
				temp_disp = readVolt_all[107];
				s_txt8   += ("Тест Радио1 ** Сигнал ГГ Радио1                \tOFF     \t< = " + readVals_all[107] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал ГГ Радио1                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 307, false);
			}
			if (coilArr_all[108] != false)
			{
				error_list_reg(1742, 2, 108, 108);
				temp_disp = readVolt_all[108];
				s_txt8   += ("Тест Радио1 ** Сигнал ГГ Радио2                \tOFF     \t< = " + readVals_all[108] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал ГГ Радио2                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 308, false);
			}
			if (coilArr_all[109] != false)
			{
				error_list_reg(1746, 2, 109, 109);
				temp_disp = readVolt_all[109];
				s_txt8   += ("Тест Радио1 ** Сигнал Radio1                   \tON      \t< = " + readVals_all[109] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио1 ** Сигнал Radio1                   \t\tON    \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 309, false);
			}

			if (coilArr_all[110] != false)
			{
				error_list_reg(1750, 2, 110, 110);
				temp_disp = readVolt_all[110];
				s_txt8   += ("Тест Радио2 ** Сигнал FrontL                   \tOFF     \t< = " + readVals_all[110] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал FrontL                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 310, false);
			}

			if (coilArr_all[111] != false)
			{
				error_list_reg(1754, 2, 111, 111);
				temp_disp = readVolt_all[111];
				s_txt8   += ("Тест Радио2 ** Сигнал FrontR                   \tOFF     \t< = " + readVals_all[111] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал FrontR                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 311, false);
			}

			if (coilArr_all[112] != false)
			{
				error_list_reg(1758, 2, 112, 112);
				temp_disp = readVolt_all[112];
				s_txt8   += ("Тест Радио2 ** Сигнал LineL                    \tOFF     \t< = " + readVals_all[112] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал LineL                    \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 312, false);
			}

			if (coilArr_all[113] != false)
			{
				error_list_reg(1762, 2, 113, 113);
				temp_disp = readVolt_all[113];
				s_txt8   += ("Тест Радио2 ** Сигнал LineR                    \tOFF     \t< = " + readVals_all[113] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал LineR                    \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 313, false);
			}
			if (coilArr_all[114] != false)
			{
				error_list_reg(1766, 2, 114, 114);
				temp_disp = readVolt_all[114];
				s_txt8   += ("Тест Радио2 ** Сигнал mag radio                \tOFF     \t< = " + readVals_all[114] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал mag radio                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 314, false);
			}
			if (coilArr_all[115] != false)
			{
				error_list_reg(1770, 2, 115, 115);
				temp_disp = readVolt_all[115];
				s_txt8   += ("Тест Радио2 ** Сигнал mag phone                \tOFF     \t< = " + readVals_all[115] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал mag phone                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 315, false);
			}
			if (coilArr_all[116] != false)
			{
				error_list_reg(1774, 2, 116, 116);
				temp_disp = readVolt_all[116];
				s_txt8   += ("Тест Радио2 ** Сигнал ГГС                      \tOFF     \t< = " + readVals_all[116] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал ГГС                      \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 316, false);
			}
			if (coilArr_all[117] != false)
			{
				error_list_reg(1778, 2, 117, 117);
				temp_disp = readVolt_all[117];
				s_txt8   += ("Тест Радио2 ** Сигнал ГГ Радио1                \tOFF     \t< = " + readVals_all[117] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал ГГ Радио1                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 317, false);
			}
			if (coilArr_all[118] != false)
			{
				error_list_reg(1782, 2, 118, 118);
				temp_disp = readVolt_all[118];
				s_txt8   += ("Тест Радио2 ** Сигнал ГГ Радио2                \tOFF     \t< = " + readVals_all[118] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал ГГ Радио2                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 318, false);
			}
			if (coilArr_all[119] != false)
			{
				error_list_reg(1786, 2, 119, 119);
				temp_disp = readVolt_all[119];
				s_txt8   += ("Тест Радио2 ** Сигнал Радио2                   \tON      \t< = " + readVals_all[119] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест Радио2 ** Сигнал Радио2                   \tON      \t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 319, false);
			}

			if (coilArr_all[120] != false)
			{
				error_list_reg(1790, 2, 120, 120);
				temp_disp = readVolt_all[120];
				s_txt8   += ("Тест микрофона ** Сигнал FrontL                \tOFF     \t< = " + readVals_all[120] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал FrontL                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 320, false);
			}

			if (coilArr_all[121] != false)
			{
				error_list_reg(1794, 2, 121, 121);
				temp_disp = readVolt_all[121];
				s_txt8   += ("Тест микрофона ** Сигнал FrontR                \tOFF     \t< = " + readVals_all[121] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал FrontR                \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 321, false);
			}

			if (coilArr_all[122] != false)
			{
				error_list_reg(1798, 2, 122, 122);
				temp_disp = readVolt_all[122];
				s_txt8   += ("Тест микрофона ** Сигнал LineL                 \tOFF     \t< = " + readVals_all[122] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал LineL                 \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 322, false);
			}

			if (coilArr_all[123] != false)
			{
				error_list_reg(1802, 2, 123, 123);
				temp_disp = readVolt_all[123];
				s_txt8   += ("Тест микрофона ** Сигнал LineR                 \tOFF     \t< = " + readVals_all[123] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал LineR                 \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 323, false);
			}
			if (coilArr_all[124] != false)
			{
				error_list_reg(1806, 2, 124, 124);
				temp_disp = readVolt_all[124];
				s_txt8   += ("Тест микрофона ** Сигнал mag radio             \tOFF     \t< = " + readVals_all[124] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал mag radio             \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 324, false);
			}
			if (coilArr_all[125] != false)
			{
				error_list_reg(1810, 2, 125, 125);
				temp_disp = readVolt_all[125];
				s_txt8   += ("Тест микрофона ** Сигнал mag phone             \tOFF     \t< = " + readVals_all[125] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал mag phone             \tОтключен\t = " + temp_disp / 100 + " V\r\n");
				res       = myProtocol.writeCoil(slave, 325, false);
			}
			if (coilArr_all[126] != false)
			{
				error_list_reg(1814, 2, 126, 126);
				temp_disp = readVolt_all[126];
				s_txt8   += ("Тест микрофона ** Сигнал ГГС                   \tOFF     \t< = " + readVals_all[126] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал ГГС                   \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 326, false);
			}
			if (coilArr_all[127] != false)
			{
				error_list_reg(1818, 2, 127, 127);
				temp_disp = readVolt_all[127];
				s_txt8   += ("Тест микрофона ** Сигнал ГГ Радио1             \tOFF     \t< = " + readVals_all[127] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал ГГ Радио1             \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 327, false);
			}
			if (coilArr_all[128] != false)
			{
				error_list_reg(1822, 2, 128, 128);
				temp_disp = readVolt_all[128];
				s_txt8   += ("Тест микрофона ** Сигнал ГГ Радио2             \tOFF     \t< = " + readVals_all[128] + ">  " + temp_disp / 100 + " V\r\n");
				s_txt48  += ("Тест микрофона ** Сигнал ГГ Радио2             \tОтключен\t = " + temp_disp / 100 + " V\r\n");
                res       = myProtocol.writeCoil(slave, 328, false);
			}
			if (coilArr_all[129] != false)
			{
				error_list_reg(1826, 2, 129, 129);
				temp_disp = readVolt_all[129];
				s_txt8   += ("Код регулировки яркости дисплея не совпадает             \t< = " + readVals_all[129] + ">  " + temp_disp + " \r\n");
				s_txt48  += ("Код регулировки яркости дисплея не совпадает             \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 329, false);
			}

			if (coilArr_all[130] != false)
			{
				error_list_reg(1830, 1, 130, 130);
				temp_disp = readVolt_all[130];
                s_txt8   += ("Тест Radio1 ** сигнал  на выходе mag radio     \tON  -       \t< = " + readVals_all[130] + ">  " + temp_disp + " \r\n");
                s_txt48  += ("Тест Radio1 ** сигнал  на выходе mag radio     \tON  -       \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 330, false);
			}

			if (coilArr_all[131] != false)
			{
				error_list_reg(1834, 1, 131, 131);
				temp_disp = readVolt_all[131];
                s_txt8   += ("Тест Radio2 ** сигнал  на выходе mag radio     \tON  -       \t< = " + readVals_all[131] + ">  " + temp_disp + " \r\n");
                s_txt48  += ("Тест Radio2 ** сигнал  на выходе mag radio     \tON  -       \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 331, false);
			}
			if (coilArr_all[132] != false)
			{
				error_list_reg(1838, 1, 132, 132);
				temp_disp = readVolt_all[132];
                s_txt8   += ("Тест ГГС ** сигнал  на выходе mag radio        \tON  -       \t< = " + readVals_all[132] + ">  " + temp_disp + " \r\n");
                s_txt48  += ("Тест ГГС ** сигнал  на выходе mag radio        \tON  -       \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 332, false);
			}

            if (coilArr_all[133] != false)
            {
                error_list_reg(1842, 1, 133, 133);
                temp_disp = readVolt_all[133];
                s_txt8   += ("Тест гарнитуры инструктора сигнал на выходе mag radio \tON  -       \t< = " + readVals_all[133] + ">  " + temp_disp + " \r\n");
                s_txt48  += ("Тест гарнитуры инструктора сигнал на выходе mag radio \tON  -       \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 333, false);
            }

            if (coilArr_all[134] != false)
            {
                error_list_reg(1846, 1, 134, 134);
                temp_disp = readVolt_all[134];
                s_txt8   += ("Тест гарнитуры диспетчера сигнал на выходе mag radio \tON  -       \t< = " + readVals_all[134] + ">  " + temp_disp + " \r\n");
                s_txt48  += ("Тест гарнитуры диспетчера сигнал на выходе mag radio \tON  -       \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 334, false);
            }
            if (coilArr_all[135] != false)
            {
                error_list_reg(1850, 1, 135, 135);
                temp_disp = readVolt_all[135];
                s_txt8   += ("Тест МТТ сигнал на выходе mag radio                  \tON  -       \t< = " + readVals_all[135] + ">  " + temp_disp + " \r\n");
                s_txt48  += ("Тест МТТ сигнал на выходе mag radio                  \tON  -       \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 335, false);
            }
            if (coilArr_all[136] != false)
            {
                error_list_reg(1854, 1, 136, 136);
                temp_disp = readVolt_all[136];
                s_txt8   += ("Тест микрофона сигнал на выходе mag radio            \tON  -       \t< = " + readVals_all[136] + ">  " + temp_disp + " \r\n");
                s_txt48  += ("Тест микрофона сигнал на выходе mag radio            \tON  -       \t = " + temp_disp + " \r\n");
                res       = myProtocol.writeCoil(slave, 336, false);
            }
			res = myProtocol.writeCoil(slave, 120, false);                      // Снять флаг общей ошибки теста

		}
		private void error_list1(ushort adr_mem, ushort i_reg_max)                                          //  40200 Получить данные со  счетчиков ошибок  
		{
			ushort[] writeVals = new ushort[20];
			ushort[] readVolt  = new ushort[30];
			ushort[] readVals  = new ushort[30];
			ushort[] readTemp  = new ushort[30];
			ushort step_reg    = 10;
			int numWrRegs      = 3;
			ushort _adr_mem    = adr_mem;                                              // Старт 1309
			ushort _i_reg_max  = i_reg_max;                                          // Количество блоков памяти
			startWrReg         = 120;
			startRdReg         = 130;
			writeVals[0]       = 130;                                                      //  Адрес блока регистров для передачи в ПК уровней порогов.
			writeVals[1]       = 1309;                                                     //  Адрес блока памяти  для передачи в ПК уровней порогов.
			writeVals[2]       = 10;                                                       //  Длина блока регистров для передачи в ПК уровней порогов.
			richTextBox2.Text  = "";

			for ( int i_reg = 0; i_reg < i_reg_max; i_reg++)
			{
				writeVals[1] = _adr_mem;                                             //  Адрес блока памяти  для передачи в ПК уровней порогов.
				writeVals[2] = step_reg;                                             //  Длина блока регистров для передачи в ПК уровней порогов.
				startWrReg = 124;                                                    //  Отправить параметры блока получения данных из памяти 
				numWrRegs = 3;                                                       //
				res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				test_end1();
				startWrReg = 120;                                                    // адрес блока команд
				res = myProtocol.writeSingleRegister(slave, startWrReg, 15);         // Команда - Передать информацию в регистры
				test_end1();
				res = myProtocol.readMultipleRegisters(slave, writeVals[0], readTemp, step_reg);
				test_end1();

				richTextBox2.Text += (writeVals[1].ToString() + "\r\n");
				for (int i = 0; i < step_reg; i++)                // Чтение блока регистров 
				{
					richTextBox2.Text += (i + " - " + readTemp[i].ToString() + "\r\n");
				}

				_adr_mem += step_reg;
				_adr_mem += step_reg;
			}

			/*
			for (int i_reg = 0; i_reg < 13; i_reg++)
			{
				res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40200 Считать счетчики ошибок  
				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = "    MODBUS ON    ";
					MODBUS_SUCCESS = true;

					for (int i_temp = 0; i_temp < 5; i_temp++)
					{
						readVals_all[startRdReg + i_temp - 200] = readVals[i_temp];
					}
					startRdReg += 5;
				}

				res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40200 Считать счетчики ошибок  
				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = "    MODBUS ON    ";
					MODBUS_SUCCESS = true;

					for (int i_temp = 0; i_temp < 5; i_temp++)
					{
						readVals_all[startRdReg + i_temp - 200] = readVals[i_temp];
					}
					startRdReg += 5;
				}
			}

			*/
		}
	
		private void error_list_reg(ushort adr_mem, ushort i_reg_max, ushort adr_readVals_all, ushort adr_readVolt_all)                                          //  40200 Получить данные со  счетчиков ошибок  
		{
			ushort[] writeVals       = new ushort[20];
			ushort[] readVolt        = new ushort[30];
			ushort[] readVals        = new ushort[30];
			ushort[] readTemp        = new ushort[30];
			ushort step_reg          = 10;
			int numWrRegs            = 3;
			ushort _adr_mem          = adr_mem;                                      // Старт 1309
			ushort _i_reg_max        = i_reg_max;                                    // Количество блоков памяти
			ushort _adr_readVals_all = adr_readVals_all;
			ushort _adr_readVolt_all = adr_readVolt_all;
			
			startWrReg = 120;
			startRdReg = 130;
			//writeVals[1] = 1310;                                                     //  Адрес блока памяти  для передачи в ПК уровней порогов.
			//writeVals[2] = 2;                                                        //  Длина блока регистров для передачи в ПК уровней порогов.
   
			writeVals[0] = 130;                                                      //  Адрес блока регистров для передачи в ПК уровней порогов.
			writeVals[1] = _adr_mem;                                             //  Адрес блока памяти  для передачи в ПК уровней порогов.
			writeVals[2] = 2;                                             //  Длина блока регистров для передачи в ПК уровней порогов.
			startWrReg   = 124;                                                    //  Отправить параметры блока получения данных из памяти 
			numWrRegs    = 3;                                                       //
			res          = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
			test_end1();
			startWrReg = 120;                                                    // адрес блока команд
			res = myProtocol.writeSingleRegister(slave, startWrReg, 15);         // Команда - Передать информацию в регистры
			test_end1();
			res = myProtocol.readMultipleRegisters(slave, writeVals[0], readTemp, step_reg);
			test_end1();

			readVals_all[_adr_readVals_all] = readTemp[0];                 
			readVolt_all[_adr_readVolt_all] = readTemp[1];
			
			
			/*
			for (int i_reg = 0; i_reg < i_reg_max; i_reg++)
			{
				writeVals[1] = _adr_mem;                                             //  Адрес блока памяти  для передачи в ПК уровней порогов.
				writeVals[2] = step_reg;                                             //  Длина блока регистров для передачи в ПК уровней порогов.
				startWrReg = 124;                                                    //  Отправить параметры блока получения данных из памяти 
				numWrRegs = 3;                                                       //
				res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				test_end1();
				startWrReg = 120;                                                    // адрес блока команд
				res = myProtocol.writeSingleRegister(slave, startWrReg, 15);         // Команда - Передать информацию в регистры
				test_end1();
				res = myProtocol.readMultipleRegisters(slave, writeVals[0], readTemp, step_reg);
				test_end1();

				for (int i_temp = 0; i_temp < step_reg; i_temp++)
				{
					readVals_all[_adr_readVals_all + i_temp] = readTemp[i_temp];
					readVolt_all[_adr_readVolt_all + i_temp] = readTemp[i_temp + 1];
					i_temp++;
				}
				_adr_readVals_all += step_reg;                    // Смещение в начало следующего блока регистров 
				_adr_readVolt_all += step_reg;                    // Смещение в начало следующего блока регистров 

				_adr_mem += step_reg;                             // Смещение в начало следующего блока памяти
				_adr_mem += step_reg;                             // Смещение в начало следующего блока памяти
				Thread.Sleep(400);
			}
			*/
		}
		private void error_list3()                                          //  200   Получить данные состояния флага индикации возникновения  ошибки
		{
			bool[] coilArr = new bool[200];
			startCoil = 200;                                                                    // Начальный Адрес 200 флага индикации возникновения  ошибки
			numCoils = 5;

			for (int i_reg = 0; i_reg < 13; i_reg++)
			{
				res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = "    MODBUS ON    ";
					MODBUS_SUCCESS = true;


					for (int i_temp = 0; i_temp < 5; i_temp++)
					{
						coilArr_all[startCoil + i_temp - 200] = coilArr[i_temp];
					}
					startCoil += 5;
				}
				res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = "    MODBUS ON    ";
					MODBUS_SUCCESS = true;

					for (int i_temp = 0; i_temp < 5; i_temp++)
					{
						coilArr_all[startCoil + i_temp - 200] = coilArr[i_temp];
					}
					startCoil += 5;
				}
			}
		}
		private void error_list_print()                                     // Вывод информации о состоянии error регистров (информация отладочная, в измерениях не применяется)
		{
			richTextBox2.Text = "";
			for (int i_reg = 0; i_reg < 130; i_reg++)
			{
				richTextBox2.Text += ((i_reg + 200) + "   -  " + readVals_all[i_reg] + "  -  " + coilArr_all[i_reg] + "  -  " + readVolt_all[i_reg] + "\r\n");
			}
			richTextBox2.Refresh();
			richTextBox2.SelectionStart = richTextBox2.Text.Length;
			richTextBox2.ScrollToCaret();
		}
		#endregion 
		//*******************************************
	
		private void button11_Click_1(object sender, EventArgs e)                     // Старт полного теста
		{
			if ((myProtocol != null))
			{
				stop_test_modbus1();                                                                //Остановить контроль связи MODBUS   
				Thread.Sleep(400);
				ushort[] writeVals = new ushort[200];
				bool[] coilArr = new bool[200];
				Button_All_Test_Stop = true;                                                        // Установить признак выполнения теста
				progressBar2.Value = 0;                                                             // Установить  progressBar2 в 0
				MaxTestCount = int.Parse(comboBox4.Text, CultureInfo.CurrentCulture);               // Загрузить количество проходов теста
				slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);                       // Загрузить номер slave прибора 
				startCoil = 8;                                                                      // Управление питанием платы "Камертон"
				res = myProtocol.writeCoil(slave, startCoil, true);                                 // Включить питание платы "Камертон"
				if ((res == BusProtocolErrors.FTALK_SUCCESS))
				{
					toolStripStatusLabel1.Text = "    MODBUS ON    ";                               // Успешное подключение по протоколу MODBUS
					toolStripStatusLabel1.BackColor = Color.Lime;                                   // Успешное подключение по протоколу MODBUS
					Thread.Sleep(1700);                                                             // Ожидание включения модуля Аудио-1
					button11.BackColor = Color.White;                                               // Кнопка "Старт" 
					button11.Enabled = false;                                                       // Кнопка "Старт" отключена
					button11.Refresh();                                                             // Кнопка "Старт"
					textBox7.Text = ("Выполняется полный  контроль звукового модуля Аудио-1" + "\r\n");
					textBox7.Text += ("\r\n");
					textBox8.Text = ("");
					textBox9.Text = ("");
					textBox48.Text = ("");
					textBox7.Refresh();
					textBox8.Refresh();
					textBox9.Refresh();
					s_txt7 = "";
					s_txt8 = "";
					s_txt9 = "";
					s_txt48 = "";
					label54.Visible = false;                                                        // Текст "Ожидайте завершения выолнения команды!!" спрятан 
					startWrReg = 120;                                                               // 
					res = myProtocol.writeSingleRegister(slave, startWrReg, 24);                    // Команда на проверку наличия SD памяти
					if ((res == BusProtocolErrors.FTALK_SUCCESS))
					{
						test_end1();                                                                // Проверка наличия SD памяти окончена
						num_module_audio();                                                         // Преобразование номера модуля Аудио 1
						numCoils = 2;                                                               //    
						startCoil = 125;
						res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);            // Проверить Адрес 125  индикации возникновения ошибки SD память
						if ((res == BusProtocolErrors.FTALK_SUCCESS))
						{
							if (coilArr[0] != false)                                                //нет ошибки
							{
								textBox9.Text += ("SD память установлена " + "\r\n");
								textBox9.Refresh();
								//  0 в регистре означает завершение выполнения фрагмента проверки
								numRdRegs = 2;
								startCoil = 124;                                                    // regBank.add(124);  Флаг индикации связи с модулем "АУДИО"
								numCoils = 2;
								res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);    // 
								if ((res == BusProtocolErrors.FTALK_SUCCESS))
								{
									if (coilArr[0] == true)                                         //нет ошибки
									{
										textBox7.Text += ("Связь со звуковой платой АУДИО-1 установлена." + "\r\n");
										textBox7.Refresh();
										// Многократная проверка, записать номера выбранных тестов
										startCoil = 118;                                            // Признак многократной проверки установлен. Передать в контроллер
										res = myProtocol.writeCoil(slave, startCoil, true);         // Отправить  признак многократной проверки в Камертон 5.0   
										if ((res == BusProtocolErrors.FTALK_SUCCESS))
										{
											if (radioButton2.Checked)                               // Признак многократной роверки необходим для блокирования вывода результатов проверки
											{
												// Многократная проверка, записать номера выбранных тестов
												startCoil = 118;                                    // Признак многократной роверки установлен. Передать в контроллер
												res = myProtocol.writeCoil(slave, startCoil, true);

												TestStep = 0;                                       // Счетчик количества тестов

												if (checkBoxPower.Checked)                          //
												{
													test_step[TestStep] = 0;
													TestStep++;
												}

												if (checkBoxSensors1.Checked)
												{
													test_step[TestStep] = 1;
													TestStep++;
												}

												if (checkBoxSensors2.Checked)
												{
													test_step[TestStep] = 2;
													TestStep++;
												}

												if (checkBoxSenGar1instr.Checked)
												{
													test_step[TestStep] = 3;
													TestStep++;
												}

												if (checkBoxSenGar1disp.Checked)
												{
													test_step[TestStep] = 4;
													TestStep++;
												}

												if (checkBoxSenTrubka.Checked)
												{
													test_step[TestStep] = 5;
													TestStep++;
												}

												if (checkBoxSenTangRuch.Checked)
												{
													test_step[TestStep] = 6;
													TestStep++;
												}

												if (checkBoxSenTangN.Checked)
												{
													test_step[TestStep] = 7;
													TestStep++;
												}

												if (checkBoxSenGGS.Checked)
												{
													test_step[TestStep] = 8;
													TestStep++;
												}

												if (checkBoxSenGGRadio1.Checked)
												{
													test_step[TestStep] = 9;
													TestStep++;
												}

												if (checkBoxSenGGRadio2.Checked)
												{
													test_step[TestStep] = 10;
													TestStep++;
												}

												if (checkBoxSenMicrophon.Checked)
												{
													test_step[TestStep] = 11;
													TestStep++;
												}

												if (checkBoxDisp.Checked)
												{
													test_step[TestStep] = 12;
													TestStep++;
												}

											}
											else
											{
												// При однократной проверки, записать все номера тестов
												startCoil = 118;                                                              // Признак многократной роверки снят.      Передать в контроллер
												res = myProtocol.writeCoil(slave, startCoil, false);
												for (int i = 0; i < 13; i++)                                                  // Заполнить список тестов (Выполнить все тесты)
												{
													test_step[i] = i;
												}
												TestStep = 13;                                                                // Количество тестов
											}

											TestN = 0;                                                                        // Обнулить счетчик номера выполняемых тестов
											TestRepeatCount = 1;                                                              // Установить начальный номер  счетчика проходов теста
											startWrReg = 120;                                                                 // Команда на 
											res = myProtocol.writeSingleRegister(slave, startWrReg, 16);                      // Команда на сброс счетчиков отправлена
											test_end1();
											startWrReg = 120;                                                                 // Команда на открытие файла отправлена
											res = myProtocol.writeSingleRegister(slave, startWrReg, 12);                      // Команда на открытие файла отправлена
											test_end1();
											textBox9.Text += ("Команда на открытие файла отправлена" + "\r\n");
											textBox9.Refresh();
											Thread.Sleep(600);
											file_fakt_namber();                                                               // Отобразить имя текущего файла
											num_string();
											Create_File();
											textBox8.Text += ("Отчет тестирования модуля Аудио-1 N " + textBox46.Text + "\r\n");
											textBox8.Text += ("Дата " + DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n\r\n");
											textBox48.Text += ("Отчет тестирования модуля Аудио-1 N " + textBox46.Text + "\r\n");
											textBox48.Text += ("Дата " + DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n\r\n");
											textBox8.Refresh();
											textBox48.Refresh();
											test_end1();
											//  Запуск теста
											button9.BackColor = Color.Red;
											button9.Enabled = true;
											button9.Refresh();
											Thread.Sleep(300);
											start_DoWorkAll_Test();

										}
										else
										{
											textBox7.Text += ("Ошибка! Номера тестов не получены" + "\r\n");
											button9.BackColor = Color.LightSalmon;
											button11.BackColor = Color.Lime;
											label92.Text = ("");
											textBox7.Text += ("Тест остановлен" + "\r\n");
											progressBar2.Value = 0;
											Thread.Sleep(200);
											Button_All_Test_Stop = false;                                          // Признак окончания теста
											button9.Enabled = false;                                               // Отключить кнопку   "Стоп"
											Thread.Sleep(200);
											label54.Visible = false;
											groupBox19.Text = ("Ход выполнения теста ");                           // Очистить индикацию счетчика тестов
											button11.Enabled = true;
											All_Test_run = false;
											start_test_modbus1();                                                  // Запустить сканирование modbus
										}
									}
									else
									{
										textBox7.Text += ("Связь со звуковой платой АУДИО-1 НЕ УСТАНОВЛЕНА !(1)" + "\r\n" + "\r\n");
										button9.BackColor = Color.LightSalmon;
										button11.BackColor = Color.Lime;
										label92.Text = ("");
										textBox7.Text += ("Тест остановлен" + "\r\n");
										progressBar2.Value = 0;
										Thread.Sleep(200);
										Button_All_Test_Stop = false;                                               // Признак для управления кнопкой "Стоп"
										button9.Enabled = false;                                               // Отключить кнопку  
										Thread.Sleep(200);
										label54.Visible = false;
										groupBox19.Text = ("Ход выполнения теста ");
										button11.Enabled = true;
										All_Test_run = false;
										start_test_modbus1();
									}

								}
								else
								{
									toolStripStatusLabel1.Text = "    MODBUS ERROR (9) ";
									toolStripStatusLabel1.BackColor = Color.Red;
									toolStripStatusLabel4.Text = ("Ошибка!запрос на подключение модуля Аудио-1");  // Обработка ошибки.
									toolStripStatusLabel4.ForeColor = Color.Red;
									toolStripStatusLabel4.BackColor = Color.White;
									Thread.Sleep(200);
									Button_All_Test_Stop = false;                                               // Признак для управления кнопкой "Стоп"
									button9.Enabled = false;                                               // Отключить кнопку  
									Thread.Sleep(200);
									label54.Visible = false;
									groupBox19.Text = ("Ход выполнения теста ");
									button11.Enabled = true;
									All_Test_run = false;
									start_test_modbus1();
								}

							}
							else
							{
								textBox9.Text += ("Ошибка! SD память не установлена " + "\r\n");
								textBox9.Text += ("Проверка остановлена. Установите  SD память " + "\r\n");
								textBox9.Refresh();
								Thread.Sleep(200);
								Button_All_Test_Stop = false;                                               // Признак для управления кнопкой "Стоп"
								button9.Enabled = false;                                               // Отключить кнопку  
								Thread.Sleep(200);
								label54.Visible = false;
								groupBox19.Text = ("Ход выполнения теста ");
								button11.Enabled = true;
								All_Test_run = false;
								start_test_modbus1();
							}

						}

						else
						{
							toolStripStatusLabel1.Text = "    MODBUS ERROR (8) ";
							toolStripStatusLabel1.BackColor = Color.Red;
							toolStripStatusLabel4.Text = ("Ошибка ответа SD !");  // Обработка ошибки.
							toolStripStatusLabel4.ForeColor = Color.Red;
							toolStripStatusLabel4.BackColor = Color.White;
							Thread.Sleep(200);
							Button_All_Test_Stop = false;                                               // Признак для управления кнопкой "Стоп"
							button9.Enabled = false;                                               // Отключить кнопку  
							Thread.Sleep(200);
							label54.Visible = false;
							groupBox19.Text = ("Ход выполнения теста ");
							button11.Enabled = true;
							All_Test_run = false;
							start_test_modbus1();

						}

					}
					else
					{
						toolStripStatusLabel1.Text = "    MODBUS ERROR (7) ";
						toolStripStatusLabel1.BackColor = Color.Red;
						toolStripStatusLabel4.Text = ("Ошибка запроса наличия SD !");  // Обработка ошибки.
						toolStripStatusLabel4.ForeColor = Color.Red;
						toolStripStatusLabel4.BackColor = Color.White;
						Thread.Sleep(200);
						Button_All_Test_Stop = false;                                               // Признак для управления кнопкой "Стоп"
						button9.Enabled = false;                                               // Отключить кнопку  
						Thread.Sleep(200);
						label54.Visible = false;
						groupBox19.Text = ("Ход выполнения теста ");
						button11.Enabled = true;
						All_Test_run = false;
						start_test_modbus1();
					}
				}
				else
				{
					toolStripStatusLabel1.Text = "    MODBUS ERROR (6) ";
					toolStripStatusLabel1.BackColor = Color.Red;
					toolStripStatusLabel4.Text = ("Ошибка включения питания Камертон 5.0.1!");  // Обработка ошибки.
					toolStripStatusLabel4.ForeColor = Color.Red;
					toolStripStatusLabel4.BackColor = Color.White;
					Thread.Sleep(200);
					Button_All_Test_Stop = false;                                               // Признак для управления кнопкой "Стоп"
					button9.Enabled = false;                                               // Отключить кнопку  
					Thread.Sleep(200);
					label54.Visible = false;
					groupBox19.Text = ("Ход выполнения теста ");
					button11.Enabled = true;
					All_Test_run = false;
					start_test_modbus1();
				}

			}
			else
			{
				toolStripStatusLabel1.Text = "    MODBUS ERROR (5) ";
				toolStripStatusLabel1.BackColor = Color.Red;
				toolStripStatusLabel4.Text = ("Ошибка!Протокол не запущен Камертон5.0.1 !");  // Обработка ошибки.
				toolStripStatusLabel4.ForeColor = Color.Red;
				toolStripStatusLabel4.BackColor = Color.White;
				Thread.Sleep(2000);
			}

			progressBar2.Value = 0;
			label80.Text = DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture);
			toolStripStatusLabel2.Text = ("Время : " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture));
	 
		}
		private void button9_Click(object sender, EventArgs e)                        // Стоп полного теста
			{
			if(Button_All_Test_Stop == true)                                                        // Признак для управления кнопкой "Стоп"
				{
					label54.Visible = true;
					textBox7.Text += ("Тест остановлен" + "\r\n");
					Thread.Sleep(300);
					stop_DoWorkAll_Test();
					Thread.Sleep(300);
					button9.Enabled = false;                                                             // Отключить кнопку  
				}
			}

		private void num_string()                                                     // Получение из Камертон 5.0 и ввод номера имени файла 
		{
			short[] readVals = new short[125];
			int startRdReg;
			int numRdRegs;
			int res;
			string s0 = "";
			string s1 = "";
			string s2 = "";
			string s3 = "";
			
			startRdReg = 112; // 40112 
			numRdRegs = 5;
			res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
			lblResult.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));

			if ((res == BusProtocolErrors.FTALK_SUCCESS))
			{
				toolStripStatusLabel1.Text = "    MODBUS ON    ";
				toolStripStatusLabel1.BackColor = Color.Lime;

				s0 = (readVals[0].ToString());

				if (readVals[1] < 10)
					{
						s1 = ("0" + readVals[1].ToString());
					}
				else
					{
					    s1 = (readVals[1].ToString());
					}
				if (readVals[2] < 10)
					{
	        		    s2 = ("0" + readVals[2].ToString());
					}
				else
					{
						s2 = (readVals[2].ToString());
					}
				if (readVals[3] < 10)
					{
						s3 = ("0" + readVals[3].ToString());
					}
				else
					{
	     				s3 = (readVals[3].ToString());
					}

			}
		   fileName = (s0 + s1 + s2 + s3 + ".KAM");
           label150.Text = fileName;
		   openFileDialog1.FileName = fileName;
		}
		private void num_module_audio()                                               // Ввод номера платы Аудио 1 и передача его в Камертон 5
		{
		
			ushort[] writeVals = new ushort[20];
			int numWrRegs;   //
				startWrReg = 10;
				numWrRegs = 4;   //
  
				int.Parse(textBox46.Text);
				num_module_audio1 = Convert.ToInt32(textBox46.Text);
				byte[] data = BitConverter.GetBytes(num_module_audio1);
				Array.Reverse(data);
				writeVals[0] = (ushort)data[0];
				writeVals[1] = (ushort)data[1];
				writeVals[2] = (ushort)data[2];
				writeVals[3] = (ushort)data[3];
				res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
		 }

		private void Create_File()
		{
			pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
			System.IO.Directory.CreateDirectory(pathString);
			pathString = System.IO.Path.Combine(pathString, fileName);


			if (!System.IO.File.Exists(pathString))
			{
				File.Create(pathString).Close();
			}
			else
			{
				MessageBox.Show("Файл уже существует!  " + pathString, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				return;
			}
		}
		private void Save_File()
		{
			pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
			System.IO.Directory.CreateDirectory(pathString);
			pathString = System.IO.Path.Combine(pathString, fileName);
			File.WriteAllText(pathString, textBox48.Text, Encoding.GetEncoding("UTF-8"));
		}
		private void Read_File()
		{
			pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
			pathString = System.IO.Path.Combine(pathString, fileName);

			if (File.Exists(pathString))
			{
				textBox48.Text = File.ReadAllText(pathString);
			}
			else
			{
				MessageBox.Show("Файл НЕ существует!  " + pathString, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Information);
			}
		}

		private void textBox4_TextChanged(object sender, EventArgs e)
		{
			textBox4.SelectionStart = textBox4.Text.Length;
			textBox4.ScrollToCaret();
			textBox4.Refresh();
		}
		private void textBox7_TextChanged(object sender, EventArgs e)
		{
			textBox7.SelectionStart = textBox7.Text.Length;
			textBox7.ScrollToCaret();
			textBox7.Refresh();
		}
		private void textBox8_TextChanged(object sender, EventArgs e)
		{
			textBox8.SelectionStart = textBox8.Text.Length;
			textBox8.ScrollToCaret();
			textBox8.Refresh();
		}
		private void textBox9_TextChanged(object sender, EventArgs e)
		{
			textBox9.SelectionStart = textBox9.Text.Length;
			textBox9.ScrollToCaret();
			textBox9.Refresh();
		}
		private void textBox11_TextChanged(object sender, EventArgs e)
		{
			textBox11.SelectionStart = textBox11.Text.Length;
			textBox11.ScrollToCaret();
			textBox11.Refresh();
		}
		private void checkBox1_CheckedChanged(object sender, EventArgs e)
		{
			if (checkBoxSenAll.Checked == true)
			{
				// Включить проверку всех сенсоров
				checkBoxSensors1.Checked     = true;
				checkBoxSensors2.Checked     = true;
				checkBoxSenGar1instr.Checked = true;
				checkBoxSenGar1disp.Checked  = true;
				checkBoxSenTrubka.Checked    = true;
				checkBoxSenTangN.Checked     = true;
				checkBoxSenTangRuch.Checked  = true;
				checkBoxSenMicrophon.Checked = true;
				checkBoxSenGGRadio1.Checked  = true;
				checkBoxSenGGRadio2.Checked  = true;
				checkBoxSenGGS.Checked       = true;
				checkBoxDisp.Checked         = true;
				checkBoxPower.Checked        = true;
			}
			else
			{
				// Отключить проверку всех сенсоров
				checkBoxSenGGRadio1.Checked  = false;
				checkBoxSenGGRadio2.Checked  = false;
				checkBoxSenTrubka.Checked    = false;
				checkBoxSenTangN.Checked     = false;
				checkBoxSenTangRuch.Checked  = false;
				checkBoxSensors1.Checked     = false;
				checkBoxSensors2.Checked     = false;
				checkBoxSenGar1instr.Checked = false;
				checkBoxSenGar1disp.Checked  = false;
				checkBoxSenMicrophon.Checked = false;
				checkBoxSenGGS.Checked       = false;
				checkBoxDisp.Checked         = false;
				checkBoxPower.Checked        = false;
			}

		}
        private void textBox48_TextChanged(object sender, EventArgs e)
        {
            textBox48.SelectionStart = textBox48.Text.Length;
            textBox48.ScrollToCaret();
        }
		private void richTextBox2_TextChanged(object sender, EventArgs e) 
		{
			richTextBox2.SelectionStart = richTextBox2.Text.Length;
			richTextBox2.ScrollToCaret();
		}
 
		private void button24_Click(object sender, EventArgs e)                       // Установка уровня входного сигнала резисторами
		{
			button24.Enabled = false;
           // tempK_lev = short.Parse(textBox5.Text, CultureInfo.CurrentCulture) * 5;   // Установка уровня входного сигнала
         //   textBox4.BackColor = Color.White;
            tempK_lev = short.Parse(textBox4.Text, CultureInfo.CurrentCulture) * 5;   // Установка уровня звукового сигнала

            if (tempK_lev > 250)
            {
                label72.Text = "<";
                tempK_lev = 250;
                textBox4.Text = "50";
                textBox4.BackColor = Color.Red;
            }
            else
            {
                label72.Text = "=";
                textBox4.BackColor = Color.White;
            }

            set_sound_level = true;
		}
		private void button25_Click(object sender, EventArgs e)                       // Проверка яркости экрана
			{
			button25.Enabled = false;
            textBox5.BackColor = Color.White;
            tempK = short.Parse(textBox5.Text, CultureInfo.CurrentCulture);           // Установка уровня входного сигнала
            if (tempK > 127)
            {
                label39.Text = "<";
                tempK = 127;
                textBox5.Text = "127";
                textBox5.BackColor = Color.Red;
            }
            else
            {
                label39.Text = "=";
                textBox5.BackColor = Color.White;
            }
            set_display = true;

			}

		private void textBox46_KeyPress(object sender, KeyPressEventArgs e)
		{
			// Проверка на ввод только цифр
			if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
			 e.Handled = true;    
		}
		private void textBox5_KeyPress(object sender, KeyPressEventArgs e)
		{
			// Проверка на ввод только цифр
            if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
            {
                e.Handled = true;
                tempK = short.Parse(textBox5.Text, CultureInfo.CurrentCulture);           // Установка уровня входного сигнала
                if (tempK > 127)
                {
                    label39.Text = "<";
                    tempK = 127;
                    textBox5.Text = "127";
                    textBox5.BackColor = Color.Red;
                }
                else
                {
                    label39.Text = "=";
                    textBox5.BackColor = Color.White;
                }
            }
		}
		private void textBox4_KeyPress(object sender, KeyPressEventArgs e)
		{
			// Проверка на ввод только цифр
            if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
            {
                e.Handled = true;
            }
		}
		private void comboBox4_KeyPress(object sender, KeyPressEventArgs e)
		{
			if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				e.Handled = true;
		}

		private void txtTimeout_KeyPress(object sender, KeyPressEventArgs e)
		{
			// Проверка на ввод только цифр
			if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				e.Handled = true;  
		}
		private void txtPollDelay_KeyPress(object sender, KeyPressEventArgs e)
		{
			// Проверка на ввод только цифр
			if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				e.Handled = true;  
		}
		//private void txtTCPPort_KeyPress(object sender, KeyPressEventArgs e)
		//{
		//    // Проверка на ввод только цифр
		//    if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
		//        e.Handled = true;  
		//}
		private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
		 {
			 button13.Enabled = true;
			 list_files       = true;
		 }

		private void button5_Click(object sender, EventArgs e)                         // Получить список файлов       
		{
			comboBox1.Items.Clear();
			textBox48.Text = "";
			textBox48.Refresh();
			list_files = true;
			read_file = false;
			slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			startWrReg = 120;                                                 // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 28);
			test_end1();
			Thread.Sleep(200);
			startWrReg = 120;                                                 // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 26);
		 //   test_end1();
			Thread.Sleep(2000);
			button13.Enabled = true;
			textBox48.Refresh();
		}
		private void button13_Click(object sender, EventArgs e)                        // Чтение содержимого файла
		 {
			 textBox48.Text = "";
			 textBox48.Refresh();
			 progressBar3.Value = 1;
			 list_files = false;
			 read_file = true;

			 if (comboBox1.SelectedIndex != -1)                                                     // Отправить имя файла в Камертон 50  
			 {
				 startWrReg = 120;                                                                      // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 28);
				 test_end1();
				 ComPort2.Write(comboBox1.SelectedItem.ToString());
				 textBox48.Text = comboBox1.SelectedItem.ToString();
				 textBox48.Text += "\r\n";
				 textBox48.Refresh();
				 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
				 Thread.Sleep(200);
				 startWrReg = 120;                                                                   // Получить файл из Камертон 50  
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 25);
				 Thread.Sleep(3000);
				 button12.Enabled = true;
			   //  test_end1();
			 }
		 }
		private void button12_Click_1(object sender, EventArgs e)                      // Отправить текст в файл на ПК
		 {
				 button12.Enabled = false;
				 fileName = comboBox1.SelectedItem.ToString();
				 pathStringSD = System.IO.Path.Combine(folderName, "SD");
				 System.IO.Directory.CreateDirectory(pathStringSD);
				 pathStringSD = System.IO.Path.Combine(pathStringSD, fileName);
				 File.WriteAllText(pathStringSD, textBox48.Text, Encoding.GetEncoding("UTF-8"));
		 }

		private void button21_Click(object sender, EventArgs e)                        // Вызов программы редактирования
		 {
			 if (fileName == "")
			 {
				 System.Diagnostics.Process.Start(folderNotepadName);
			 }
			 else
			 {

			 if((File.Exists(folderNotepadName)))
				 {
				 System.Diagnostics.Process.Start((@"C:\Program Files\Notepad++\notepad++.exe"), pathStringSD);
				 }
			 else
				 {
				 folderFormatName = @"C:\Program Files (x86)\Notepad++\notepad++.exe";
				 if((File.Exists(folderFormatName)))
					 {
					 System.Diagnostics.Process.Start((@"C:\Program Files (x86)\Notepad++\notepad++.exe"), pathStringSD);
					 }
				 else
					 {
					 MessageBox.Show("                         Внимание!\r\n\r\n Программа редактирования не установлена", "Вызов программы редактирования", MessageBoxButtons.OK, MessageBoxIcon.Warning);
					 }

				 }

			 }
		 }
		private void button6_Click(object sender, EventArgs e)                         // Вызов программы форматирования SD карты
		 {
		   //  bool file_del = false;


			 if ((File.Exists(folderFormatName)))
			 {
				 MessageBox.Show("                         Внимание!\r\n\r\n Установите  SD карту в устройство чтения на ПК", "Вызов программы форматирования SD карты", MessageBoxButtons.OK, MessageBoxIcon.Warning);
				 System.Diagnostics.Process.Start(folderFormatName);
			 }
			 else
			 {
				 folderFormatName = @"C:\Program Files (x86)\SDA\SD Formatter\SDFormatter.exe";
				 if ((File.Exists(folderFormatName)))
				 {
					 MessageBox.Show("                         Внимание!\r\n\r\n Установите  SD карту в устройство чтения на ПК", "Вызов программы форматирования SD карты", MessageBoxButtons.OK, MessageBoxIcon.Warning);
					 System.Diagnostics.Process.Start(folderFormatName);
				 }
				 else
				 {
					 MessageBox.Show("                         Внимание!\r\n\r\n Программа форматирования не установлена", "Вызов программы форматирования SD карты", MessageBoxButtons.OK, MessageBoxIcon.Warning);
				 }

			 }

		 }

		private void file_del_SD_Click(object sender, EventArgs e)
		 {
			 if (comboBox1.Text != "")
			 {
				 file_del_yes.Visible = true;
				 file_del_no.Visible  = true;
				 file_del_SD.Enabled  = false;
				 file_del_no.Enabled  = true;
				 file_del_yes.Enabled = true;
				 fileName             = comboBox1.SelectedItem.ToString();
				 MessageBox.Show(("  Файл для удаления с SD карты и ПК\r\n\r\n                    ") + fileName, "Программа удаления файла с SD карты и ПК", MessageBoxButtons.OK, MessageBoxIcon.Warning);
			 }
			 else
			 {
				 MessageBox.Show("Файл для удаления с SD карты и ПК не указан", "Программа удаления файла с SD карты и ПК", MessageBoxButtons.OK, MessageBoxIcon.Error);
			 }

		 }
		private void file_del_no_Click(object sender, EventArgs e)
		 {
			 file_del_SD.Enabled  = true;
			 file_del_no.Enabled  = false;
			 file_del_yes.Enabled = false;
			 Thread.Sleep(200);
			 file_del_SD.Enabled  = true;
			 file_del_yes.Visible = false;
			 file_del_no.Visible  = false;
		 }
		private void file_del_yes_Click(object sender, EventArgs e)
		 {
			 file_del_SD.Enabled = false;                                              // Удалить  файл из SD и ПК
			 file_del_yes.Enabled = false;
			 file_del_no.Enabled = false;
			 if (comboBox1.Text != "")
			 {
				 fileName = comboBox1.SelectedItem.ToString();
				 pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
				 pathStringSD = System.IO.Path.Combine(folderName, "SD");
				 pathString = System.IO.Path.Combine(pathString, fileName);
				 pathStringSD = System.IO.Path.Combine(pathStringSD, fileName);


				 if (!(File.Exists(pathString)))
				 {
					 textBox48.Text = ("Внимание!  " + pathString + "   Файл на компьютере не найден");
				 }

				 else
				 {
					 File.Delete(pathString);
				 }

				 if (!(File.Exists(pathStringSD)))
				 {
					 textBox48.Text = ("Внимание!  " + pathStringSD + "   Файл на компьютере не найден");
				 }
				 else
				 {
					 File.Delete(pathStringSD);
				 }
				 if (!(ComPort2.IsOpen)) ComPort2.Open();
				 //Удаление файла на SD
				 ComPort2.Write(comboBox1.SelectedItem.ToString());
				 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
				 startWrReg = 120;                                                     // Удалить  файл из Камертон 50  
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 27);
				 test_end1();
				 button12.Enabled = true;
				 //  ComPort2.Close();
			 }
			 else
			 {
				 MessageBox.Show("Файл для удаления с SD карты и ПК не указан", "Программа удаления файла с SD карты и ПК", MessageBoxButtons.OK, MessageBoxIcon.Error);
			 }

			 file_del_SD.Enabled = true;
			 file_del_yes.Visible = false;
			 file_del_no.Visible = false;
		 }

 // Вызов дополнительных форм
#region setup_perametr

		 private void button60_Click_2(object sender, EventArgs e)        // Гарнитура инструктора
			 {
				 startCoil = 8;                                            // Управление питанием платы "Камертон"
				 res = myProtocol.writeCoil(slave, startCoil, true);       // Выключить питание платы "Камертон"
				 CloseSerialPort2();
				 Form2 frm2 = new Form2();                                //Создаем дочернюю форму
				 //Подключение обработчика события в дочерней форме
				 frm2.sendDataFromFormEvent2  += new EventHandler<UserEventArgs>(Form2m_sendDataFromFormEvent);  //Обработчик события получения данных из дочерней формы
				 frm2.sendDataFromFormEventS2 += new EventHandler<UserEventArgs>(Form2m_sendDataFromFormEventS);
				 frm2.ShowDialog();                                       //Выводим ее для заполнения текстовых полей
			 }
		 private void button91_Click(object sender, EventArgs e)          // Гарнитура диспетчера
			{
			 /*
				//Создаем дочернюю форму
				Form3 frm3 = new Form3();
				//Подключение обработчика события в дочерней форме
				 frm3.sendDataFromFormEvent3 += new EventHandler<UserEventArgs>(Form3m_sendDataFromFormEvent);
				 frm3.sendDataFromFormEventS3 += new EventHandler<UserEventArgs>(Form3m_sendDataFromFormEventS);
				//Выводим ее для заполнения текстовых полей
				frm3.ShowDialog();
			 */
			}
		 private void button86_Click(object sender, EventArgs e)          // Трубка МТТ
			 {
			 /*
				 //Создаем дочернюю форму
				  Form4 frm4 = new Form4();
				 //Подключение обработчика события в дочерней форме
				  frm4.sendDataFromFormEvent4 += new EventHandler<UserEventArgs>(Form4m_sendDataFromFormEvent);
				  frm4.sendDataFromFormEventS4 += new EventHandler<UserEventArgs>(Form4m_sendDataFromFormEventS);
				 //Выводим ее для заполнения текстовых полей
				//  CallBackMy4.callbackEventHandler("Проверка формы 4.", 10, 20, 30);
				  frm4.ShowDialog();
			 */
			 }
		 private void button87_Click(object sender, EventArgs e)          // Микрофон
			 {
			 /*
				 //Создаем дочернюю форму
				 Form5 frm5 = new Form5();
				 //Подключение обработчика события в дочерней форме
				 frm5.sendDataFromFormEvent5 += new EventHandler<UserEventArgs>(Form5m_sendDataFromFormEvent);
				 frm5.sendDataFromFormEventS5 += new EventHandler<UserEventArgs>(Form5m_sendDataFromFormEventS);
				 //Выводим ее для заполнения текстовых полей
				 frm5.ShowDialog();
			 */
			 }
		 private void button88_Click(object sender, EventArgs e)          // ГГ Радио 1
			 {
			 /*
				 //Создаем дочернюю форму
				 Form7 frm7 = new Form7();
				 //Подключение обработчика события в дочерней форме
				 frm7.sendDataFromFormEvent7 += new EventHandler<UserEventArgs>(Form7m_sendDataFromFormEvent);
				 frm7.sendDataFromFormEventS7 += new EventHandler<UserEventArgs>(Form7m_sendDataFromFormEventS);
				 //Выводим ее для заполнения текстовых полей
				 frm7.ShowDialog();
			 */
			 }
		 private void button76_Click_2(object sender, EventArgs e)        // ГГ Радио 2
			 {
			 /*
				 //Создаем дочернюю форму
				 Form8 frm8 = new Form8();
				 //Подключение обработчика события в дочерней форме
				 frm8.sendDataFromFormEvent8 += new EventHandler<UserEventArgs>(Form8m_sendDataFromFormEvent);
				 frm8.sendDataFromFormEventS8 += new EventHandler<UserEventArgs>(Form8m_sendDataFromFormEventS);
				 //Выводим ее для заполнения текстовых полей
				 frm8.ShowDialog();
			 */
			 }
		 private void button89_Click(object sender, EventArgs e)          // ГГС
			 {
			 /*
				 //Создаем дочернюю форму
				 Form6 frm6 = new Form6();
				 //Подключение обработчика события в дочерней форме
				 frm6.sendDataFromFormEvent6 += new EventHandler<UserEventArgs>(Form6m_sendDataFromFormEvent);
				 frm6.sendDataFromFormEventS6 += new EventHandler<UserEventArgs>(Form6m_sendDataFromFormEventS);
				 //Выводим ее для заполнения текстовых полей
				 frm6.ShowDialog();
			 */
			 }

#endregion


#region setup_table

	 
		 //Обработчик события получения данных из дочерней формы    sendDataFromFormEvent2
		 void Form2m_sendDataFromFormEvent(object sender, UserEventArgs e)
		 {
			 DataTable tableFromForm = e.SendingTable;                //Получаем таблицу из нашего именованного класса-аргумента события
			 dataGridViewParentForm2.DataSource = tableFromForm;      //Выводим таблицу для показа в грид
		 }
		 void Form2m_sendDataFromFormEventS(object sender, UserEventArgs e)
		 {
			 DataTable tableFromFormS = e.SendingTable;               //Получаем таблицу из нашего именованного класса-аргумента события
			 dataGridViewParentForm2S.DataSource = tableFromFormS;    //Выводим таблицу для показа в грид
		 }
  

		 void Form3m_sendDataFromFormEvent(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromForm = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm3.DataSource = tableFromForm;
		 }
		 void Form3m_sendDataFromFormEventS(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromFormS = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm3S.DataSource = tableFromFormS;
		 }
   
		 //Обработчик события получения данных из дочерней формы
		 void Form4m_sendDataFromFormEvent(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromForm = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm4.DataSource = tableFromForm;
		 }
		 void Form4m_sendDataFromFormEventS(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromFormS = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm4S.DataSource = tableFromFormS;
		 }
	
		//Обработчик события получения данных из дочерней формы
		 void Form5m_sendDataFromFormEvent(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromForm = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm5.DataSource = tableFromForm;
		 }
		 void Form5m_sendDataFromFormEventS(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromFormS = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm5S.DataSource = tableFromFormS;
		 }
	
		//Обработчик события получения данных из дочерней формы
		 void Form6m_sendDataFromFormEvent(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromForm = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm6.DataSource = tableFromForm;
		 }
		 void Form6m_sendDataFromFormEventS(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromFormS = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm6S.DataSource = tableFromFormS;
		 }
  
		//Обработчик события получения данных из дочерней формы
		 void Form7m_sendDataFromFormEvent(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromForm = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm7.DataSource = tableFromForm;
		 }
		 void Form7m_sendDataFromFormEventS(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromFormS = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm7S.DataSource = tableFromFormS;
		 }
   
		//Обработчик события получения данных из дочерней формы
		 void Form8m_sendDataFromFormEvent(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromForm = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm8.DataSource = tableFromForm;
		 }
		 void Form8m_sendDataFromFormEventS(object sender, UserEventArgs e)
		 {
			 //Получаем таблицу из нашего именованного класса-аргумента события
			 DataTable tableFromFormS = e.SendingTable;
			 //Выводим таблицу для показа в грид
			 dataGridViewParentForm8S.DataSource = tableFromFormS;
		 }
	 
		 private void button1_Click_1(object sender, EventArgs e)              // Загрузка  уровней порогов из Камертон 5.0
		 {

             read_porog_all();
            // // Уровни порогов сохранены в массиве table_porog[хх]
            // button1.Enabled = false;
            // button1.BackColor = Color.LightSalmon;
            // button1.Text = "Загрузка";
            // button1.Refresh();
            // get_porog_memory();                                               // Получить уровни порогов из Камертон 5.0
            // instruk_table1();                                                 // Отобразить таблицу порогов  инструктора
            // dispatch_table();                                                 // Отобразить таблицу порогов диспетчера
            // mtt_table();                                                      // Отобразить таблицу порогов MTT
            // mic_table();                                                      // Отобразить таблицу порогов микрофона
            // ggs_table();                                                      // Отобразить таблицу порогов GGS
            // radio1_table();                                                   // Отобразить таблицу порогов Radio1
            // radio2_table();                                                   // Отобразить таблицу порогов Radio2
            //// All_Table_Read = true;                                            // Таблица уровней загружена

		 }
         private void read_porog_all()
         {
             // Уровни порогов сохранены в массиве table_porog[хх]
             button1.Enabled = false;
             button1.BackColor = Color.LightSalmon;
             button1.Text = "Загрузка";
             button1.Refresh();
             get_porog_memory();                                               // Получить уровни порогов из Камертон 5.0
             instruk_table1();                                                 // Отобразить таблицу порогов  инструктора
             dispatch_table();                                                 // Отобразить таблицу порогов диспетчера
             mtt_table();                                                      // Отобразить таблицу порогов MTT
             mic_table();                                                      // Отобразить таблицу порогов микрофона
             ggs_table();                                                      // Отобразить таблицу порогов GGS
             radio1_table();                                                   // Отобразить таблицу порогов Radio1
             radio2_table();                                                   // Отобразить таблицу порогов Radio2
             All_Table_Read = true;                                            // Таблица уровней загружена полностью
         }

		 private void get_porog_memory()
		 {
			 ushort[] writeVals = new ushort[20];
			 ushort[] readVals = new ushort[125];
			 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			 int res;
			 int startWrReg;
			 int numWrRegs;

			 /* Процедура получения уровня порогов из Камертон 5.0
	
			  * Передача происходит через буфер обмена 40130 - 40ХХХ
			  */

			 // Подготовка параметров для считывания данных из памяти EEPROM
			 writeVals[0] = 130;                                                  //  Адрес блока регистров для передачи в ПК уровней порогов.
			 writeVals[1] = 1;                                                    //  Адрес блока памяти +200 для передачи в ПК уровней порогов.
			 writeVals[2] = 20;                                                   //  Длина блока регистров для передачи в ПК уровней порогов.
			 startRdReg = 130;                                                    //  40130 Адрес дата 
			 int step_mem = 0;                                                    //  Номер 

			 for (int i_mem = 0; i_mem < 14; i_mem++)
			 {
				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 127;                                                //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 3;                                                   //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 // Отправить команду получения данных из памяти
				 startWrReg = 120;                                                // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 22);     //  Отправить команду - Получить блок из Камертон 5.0
				 test_end1();

				 res = myProtocol.readMultipleRegisters(slave, writeVals[0], readVals, writeVals[2]); // Получить  блок с уровнями регистров

				 if ((res == BusProtocolErrors.FTALK_SUCCESS))
				 {
					 for (int i_reg = 0; i_reg < 20; i_reg++)            // Чтение блока регистров 
					 {
						 table_porog[(20 * step_mem) + i_reg] = readVals[i_reg]; //
					 }
					 step_mem++;           // 
				 }
				 writeVals[1] += 40;                                      //  Адрес блока памяти для передачи в ПК уровней порогов.
			 }
	
			 button1.Enabled = true;
			 button1.Text = "Обновить";
			 button1.BackColor = Color.LightGreen;
			 button1.Refresh();

		 }
		 private void set_porog_memory(ushort adr_reg, ushort adr_mem, ushort blok)
		 {
			 ushort[] writeVals = new ushort[200];
			 ushort[] readVals = new ushort[125];
			 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			 int res;
			 int startWrReg;
			 int numWrRegs;

			 /* Процедура получения уровня порогов из Камертон 5.0
			  * Данные об уровнях порогов находятся в энергонезависимой памяти EEPROM по адресам 
			  * //Новые пороги две ячейки, 2 байта
				 const  int adr_int_porog_instruktor            = 200;      // 19 адресов 
				 const  int adr_int_porog_dispatcher            = 250;      // 19 адресов 
				 const  int adr_int_porog_MTT                   = 300;      // 21 адресов 
				 const  int adr_int_porog_GGS                   = 350;      // 29 адресов 
				 const  int adr_int_porog_Radio1                = 420;      // 20 адресов 
				 const  int adr_int_porog_Radio2                = 460;      // 20 адресов 
				 const  int adr_int_porog_Microphone            = 500;      // xx адресов 
			  * 
			  * Передача происходит через буфер обмена 40130 - 40ХХХ
			  */

			 // Подготовка параметров для считывания данных из памяти EEPROM
			 writeVals[0] = adr_reg;                                              // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
			 writeVals[1] = adr_mem;                                              // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
			 writeVals[2] = blok;                                                 // 20  - Длина блока регистров для передачи в ПК уровней порогов.
			 startRdReg = 130;                                                    //  40130 Адрес дата 
			 int step_mem = 0;  

			 // Отправить параметры блока получения данных из памяти 
			 startWrReg = 127;                                                //  Отправить параметры блока получения данных из памяти 
			 numWrRegs = 3;                                                   //
			 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
			 // Отправить команду получения данных из памяти
		   //  res = myProtocol.writeCoil(slave, startCoil, false);             // 

			 for (int i_mem = 0; i_mem < 9; i_mem++)
			 {
				 writeVals[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[0].Value;
				 step_mem++;
				 writeVals[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[1].Value;
				 step_mem++;
			 }

              
			 //for (int i_mem = 0; i_mem < 9; i_mem++)
			 //{
			 //    writeVals[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[0].Value;
			 //    step_mem++;
			 //    writeVals[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[1].Value;
			 //    step_mem++;
			 //}

			 // Отправить параметры блока получения данных из памяти 
			 startWrReg = 130;                                                //  Отправить параметры блока получения данных из памяти 
			 numWrRegs = 18;                                                   //
			 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
			 test_end1();
			 startWrReg = 120;                                                // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);     //  Записать блок в EEPROM Камертон 5.0
			 test_end1();

		 }

		// Таблицы порогов
  
		 private void instruk_table1()
		 {
			 DataTable dt_instruk_table1 = new DataTable("InstruktorDataTable");                                                // Создаем таблицу
			 DataColumn noise_min        = new DataColumn("мин. Минимальный уровень порога шумов без сигнала", typeof(ushort)); // Определяем, создаем столбцы
			 DataColumn noise_min1       = new DataColumn("минC. Минимальный уровень порога шумов с сигналом", typeof(ushort)); // Определяем, создаем столбцы
 
			 dt_instruk_table1.Columns.AddRange(new DataColumn[]                                                                // Создаем столбцы 
			{noise_min, noise_min1});

			 //Формируем строку из данных в текстовых полях
			 DataRow row      = dt_instruk_table1.NewRow();                                 // Формируем строку из данных в текстовых полях
			 row[noise_min]   = table_porog[0];                                             // Записываем данные в первую строку, певого столбца таблицы
			 row[noise_min1]  = table_porog[1];                                             // Записываем данные в первую строку, второго столбца таблицы
			 dt_instruk_table1.Rows.Add(row);
			 DataRow row1     = dt_instruk_table1.NewRow();
			 row1[noise_min]  = table_porog[2];
			 row1[noise_min1] = table_porog[3];
			 dt_instruk_table1.Rows.Add(row1);
			 DataRow row2     = dt_instruk_table1.NewRow();
			 row2[noise_min]  = table_porog[4];
			 row2[noise_min1] = table_porog[5];
			 dt_instruk_table1.Rows.Add(row2);
			 DataRow row3     = dt_instruk_table1.NewRow();
			 row3[noise_min]  = table_porog[6];
			 row3[noise_min1] = table_porog[7];
			 dt_instruk_table1.Rows.Add(row3);
			 DataRow row4     = dt_instruk_table1.NewRow();
			 row4[noise_min]  = table_porog[8];
			 row4[noise_min1] = table_porog[9];
			 dt_instruk_table1.Rows.Add(row4);
			 DataRow row5     = dt_instruk_table1.NewRow();
			 row5[noise_min]  = table_porog[10];
			 row5[noise_min1] = table_porog[11];
			 dt_instruk_table1.Rows.Add(row5);
			 DataRow row6     = dt_instruk_table1.NewRow();
			 row6[noise_min]  = table_porog[12];
			 row6[noise_min1] = table_porog[13];
			 dt_instruk_table1.Rows.Add(row6);
			 DataRow row7     = dt_instruk_table1.NewRow();
			 row7[noise_min]  = table_porog[14];
			 row7[noise_min1] = table_porog[15];
			 dt_instruk_table1.Rows.Add(row7);
			 DataRow row8     = dt_instruk_table1.NewRow();
			 row8[noise_min]  = table_porog[16];
			 row8[noise_min1] = table_porog[17];
			 dt_instruk_table1.Rows.Add(row8);
             DataRow row9 = dt_instruk_table1.NewRow();
             row9[noise_min] = table_porog[18];
             row9[noise_min1] = table_porog[19];
             dt_instruk_table1.Rows.Add(row9);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу

			 dataGridViewParentForm2.DataSource = dt_instruk_table1;

			 //=dataGridViewParentForm2.Rows[0].Cells[1].Value.ToString();

		  //   if (sendDataFromFormEvent_instr != null) sendDataFromFormEvent_instr(this, new UserEventArgs(dt_instruk_table1)); // Отправить данные в основную форму

			 //Создаем таблицу
			 DataTable dt_instruk_table1S = new DataTable("InstruktorDataTable");
			 DataColumn noise_minS        = new DataColumn("мин. Минимальный уровень порога сигнала", typeof(ushort));     //Создаем столбцы
			 DataColumn noise_maxS        = new DataColumn("макс. Максимальный уровень порога сигнала", typeof(ushort));   //Создаем столбцы

			 dt_instruk_table1S.Columns.AddRange(new DataColumn[] {
			 noise_minS,
			 noise_maxS});

			 //Формируем строку из данных в текстовых полях
			 DataRow rowS      = dt_instruk_table1S.NewRow();
			 rowS[noise_minS]  = table_porog[20];
			 rowS[noise_maxS]  = table_porog[21];
			 dt_instruk_table1S.Rows.Add(rowS);
			 DataRow row1S     = dt_instruk_table1S.NewRow();
			 row1S[noise_minS] = table_porog[22];
			 row1S[noise_maxS] = table_porog[23];
			 dt_instruk_table1S.Rows.Add(row1S);
			 DataRow row2S     = dt_instruk_table1S.NewRow();
			 row2S[noise_minS] = table_porog[24];
			 row2S[noise_maxS] = table_porog[25];
			 dt_instruk_table1S.Rows.Add(row2S);
			 DataRow row3S     = dt_instruk_table1S.NewRow();
			 row3S[noise_minS] = table_porog[26];
			 row3S[noise_maxS] = table_porog[27];
			 dt_instruk_table1S.Rows.Add(row3S);
			 DataRow row4S     = dt_instruk_table1S.NewRow();
			 row4S[noise_minS] = table_porog[28];
			 row4S[noise_maxS] = table_porog[29];
			 dt_instruk_table1S.Rows.Add(row4S);
			 DataRow row5S     = dt_instruk_table1S.NewRow();
			 row5S[noise_minS] = table_porog[30];
			 row5S[noise_maxS] = table_porog[31];
			 dt_instruk_table1S.Rows.Add(row5S);
			 DataRow row6S     = dt_instruk_table1S.NewRow();
			 row6S[noise_minS] = table_porog[32];
			 row6S[noise_maxS] = table_porog[33];
			 dt_instruk_table1S.Rows.Add(row6S);
			 DataRow row7S     = dt_instruk_table1S.NewRow();
			 row7S[noise_minS] = table_porog[34];
			 row7S[noise_maxS] = table_porog[35];
			 dt_instruk_table1S.Rows.Add(row7S);
			 DataRow row8S     = dt_instruk_table1S.NewRow();
			 row8S[noise_minS] = table_porog[36];
			 row8S[noise_maxS] = table_porog[37];
			 dt_instruk_table1S.Rows.Add(row8S);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу
			 dataGridViewParentForm2S.DataSource = dt_instruk_table1S;


			 if(checkBox1.Checked)
				 {
					     // Запрет ввода данных в таблицу
				    if (table_porog[20] == 0) dataGridViewParentForm2S.Rows[0].Cells[0].ReadOnly = true;
                    if (table_porog[21] == 0) dataGridViewParentForm2S.Rows[0].Cells[1].ReadOnly = true;
                    if (table_porog[22] == 0) dataGridViewParentForm2S.Rows[1].Cells[0].ReadOnly = true;
				    if (table_porog[23] == 0) dataGridViewParentForm2S.Rows[1].Cells[1].ReadOnly = true;
                    if (table_porog[24] == 0) dataGridViewParentForm2S.Rows[2].Cells[0].ReadOnly = true;
                    if (table_porog[25] == 0) dataGridViewParentForm2S.Rows[2].Cells[1].ReadOnly = true;
                    if (table_porog[26] == 0) dataGridViewParentForm2S.Rows[3].Cells[0].ReadOnly = true;
                    if (table_porog[27] == 0) dataGridViewParentForm2S.Rows[3].Cells[1].ReadOnly = true;
                    if (table_porog[28] == 0) dataGridViewParentForm2S.Rows[4].Cells[0].ReadOnly = true;
                    if (table_porog[29] == 0) dataGridViewParentForm2S.Rows[4].Cells[1].ReadOnly = true;
                    if (table_porog[30] == 0) dataGridViewParentForm2S.Rows[5].Cells[0].ReadOnly = true;
                    if (table_porog[31] == 0) dataGridViewParentForm2S.Rows[5].Cells[1].ReadOnly = true;
                    if (table_porog[32] == 0) dataGridViewParentForm2S.Rows[6].Cells[0].ReadOnly = true;
                    if (table_porog[33] == 0) dataGridViewParentForm2S.Rows[6].Cells[1].ReadOnly = true;
                    if (table_porog[34] == 0) dataGridViewParentForm2S.Rows[7].Cells[0].ReadOnly = true;
                    if (table_porog[35] == 0) dataGridViewParentForm2S.Rows[7].Cells[1].ReadOnly = true;
                    if (table_porog[36] == 0) dataGridViewParentForm2S.Rows[8].Cells[0].ReadOnly = true;
                    if (table_porog[37] == 0) dataGridViewParentForm2S.Rows[8].Cells[1].ReadOnly = true;

				     // Отметить ячейки с запретом ввода  красным цветом
                    if (table_porog[20] == 0) dataGridViewParentForm2S.Rows[0].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[21] == 0) dataGridViewParentForm2S.Rows[0].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[22] == 0) dataGridViewParentForm2S.Rows[1].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[23] == 0) dataGridViewParentForm2S.Rows[1].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[24] == 0) dataGridViewParentForm2S.Rows[2].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[25] == 0) dataGridViewParentForm2S.Rows[2].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[26] == 0) dataGridViewParentForm2S.Rows[3].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[27] == 0) dataGridViewParentForm2S.Rows[3].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[28] == 0) dataGridViewParentForm2S.Rows[4].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[29] == 0) dataGridViewParentForm2S.Rows[4].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[30] == 0) dataGridViewParentForm2S.Rows[5].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[31] == 0) dataGridViewParentForm2S.Rows[5].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[32] == 0) dataGridViewParentForm2S.Rows[6].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[33] == 0) dataGridViewParentForm2S.Rows[6].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[34] == 0) dataGridViewParentForm2S.Rows[7].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[35] == 0) dataGridViewParentForm2S.Rows[7].Cells[1].Style.BackColor = Color.Red;
                    if (table_porog[36] == 0) dataGridViewParentForm2S.Rows[8].Cells[0].Style.BackColor = Color.Red;
                    if (table_porog[37] == 0) dataGridViewParentForm2S.Rows[8].Cells[1].Style.BackColor = Color.Red;
				}

			 Insrt_Table_Read = true;
		 }
		 private void dispatch_table()
		 {
			 DataTable dt_dispatch_table = new DataTable("DispatchDataTable");                                                // Создаем таблицу
			 DataColumn noise_min = new DataColumn("мин. Минимальный уровень порога шумов без сигнала", typeof(ushort)); // Определяем, создаем столбцы
			 DataColumn noise_min1 = new DataColumn("минC. Минимальный уровень порога шумов с сигналом", typeof(ushort)); // Определяем, создаем столбцы

			 dt_dispatch_table.Columns.AddRange(new DataColumn[]                                                                // Создаем столбцы 
			{ noise_min, noise_min1 });

			 //Формируем строку из данных в текстовых полях
			 DataRow row     = dt_dispatch_table.NewRow();                                  // Формируем строку из данных в текстовых полях
			 row[noise_min]  = table_porog[38];                                             // Записываем данные в первую строку, певого столбца таблицы
			 row[noise_min1] = table_porog[39];                                             // Записываем данные в первую строку, второго столбца таблицы
			 dt_dispatch_table.Rows.Add(row);
			 DataRow row1     = dt_dispatch_table.NewRow();
			 row1[noise_min]  = table_porog[40];
			 row1[noise_min1] = table_porog[41];
			 dt_dispatch_table.Rows.Add(row1);
			 DataRow row2     = dt_dispatch_table.NewRow();
			 row2[noise_min]  = table_porog[42];
			 row2[noise_min1] = table_porog[43];
			 dt_dispatch_table.Rows.Add(row2);
			 DataRow row3     = dt_dispatch_table.NewRow();
			 row3[noise_min]  = table_porog[44];
			 row3[noise_min1] = table_porog[45];
			 dt_dispatch_table.Rows.Add(row3);
			 DataRow row4     = dt_dispatch_table.NewRow();
			 row4[noise_min]  = table_porog[46];
			 row4[noise_min1] = table_porog[47];
			 dt_dispatch_table.Rows.Add(row4);
			 DataRow row5     = dt_dispatch_table.NewRow();
			 row5[noise_min]  = table_porog[48];
			 row5[noise_min1] = table_porog[49];
			 dt_dispatch_table.Rows.Add(row5);
			 DataRow row6     = dt_dispatch_table.NewRow();
			 row6[noise_min]  = table_porog[50];
			 row6[noise_min1] = table_porog[51];
			 dt_dispatch_table.Rows.Add(row6);
			 DataRow row7     = dt_dispatch_table.NewRow();
			 row7[noise_min]  = table_porog[52];
			 row7[noise_min1] = table_porog[53];
			 dt_dispatch_table.Rows.Add(row7);
			 DataRow row8     = dt_dispatch_table.NewRow();
			 row8[noise_min]  = table_porog[54];
			 row8[noise_min1] = table_porog[55];
			 dt_dispatch_table.Rows.Add(row8);
             DataRow row9 = dt_dispatch_table.NewRow();
             row9[noise_min] = table_porog[56];
             row9[noise_min1] = table_porog[57];
             dt_dispatch_table.Rows.Add(row9);
			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу

			 dataGridViewParentForm3.DataSource = dt_dispatch_table;

			 //=dataGridViewParentForm2.Rows[0].Cells[1].Value.ToString();

			 //   if (sendDataFromFormEvent_instr != null) sendDataFromFormEvent_instr(this, new UserEventArgs(dt_instruk_table1)); // Отправить данные в основную форму

			 //Создаем таблицу
			 DataTable dt_dispatch_tableS = new DataTable("DispatchDataTable");
			 DataColumn noise_minS        = new DataColumn("мин. Минимальный уровень порога сигнала", typeof(ushort));     //Создаем столбцы
			 DataColumn noise_maxS        = new DataColumn("макс. Максимальный уровень порога сигнала", typeof(ushort));   //Создаем столбцы

			 dt_dispatch_tableS.Columns.AddRange(new DataColumn[] {
			 noise_minS,
			 noise_maxS});

			 //Формируем строку из данных в текстовых полях
			 DataRow rowS     = dt_dispatch_tableS.NewRow();
			 rowS[noise_minS] = table_porog[58];
			 rowS[noise_maxS] = table_porog[59];
			 dt_dispatch_tableS.Rows.Add(rowS);
			 DataRow row1S     = dt_dispatch_tableS.NewRow();
			 row1S[noise_minS] = table_porog[60];
			 row1S[noise_maxS] = table_porog[61];
			 dt_dispatch_tableS.Rows.Add(row1S);
			 DataRow row2S     = dt_dispatch_tableS.NewRow();
			 row2S[noise_minS] = table_porog[62];
			 row2S[noise_maxS] = table_porog[63];
			 dt_dispatch_tableS.Rows.Add(row2S);
			 DataRow row3S     = dt_dispatch_tableS.NewRow();
			 row3S[noise_minS] = table_porog[64];
			 row3S[noise_maxS] = table_porog[65];
			 dt_dispatch_tableS.Rows.Add(row3S);
			 DataRow row4S     = dt_dispatch_tableS.NewRow();
			 row4S[noise_minS] = table_porog[66];
			 row4S[noise_maxS] = table_porog[67];
			 dt_dispatch_tableS.Rows.Add(row4S);
			 DataRow row5S     = dt_dispatch_tableS.NewRow();
			 row5S[noise_minS] = table_porog[68];
			 row5S[noise_maxS] = table_porog[69];
			 dt_dispatch_tableS.Rows.Add(row5S);
			 DataRow row6S     = dt_dispatch_tableS.NewRow();
			 row6S[noise_minS] = table_porog[70];
			 row6S[noise_maxS] = table_porog[71];
			 dt_dispatch_tableS.Rows.Add(row6S);
			 DataRow row7S     = dt_dispatch_tableS.NewRow();
			 row7S[noise_minS] = table_porog[72];
			 row7S[noise_maxS] = table_porog[73];
			 dt_dispatch_tableS.Rows.Add(row7S);
			 DataRow row8S     = dt_dispatch_tableS.NewRow();
			 row8S[noise_minS] = table_porog[74];
			 row8S[noise_maxS] = table_porog[75];
			 dt_dispatch_tableS.Rows.Add(row8S);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу
			 dataGridViewParentForm3S.DataSource = dt_dispatch_tableS;

			 if(checkBox1.Checked)
				 {
                     if (table_porog[58] == 0) dataGridViewParentForm3S.Rows[0].Cells[0].ReadOnly = true;
                     if (table_porog[59] == 0) dataGridViewParentForm3S.Rows[0].Cells[1].ReadOnly = true;
                     if (table_porog[60] == 0) dataGridViewParentForm3S.Rows[1].Cells[0].ReadOnly = true;
                     if (table_porog[61] == 0) dataGridViewParentForm3S.Rows[1].Cells[1].ReadOnly = true;
                     if (table_porog[62] == 0) dataGridViewParentForm3S.Rows[2].Cells[0].ReadOnly = true;
                     if (table_porog[63] == 0) dataGridViewParentForm3S.Rows[2].Cells[1].ReadOnly = true;
                     if (table_porog[64] == 0) dataGridViewParentForm3S.Rows[3].Cells[0].ReadOnly = true;
                     if (table_porog[65] == 0) dataGridViewParentForm3S.Rows[3].Cells[1].ReadOnly = true;
                     if (table_porog[66] == 0) dataGridViewParentForm3S.Rows[4].Cells[0].ReadOnly = true;
                     if (table_porog[67] == 0) dataGridViewParentForm3S.Rows[4].Cells[1].ReadOnly = true;
                     if (table_porog[68] == 0) dataGridViewParentForm3S.Rows[5].Cells[0].ReadOnly = true;
                     if (table_porog[69] == 0) dataGridViewParentForm3S.Rows[5].Cells[1].ReadOnly = true;
                     if (table_porog[70] == 0) dataGridViewParentForm3S.Rows[6].Cells[0].ReadOnly = true;
                     if (table_porog[71] == 0) dataGridViewParentForm3S.Rows[6].Cells[1].ReadOnly = true;
                     if (table_porog[72] == 0) dataGridViewParentForm3S.Rows[7].Cells[0].ReadOnly = true;
                     if (table_porog[73] == 0) dataGridViewParentForm3S.Rows[7].Cells[1].ReadOnly = true;
                     if (table_porog[74] == 0) dataGridViewParentForm3S.Rows[8].Cells[0].ReadOnly = true;
                     if (table_porog[75] == 0) dataGridViewParentForm3S.Rows[8].Cells[1].ReadOnly = true;


                     if (table_porog[58] == 0) dataGridViewParentForm3S.Rows[0].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[59] == 0) dataGridViewParentForm3S.Rows[0].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[60] == 0) dataGridViewParentForm3S.Rows[1].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[61] == 0) dataGridViewParentForm3S.Rows[1].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[62] == 0) dataGridViewParentForm3S.Rows[2].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[63] == 0) dataGridViewParentForm3S.Rows[2].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[64] == 0) dataGridViewParentForm3S.Rows[3].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[65] == 0) dataGridViewParentForm3S.Rows[3].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[66] == 0) dataGridViewParentForm3S.Rows[4].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[67] == 0) dataGridViewParentForm3S.Rows[4].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[68] == 0) dataGridViewParentForm3S.Rows[5].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[69] == 0) dataGridViewParentForm3S.Rows[5].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[70] == 0) dataGridViewParentForm3S.Rows[6].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[71] == 0) dataGridViewParentForm3S.Rows[6].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[72] == 0) dataGridViewParentForm3S.Rows[7].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[73] == 0) dataGridViewParentForm3S.Rows[7].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[74] == 0) dataGridViewParentForm3S.Rows[8].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[75] == 0) dataGridViewParentForm3S.Rows[8].Cells[1].Style.BackColor = Color.Red;
				 }
			 Disp_Table_Read = true;
		 }
		 private void mtt_table()
		 {
			 DataTable dt_mtt_table = new DataTable("DispatchDataTable");                                                // Создаем таблицу
			 DataColumn noise_min   = new DataColumn("мин. Минимальный уровень порога шумов без сигнала", typeof(ushort)); // Определяем, создаем столбцы
			 DataColumn noise_min1  = new DataColumn("минC. Минимальный уровень порога шумов с сигналом", typeof(ushort)); // Определяем, создаем столбцы

			 dt_mtt_table.Columns.AddRange(new DataColumn[]                                                                // Создаем столбцы 
			{ noise_min, noise_min1 });

			 //Формируем строку из данных в текстовых полях
			 DataRow row     = dt_mtt_table.NewRow();                                      // Формируем строку из данных в текстовых полях
			 row[noise_min]  = table_porog[76];                                            // Записываем данные в первую строку, певого столбца таблицы
			 row[noise_min1] = table_porog[77];                                              // Записываем данные в первую строку, второго столбца таблицы
			 dt_mtt_table.Rows.Add(row);
			 DataRow row1     = dt_mtt_table.NewRow();
			 row1[noise_min]  = table_porog[78];
			 row1[noise_min1] = table_porog[79];
			 dt_mtt_table.Rows.Add(row1);
			 DataRow row2     = dt_mtt_table.NewRow();
			 row2[noise_min]  = table_porog[80];
			 row2[noise_min1] = table_porog[81];
			 dt_mtt_table.Rows.Add(row2);
			 DataRow row3     = dt_mtt_table.NewRow();
			 row3[noise_min]  = table_porog[82];
			 row3[noise_min1] = table_porog[83];
			 dt_mtt_table.Rows.Add(row3);
			 DataRow row4     = dt_mtt_table.NewRow();
			 row4[noise_min]  = table_porog[84];
			 row4[noise_min1] = table_porog[85];
			 dt_mtt_table.Rows.Add(row4);
			 DataRow row5     = dt_mtt_table.NewRow();
			 row5[noise_min]  = table_porog[86];
			 row5[noise_min1] = table_porog[87];
			 dt_mtt_table.Rows.Add(row5);
			 DataRow row6     = dt_mtt_table.NewRow();
			 row6[noise_min]  = table_porog[88];
			 row6[noise_min1] = table_porog[89];
			 dt_mtt_table.Rows.Add(row6);
			 DataRow row7     = dt_mtt_table.NewRow();
			 row7[noise_min]  = table_porog[90];
			 row7[noise_min1] = table_porog[91];
			 dt_mtt_table.Rows.Add(row7);
			 DataRow row8     = dt_mtt_table.NewRow();
			 row8[noise_min]  = table_porog[92];
			 row8[noise_min1] = table_porog[93];
			 dt_mtt_table.Rows.Add(row8);
             DataRow row9 = dt_mtt_table.NewRow();
             row9[noise_min] = table_porog[94];
             row9[noise_min1] = table_porog[95];
             dt_mtt_table.Rows.Add(row9);
			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу

			 dataGridViewParentForm4.DataSource = dt_mtt_table;

			 //Создаем таблицу
			 DataTable dt_mtt_tableS = new DataTable("MTTDataTable");
			 DataColumn noise_minS   = new DataColumn("мин. Минимальный уровень порога сигнала", typeof(ushort));     //Создаем столбцы
			 DataColumn noise_maxS   = new DataColumn("макс. Максимальный уровень порога сигнала", typeof(ushort));   //Создаем столбцы

			 dt_mtt_tableS.Columns.AddRange(new DataColumn[] {
			 noise_minS,
			 noise_maxS});

			 //Формируем строку из данных в текстовых полях
			 DataRow rowS      = dt_mtt_tableS.NewRow();
			 rowS[noise_minS]  = table_porog[96];
			 rowS[noise_maxS]  = table_porog[97];
			 dt_mtt_tableS.Rows.Add(rowS);
			 DataRow row1S     = dt_mtt_tableS.NewRow();
			 row1S[noise_minS] = table_porog[98];
			 row1S[noise_maxS] = table_porog[99];
			 dt_mtt_tableS.Rows.Add(row1S);
			 DataRow row2S     = dt_mtt_tableS.NewRow();
			 row2S[noise_minS] = table_porog[100];
			 row2S[noise_maxS] = table_porog[101];
			 dt_mtt_tableS.Rows.Add(row2S);
			 DataRow row3S     = dt_mtt_tableS.NewRow();
			 row3S[noise_minS] = table_porog[102];
			 row3S[noise_maxS] = table_porog[103];
			 dt_mtt_tableS.Rows.Add(row3S);
			 DataRow row4S     = dt_mtt_tableS.NewRow();
			 row4S[noise_minS] = table_porog[104];
			 row4S[noise_maxS] = table_porog[105];
			 dt_mtt_tableS.Rows.Add(row4S);
			 DataRow row5S     = dt_mtt_tableS.NewRow();
			 row5S[noise_minS] = table_porog[106];
			 row5S[noise_maxS] = table_porog[107];
			 dt_mtt_tableS.Rows.Add(row5S);
			 DataRow row6S     = dt_mtt_tableS.NewRow();
			 row6S[noise_minS] = table_porog[108];
			 row6S[noise_maxS] = table_porog[109];
			 dt_mtt_tableS.Rows.Add(row6S);
			 DataRow row7S     = dt_mtt_tableS.NewRow();
			 row7S[noise_minS] = table_porog[110];
			 row7S[noise_maxS] = table_porog[111];
			 dt_mtt_tableS.Rows.Add(row7S);
			 DataRow row8S     = dt_mtt_tableS.NewRow();
			 row8S[noise_minS] = table_porog[112];
			 row8S[noise_maxS] = table_porog[113];
			 dt_mtt_tableS.Rows.Add(row8S);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу
			 dataGridViewParentForm4S.DataSource = dt_mtt_tableS;

			 if(checkBox1.Checked)
				 {
				     // Запрет ввода данных в таблицу
                     if (table_porog[96]  == 0) dataGridViewParentForm4S.Rows[0].Cells[0].ReadOnly = true;
                     if (table_porog[97]  == 0) dataGridViewParentForm4S.Rows[0].Cells[1].ReadOnly = true;
                     if (table_porog[98]  == 0) dataGridViewParentForm4S.Rows[1].Cells[0].ReadOnly = true;
                     if (table_porog[99]  == 0) dataGridViewParentForm4S.Rows[1].Cells[1].ReadOnly = true;
                     if (table_porog[100] == 0) dataGridViewParentForm4S.Rows[2].Cells[0].ReadOnly = true;
                     if (table_porog[101] == 0) dataGridViewParentForm4S.Rows[2].Cells[1].ReadOnly = true;
                     if (table_porog[102] == 0) dataGridViewParentForm4S.Rows[3].Cells[0].ReadOnly = true;
                     if (table_porog[103] == 0) dataGridViewParentForm4S.Rows[3].Cells[1].ReadOnly = true;
                     if (table_porog[104] == 0) dataGridViewParentForm4S.Rows[4].Cells[0].ReadOnly = true;
                     if (table_porog[105] == 0) dataGridViewParentForm4S.Rows[4].Cells[1].ReadOnly = true;
                     if (table_porog[106] == 0) dataGridViewParentForm4S.Rows[5].Cells[0].ReadOnly = true;
                     if (table_porog[107] == 0) dataGridViewParentForm4S.Rows[5].Cells[1].ReadOnly = true;
                     if (table_porog[108] == 0) dataGridViewParentForm4S.Rows[6].Cells[0].ReadOnly = true;
                     if (table_porog[109] == 0) dataGridViewParentForm4S.Rows[6].Cells[1].ReadOnly = true;
                     if (table_porog[110] == 0) dataGridViewParentForm4S.Rows[7].Cells[0].ReadOnly = true;
                     if (table_porog[111] == 0) dataGridViewParentForm4S.Rows[7].Cells[1].ReadOnly = true;
                     if (table_porog[112] == 0) dataGridViewParentForm4S.Rows[8].Cells[0].ReadOnly = true;
                     if (table_porog[113] == 0) dataGridViewParentForm4S.Rows[8].Cells[1].ReadOnly = true;

				     // Отметить ячейки с запретом ввода  красным цветом
                     if (table_porog[96]  == 0) dataGridViewParentForm4S.Rows[0].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[97]  == 0) dataGridViewParentForm4S.Rows[0].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[98]  == 0) dataGridViewParentForm4S.Rows[1].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[99]  == 0) dataGridViewParentForm4S.Rows[1].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[100] == 0) dataGridViewParentForm4S.Rows[2].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[101] == 0) dataGridViewParentForm4S.Rows[2].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[102] == 0) dataGridViewParentForm4S.Rows[3].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[103] == 0) dataGridViewParentForm4S.Rows[3].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[104] == 0) dataGridViewParentForm4S.Rows[4].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[105] == 0) dataGridViewParentForm4S.Rows[4].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[106] == 0) dataGridViewParentForm4S.Rows[5].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[107] == 0) dataGridViewParentForm4S.Rows[5].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[108] == 0) dataGridViewParentForm4S.Rows[6].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[109] == 0) dataGridViewParentForm4S.Rows[6].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[110] == 0) dataGridViewParentForm4S.Rows[7].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[111] == 0) dataGridViewParentForm4S.Rows[7].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[112] == 0) dataGridViewParentForm4S.Rows[8].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[113] == 0) dataGridViewParentForm4S.Rows[8].Cells[1].Style.BackColor = Color.Red;
				 }
			 Mtt_Table_Read = true;
		 }
		 private void mic_table()
		 {
			 DataTable dt_mic_table = new DataTable("DispatchDataTable");                                                // Создаем таблицу
			 DataColumn noise_min   = new DataColumn("мин. Минимальный уровень порога шумов без сигнала", typeof(ushort)); // Определяем, создаем столбцы
			 DataColumn noise_min1  = new DataColumn("минC. Минимальный уровень порога шумов с сигналом", typeof(ushort)); // Определяем, создаем столбцы

			 dt_mic_table.Columns.AddRange(new DataColumn[]                                                                // Создаем столбцы 
			{ noise_min, noise_min1 });

			 //Формируем строку из данных в текстовых полях
			 DataRow row     = dt_mic_table.NewRow();                                      // Формируем строку из данных в текстовых полях
			 row[noise_min]  = table_porog[114];                                             // Записываем данные в первую строку, певого столбца таблицы
			 row[noise_min1] = table_porog[115];                                              // Записываем данные в первую строку, второго столбца таблицы
			 dt_mic_table.Rows.Add(row);
			 DataRow row1     = dt_mic_table.NewRow();
			 row1[noise_min]  = table_porog[116];
			 row1[noise_min1] = table_porog[117];
			 dt_mic_table.Rows.Add(row1);
			 DataRow row2     = dt_mic_table.NewRow();
			 row2[noise_min]  = table_porog[118];
			 row2[noise_min1] = table_porog[119];
			 dt_mic_table.Rows.Add(row2);
			 DataRow row3     = dt_mic_table.NewRow();
			 row3[noise_min]  = table_porog[120];
			 row3[noise_min1] = table_porog[121];
			 dt_mic_table.Rows.Add(row3);
			 DataRow row4     = dt_mic_table.NewRow();
			 row4[noise_min]  = table_porog[122];
			 row4[noise_min1] = table_porog[123];
			 dt_mic_table.Rows.Add(row4);
			 DataRow row5     = dt_mic_table.NewRow();
			 row5[noise_min]  = table_porog[124];
			 row5[noise_min1] = table_porog[125];
			 dt_mic_table.Rows.Add(row5);
			 DataRow row6     = dt_mic_table.NewRow();
			 row6[noise_min]  = table_porog[126];
			 row6[noise_min1] = table_porog[127];
			 dt_mic_table.Rows.Add(row6);
			 DataRow row7     = dt_mic_table.NewRow();
			 row7[noise_min]  = table_porog[128];
			 row7[noise_min1] = table_porog[129];
			 dt_mic_table.Rows.Add(row7);
			 DataRow row8     = dt_mic_table.NewRow();
			 row8[noise_min]  = table_porog[130];
			 row8[noise_min1] = table_porog[131];
			 dt_mic_table.Rows.Add(row8);
             DataRow row9 = dt_mic_table.NewRow();
             row9[noise_min] = table_porog[132];
             row9[noise_min1] = table_porog[133];
             dt_mic_table.Rows.Add(row9);
			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу

			 dataGridViewParentForm5.DataSource = dt_mic_table;

			 //Создаем таблицу
			 DataTable dt_mic_tableS = new DataTable("MTTDataTable");
			 DataColumn noise_minS   = new DataColumn("мин. Минимальный уровень порога сигнала", typeof(ushort));     //Создаем столбцы
			 DataColumn noise_maxS   = new DataColumn("макс. Максимальный уровень порога сигнала", typeof(ushort));   //Создаем столбцы

			 dt_mic_tableS.Columns.AddRange(new DataColumn[] {
			 noise_minS,
			 noise_maxS});

			 //Формируем строку из данных в текстовых полях
			 DataRow rowS     = dt_mic_tableS.NewRow();
			 rowS[noise_minS] = table_porog[134];
			 rowS[noise_maxS] = table_porog[135];
			 dt_mic_tableS.Rows.Add(rowS);
			 DataRow row1S     = dt_mic_tableS.NewRow();
			 row1S[noise_minS] = table_porog[136];
			 row1S[noise_maxS] = table_porog[137];
			 dt_mic_tableS.Rows.Add(row1S);
			 DataRow row2S     = dt_mic_tableS.NewRow();
			 row2S[noise_minS] = table_porog[138];
			 row2S[noise_maxS] = table_porog[139];
			 dt_mic_tableS.Rows.Add(row2S);
			 DataRow row3S     = dt_mic_tableS.NewRow();
			 row3S[noise_minS] = table_porog[140];
			 row3S[noise_maxS] = table_porog[141];
			 dt_mic_tableS.Rows.Add(row3S);
			 DataRow row4S     = dt_mic_tableS.NewRow();
			 row4S[noise_minS] = table_porog[142];
			 row4S[noise_maxS] = table_porog[143];
			 dt_mic_tableS.Rows.Add(row4S);
			 DataRow row5S     = dt_mic_tableS.NewRow();
			 row5S[noise_minS] = table_porog[144];
			 row5S[noise_maxS] = table_porog[145];
			 dt_mic_tableS.Rows.Add(row5S);
			 DataRow row6S     = dt_mic_tableS.NewRow();
			 row6S[noise_minS] = table_porog[146];
			 row6S[noise_maxS] = table_porog[147];
			 dt_mic_tableS.Rows.Add(row6S);
			 DataRow row7S     = dt_mic_tableS.NewRow();
			 row7S[noise_minS] = table_porog[148];
			 row7S[noise_maxS] = table_porog[149];
			 dt_mic_tableS.Rows.Add(row7S);
			 DataRow row8S     = dt_mic_tableS.NewRow();
			 row8S[noise_minS] = table_porog[150];
			 row8S[noise_maxS] = table_porog[151];
			 dt_mic_tableS.Rows.Add(row8S);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу
			 dataGridViewParentForm5S.DataSource = dt_mic_tableS;

			 if(checkBox1.Checked)
				 {
                     if (table_porog[134] == 0) dataGridViewParentForm5S.Rows[0].Cells[0].ReadOnly = true;
                     if (table_porog[135] == 0) dataGridViewParentForm5S.Rows[0].Cells[1].ReadOnly = true;
                     if (table_porog[136] == 0) dataGridViewParentForm5S.Rows[1].Cells[0].ReadOnly = true;
                     if (table_porog[137] == 0) dataGridViewParentForm5S.Rows[1].Cells[1].ReadOnly = true;
                     if (table_porog[138] == 0) dataGridViewParentForm5S.Rows[2].Cells[0].ReadOnly = true;
                     if (table_porog[139] == 0) dataGridViewParentForm5S.Rows[2].Cells[1].ReadOnly = true;
                     if (table_porog[140] == 0) dataGridViewParentForm5S.Rows[3].Cells[0].ReadOnly = true;
                     if (table_porog[141] == 0) dataGridViewParentForm5S.Rows[3].Cells[1].ReadOnly = true;
                     if (table_porog[142] == 0) dataGridViewParentForm5S.Rows[4].Cells[0].ReadOnly = true;
                     if (table_porog[143] == 0) dataGridViewParentForm5S.Rows[4].Cells[1].ReadOnly = true;
                     if (table_porog[144] == 0) dataGridViewParentForm5S.Rows[5].Cells[0].ReadOnly = true;
                     if (table_porog[145] == 0) dataGridViewParentForm5S.Rows[5].Cells[1].ReadOnly = true;
                     if (table_porog[146] == 0) dataGridViewParentForm5S.Rows[6].Cells[0].ReadOnly = true;
                     if (table_porog[147] == 0) dataGridViewParentForm5S.Rows[6].Cells[1].ReadOnly = true;
                     if (table_porog[148] == 0) dataGridViewParentForm5S.Rows[7].Cells[0].ReadOnly = true;
                     if (table_porog[149] == 0) dataGridViewParentForm5S.Rows[7].Cells[1].ReadOnly = true;
                     if (table_porog[150] == 0) dataGridViewParentForm5S.Rows[8].Cells[0].ReadOnly = true;
                     if (table_porog[151] == 0) dataGridViewParentForm4S.Rows[8].Cells[1].ReadOnly = true;


                     if (table_porog[134] == 0) dataGridViewParentForm5S.Rows[0].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[135] == 0) dataGridViewParentForm5S.Rows[0].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[136] == 0) dataGridViewParentForm5S.Rows[1].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[137] == 0) dataGridViewParentForm5S.Rows[1].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[138] == 0) dataGridViewParentForm5S.Rows[2].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[139] == 0) dataGridViewParentForm5S.Rows[2].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[140] == 0) dataGridViewParentForm5S.Rows[3].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[141] == 0) dataGridViewParentForm5S.Rows[3].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[142] == 0) dataGridViewParentForm5S.Rows[4].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[143] == 0) dataGridViewParentForm5S.Rows[4].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[144] == 0) dataGridViewParentForm5S.Rows[5].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[145] == 0) dataGridViewParentForm5S.Rows[5].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[146] == 0) dataGridViewParentForm5S.Rows[6].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[147] == 0) dataGridViewParentForm5S.Rows[6].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[148] == 0) dataGridViewParentForm5S.Rows[7].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[149] == 0) dataGridViewParentForm5S.Rows[7].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[150] == 0) dataGridViewParentForm5S.Rows[8].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[151] == 0) dataGridViewParentForm5S.Rows[8].Cells[1].Style.BackColor = Color.Red;
				 }
			 Mic_Table_Read = true;
		 }
		 private void ggs_table()
		 {
			 DataTable dt_ggs_table = new DataTable("DispatchDataTable");                                                // Создаем таблицу
			 DataColumn noise_min   = new DataColumn("мин. Минимальный уровень порога шумов без сигнала", typeof(ushort)); // Определяем, создаем столбцы
			 DataColumn noise_min1  = new DataColumn("минC. Минимальный уровень порога шумов с сигналом", typeof(ushort)); // Определяем, создаем столбцы

			 dt_ggs_table.Columns.AddRange(new DataColumn[]                                                                // Создаем столбцы 
			{ noise_min, noise_min1 });

			 //Формируем строку из данных в текстовых полях
			 DataRow row      = dt_ggs_table.NewRow();                                      // Формируем строку из данных в текстовых полях
			 row[noise_min]   = table_porog[152];                                           // Записываем данные в первую строку, певого столбца таблицы
			 row[noise_min1]  = table_porog[153];                                           // Записываем данные в первую строку, второго столбца таблицы
			 dt_ggs_table.Rows.Add(row);
			 DataRow row1     = dt_ggs_table.NewRow();
			 row1[noise_min]  = table_porog[154];
			 row1[noise_min1] = table_porog[155];
			 dt_ggs_table.Rows.Add(row1);
			 DataRow row2     = dt_ggs_table.NewRow();
			 row2[noise_min]  = table_porog[156];
			 row2[noise_min1] = table_porog[157];
			 dt_ggs_table.Rows.Add(row2);
			 DataRow row3     = dt_ggs_table.NewRow();
			 row3[noise_min]  = table_porog[158];
			 row3[noise_min1] = table_porog[159];
			 dt_ggs_table.Rows.Add(row3);
			 DataRow row4     = dt_ggs_table.NewRow();
			 row4[noise_min]  = table_porog[160];
			 row4[noise_min1] = table_porog[161];
			 dt_ggs_table.Rows.Add(row4);
			 DataRow row5     = dt_ggs_table.NewRow();
			 row5[noise_min]  = table_porog[162];
			 row5[noise_min1] = table_porog[163];
			 dt_ggs_table.Rows.Add(row5);
			 DataRow row6     = dt_ggs_table.NewRow();
			 row6[noise_min]  = table_porog[164];
			 row6[noise_min1] = table_porog[165];
			 dt_ggs_table.Rows.Add(row6);
			 DataRow row7     = dt_ggs_table.NewRow();
			 row7[noise_min]  = table_porog[166];
			 row7[noise_min1] = table_porog[167];
			 dt_ggs_table.Rows.Add(row7);
			 DataRow row8     = dt_ggs_table.NewRow();
			 row8[noise_min]  = table_porog[168];
			 row8[noise_min1] = table_porog[169];
			 dt_ggs_table.Rows.Add(row8);
             DataRow row9 = dt_ggs_table.NewRow();
             row9[noise_min] = table_porog[170];
             row9[noise_min1] = table_porog[171];
             dt_ggs_table.Rows.Add(row9);
			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу

			 dataGridViewParentForm6.DataSource = dt_ggs_table;

			 //Создаем таблицу
			 DataTable dt_ggs_tableS = new DataTable("MTTDataTable");
			 DataColumn noise_minS   = new DataColumn("мин. Минимальный уровень порога сигнала", typeof(ushort));     //Создаем столбцы
			 DataColumn noise_maxS   = new DataColumn("макс. Максимальный уровень порога сигнала", typeof(ushort));   //Создаем столбцы

			 dt_ggs_tableS.Columns.AddRange(new DataColumn[] {
			 noise_minS,
			 noise_maxS});

			 //Формируем строку из данных в текстовых полях
			 DataRow rowS      = dt_ggs_tableS.NewRow();
			 rowS[noise_minS]  = table_porog[172];
			 rowS[noise_maxS]  = table_porog[173];
			 dt_ggs_tableS.Rows.Add(rowS);
			 DataRow row1S     = dt_ggs_tableS.NewRow();
			 row1S[noise_minS] = table_porog[174];
			 row1S[noise_maxS] = table_porog[175];
			 dt_ggs_tableS.Rows.Add(row1S);
			 DataRow row2S     = dt_ggs_tableS.NewRow();
			 row2S[noise_minS] = table_porog[176];
			 row2S[noise_maxS] = table_porog[177];
			 dt_ggs_tableS.Rows.Add(row2S);
			 DataRow row3S     = dt_ggs_tableS.NewRow();
			 row3S[noise_minS] = table_porog[178];
			 row3S[noise_maxS] = table_porog[179];
			 dt_ggs_tableS.Rows.Add(row3S);
			 DataRow row4S     = dt_ggs_tableS.NewRow();
			 row4S[noise_minS] = table_porog[180];
			 row4S[noise_maxS] = table_porog[181];
			 dt_ggs_tableS.Rows.Add(row4S);
			 DataRow row5S     = dt_ggs_tableS.NewRow();
			 row5S[noise_minS] = table_porog[182];
			 row5S[noise_maxS] = table_porog[183];
			 dt_ggs_tableS.Rows.Add(row5S);
			 DataRow row6S     = dt_ggs_tableS.NewRow();
			 row6S[noise_minS] = table_porog[184];
			 row6S[noise_maxS] = table_porog[185];
			 dt_ggs_tableS.Rows.Add(row6S);
			 DataRow row7S     = dt_ggs_tableS.NewRow();
			 row7S[noise_minS] = table_porog[186];
			 row7S[noise_maxS] = table_porog[187];
			 dt_ggs_tableS.Rows.Add(row7S);
			 DataRow row8S     = dt_ggs_tableS.NewRow();
			 row8S[noise_minS] = table_porog[188];
			 row8S[noise_maxS] = table_porog[189];
			 dt_ggs_tableS.Rows.Add(row8S);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу
			 dataGridViewParentForm6S.DataSource = dt_ggs_tableS;

			 if(checkBox1.Checked)
				 {
                     if (table_porog[172] == 0) dataGridViewParentForm6S.Rows[0].Cells[0].ReadOnly = true;
                     if (table_porog[173] == 0) dataGridViewParentForm6S.Rows[0].Cells[1].ReadOnly = true;
                     if (table_porog[174] == 0) dataGridViewParentForm6S.Rows[1].Cells[0].ReadOnly = true;
                     if (table_porog[175] == 0) dataGridViewParentForm6S.Rows[1].Cells[1].ReadOnly = true;
                     if (table_porog[176] == 0) dataGridViewParentForm6S.Rows[2].Cells[0].ReadOnly = true;
                     if (table_porog[177] == 0) dataGridViewParentForm6S.Rows[2].Cells[1].ReadOnly = true;
                     if (table_porog[178] == 0) dataGridViewParentForm6S.Rows[3].Cells[0].ReadOnly = true;
                     if (table_porog[179] == 0) dataGridViewParentForm6S.Rows[3].Cells[1].ReadOnly = true;
                     if (table_porog[180] == 0) dataGridViewParentForm6S.Rows[4].Cells[0].ReadOnly = true;
                     if (table_porog[181] == 0) dataGridViewParentForm6S.Rows[4].Cells[1].ReadOnly = true;
                     if (table_porog[182] == 0) dataGridViewParentForm6S.Rows[5].Cells[0].ReadOnly = true;
                     if (table_porog[183] == 0) dataGridViewParentForm6S.Rows[5].Cells[1].ReadOnly = true;
                     if (table_porog[184] == 0) dataGridViewParentForm6S.Rows[6].Cells[0].ReadOnly = true;
                     if (table_porog[185] == 0) dataGridViewParentForm6S.Rows[6].Cells[1].ReadOnly = true;
                     if (table_porog[186] == 0) dataGridViewParentForm6S.Rows[7].Cells[0].ReadOnly = true;
                     if (table_porog[187] == 0) dataGridViewParentForm6S.Rows[7].Cells[1].ReadOnly = true;
                     if (table_porog[188] == 0) dataGridViewParentForm6S.Rows[8].Cells[0].ReadOnly = true;
                     if (table_porog[189] == 0) dataGridViewParentForm6S.Rows[8].Cells[1].ReadOnly = true;


                     if (table_porog[172] == 0) dataGridViewParentForm6S.Rows[0].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[173] == 0) dataGridViewParentForm6S.Rows[0].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[174] == 0) dataGridViewParentForm6S.Rows[1].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[175] == 0) dataGridViewParentForm6S.Rows[1].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[176] == 0) dataGridViewParentForm6S.Rows[2].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[177] == 0) dataGridViewParentForm6S.Rows[2].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[178] == 0) dataGridViewParentForm6S.Rows[3].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[179] == 0) dataGridViewParentForm6S.Rows[3].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[180] == 0) dataGridViewParentForm6S.Rows[4].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[181] == 0) dataGridViewParentForm6S.Rows[4].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[182] == 0) dataGridViewParentForm6S.Rows[5].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[183] == 0) dataGridViewParentForm6S.Rows[5].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[184] == 0) dataGridViewParentForm6S.Rows[6].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[185] == 0) dataGridViewParentForm6S.Rows[6].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[186] == 0) dataGridViewParentForm6S.Rows[7].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[187] == 0) dataGridViewParentForm6S.Rows[7].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[188] == 0) dataGridViewParentForm6S.Rows[8].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[189] == 0) dataGridViewParentForm6S.Rows[8].Cells[1].Style.BackColor = Color.Red;
				 }
			 Ggs_Table_Read = true;
		 }
		 private void radio1_table()
		 {
			 DataTable dt_radio1_table = new DataTable("DispatchDataTable");                                                // Создаем таблицу
			 DataColumn noise_min      = new DataColumn("мин. Минимальный уровень порога шумов без сигнала", typeof(ushort)); // Определяем, создаем столбцы
			 DataColumn noise_min1     = new DataColumn("минC. Минимальный уровень порога шумов с сигналом", typeof(ushort)); // Определяем, создаем столбцы

			 dt_radio1_table.Columns.AddRange(new DataColumn[]                                                                // Создаем столбцы 
			{ noise_min, noise_min1 });

			 //Формируем строку из данных в текстовых полях
			 DataRow row      = dt_radio1_table.NewRow();                                      // Формируем строку из данных в текстовых полях
			 row[noise_min]   = table_porog[190];                                             // Записываем данные в первую строку, певого столбца таблицы
			 row[noise_min1]  = table_porog[191];                                              // Записываем данные в первую строку, второго столбца таблицы
			 dt_radio1_table.Rows.Add(row);
			 DataRow row1     = dt_radio1_table.NewRow();
			 row1[noise_min]  = table_porog[192];
			 row1[noise_min1] = table_porog[193];
			 dt_radio1_table.Rows.Add(row1);
			 DataRow row2     = dt_radio1_table.NewRow();
			 row2[noise_min]  = table_porog[194];
			 row2[noise_min1] = table_porog[195];
			 dt_radio1_table.Rows.Add(row2);
			 DataRow row3     = dt_radio1_table.NewRow();
			 row3[noise_min]  = table_porog[196];
			 row3[noise_min1] = table_porog[197];
			 dt_radio1_table.Rows.Add(row3);
			 DataRow row4     = dt_radio1_table.NewRow();
			 row4[noise_min]  = table_porog[198];
			 row4[noise_min1] = table_porog[199];
			 dt_radio1_table.Rows.Add(row4);
			 DataRow row5     = dt_radio1_table.NewRow();
			 row5[noise_min]  = table_porog[200];
			 row5[noise_min1] = table_porog[201];
			 dt_radio1_table.Rows.Add(row5);
			 DataRow row6     = dt_radio1_table.NewRow();
			 row6[noise_min]  = table_porog[202];
			 row6[noise_min1] = table_porog[203];
			 dt_radio1_table.Rows.Add(row6);
			 DataRow row7     = dt_radio1_table.NewRow();
			 row7[noise_min]  = table_porog[204];
			 row7[noise_min1] = table_porog[205];
			 dt_radio1_table.Rows.Add(row7);
			 DataRow row8     = dt_radio1_table.NewRow();
			 row8[noise_min]  = table_porog[206];
			 row8[noise_min1] = table_porog[207];
			 dt_radio1_table.Rows.Add(row8);
             DataRow row9 = dt_radio1_table.NewRow();
             row9[noise_min] = table_porog[208];
             row9[noise_min1] = table_porog[209];
             dt_radio1_table.Rows.Add(row9);
			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу

			 dataGridViewParentForm7.DataSource = dt_radio1_table;

			 //Создаем таблицу
			 DataTable dt_radio1_tableS = new DataTable("MTTDataTable");
			 DataColumn noise_minS      = new DataColumn("мин. Минимальный уровень порога сигнала", typeof(ushort));     //Создаем столбцы
			 DataColumn noise_maxS      = new DataColumn("макс. Максимальный уровень порога сигнала", typeof(ushort));   //Создаем столбцы

			 dt_radio1_tableS.Columns.AddRange(new DataColumn[] {
			 noise_minS,
			 noise_maxS});

			 //Формируем строку из данных в текстовых полях
			 DataRow rowS      = dt_radio1_tableS.NewRow();
			 rowS[noise_minS]  = table_porog[210];
			 rowS[noise_maxS]  = table_porog[211];
			 dt_radio1_tableS.Rows.Add(rowS);
			 DataRow row1S     = dt_radio1_tableS.NewRow();
			 row1S[noise_minS] = table_porog[212];
			 row1S[noise_maxS] = table_porog[213];
			 dt_radio1_tableS.Rows.Add(row1S);
			 DataRow row2S     = dt_radio1_tableS.NewRow();
			 row2S[noise_minS] = table_porog[214];
			 row2S[noise_maxS] = table_porog[215];
			 dt_radio1_tableS.Rows.Add(row2S);
			 DataRow row3S     = dt_radio1_tableS.NewRow();
			 row3S[noise_minS] = table_porog[216];
			 row3S[noise_maxS] = table_porog[217];
			 dt_radio1_tableS.Rows.Add(row3S);
			 DataRow row4S     = dt_radio1_tableS.NewRow();
			 row4S[noise_minS] = table_porog[218];
			 row4S[noise_maxS] = table_porog[219];
			 dt_radio1_tableS.Rows.Add(row4S);
			 DataRow row5S     = dt_radio1_tableS.NewRow();
			 row5S[noise_minS] = table_porog[220];
			 row5S[noise_maxS] = table_porog[221];
			 dt_radio1_tableS.Rows.Add(row5S);
			 DataRow row6S     = dt_radio1_tableS.NewRow();
			 row6S[noise_minS] = table_porog[222];
			 row6S[noise_maxS] = table_porog[223];
			 dt_radio1_tableS.Rows.Add(row6S);
			 DataRow row7S     = dt_radio1_tableS.NewRow();
			 row7S[noise_minS] = table_porog[224];
			 row7S[noise_maxS] = table_porog[225];
			 dt_radio1_tableS.Rows.Add(row7S);
			 DataRow row8S     = dt_radio1_tableS.NewRow();
			 row8S[noise_minS] = table_porog[226];
			 row8S[noise_maxS] = table_porog[227];
			 dt_radio1_tableS.Rows.Add(row8S);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу
			 dataGridViewParentForm7S.DataSource = dt_radio1_tableS;

			 if(checkBox1.Checked)
				 {
                     if (table_porog[210] == 0) dataGridViewParentForm7S.Rows[0].Cells[0].ReadOnly = true;
                     if (table_porog[211] == 0) dataGridViewParentForm7S.Rows[0].Cells[1].ReadOnly = true;
                     if (table_porog[212] == 0) dataGridViewParentForm7S.Rows[1].Cells[0].ReadOnly = true;
                     if (table_porog[213] == 0) dataGridViewParentForm7S.Rows[1].Cells[1].ReadOnly = true;
                     if (table_porog[214] == 0) dataGridViewParentForm7S.Rows[2].Cells[0].ReadOnly = true;
                     if (table_porog[215] == 0) dataGridViewParentForm7S.Rows[2].Cells[1].ReadOnly = true;
                     if (table_porog[216] == 0) dataGridViewParentForm7S.Rows[3].Cells[0].ReadOnly = true;
                     if (table_porog[217] == 0) dataGridViewParentForm7S.Rows[3].Cells[1].ReadOnly = true;
                     if (table_porog[218] == 0) dataGridViewParentForm7S.Rows[4].Cells[0].ReadOnly = true;
                     if (table_porog[219] == 0) dataGridViewParentForm7S.Rows[4].Cells[1].ReadOnly = true;
                     if (table_porog[220] == 0) dataGridViewParentForm7S.Rows[5].Cells[0].ReadOnly = true;
                     if (table_porog[221] == 0) dataGridViewParentForm7S.Rows[5].Cells[1].ReadOnly = true;
                     if (table_porog[222] == 0) dataGridViewParentForm7S.Rows[6].Cells[0].ReadOnly = true;
                     if (table_porog[223] == 0) dataGridViewParentForm7S.Rows[6].Cells[1].ReadOnly = true;
                     if (table_porog[224] == 0) dataGridViewParentForm7S.Rows[7].Cells[0].ReadOnly = true;
                     if (table_porog[225] == 0) dataGridViewParentForm7S.Rows[7].Cells[1].ReadOnly = true;
                     if (table_porog[226] == 0) dataGridViewParentForm7S.Rows[8].Cells[0].ReadOnly = true;
                     if (table_porog[227] == 0) dataGridViewParentForm7S.Rows[8].Cells[1].ReadOnly = true;


                     if (table_porog[210] == 0) dataGridViewParentForm7S.Rows[0].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[211] == 0) dataGridViewParentForm7S.Rows[0].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[212] == 0) dataGridViewParentForm7S.Rows[1].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[213] == 0) dataGridViewParentForm7S.Rows[1].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[214] == 0) dataGridViewParentForm7S.Rows[2].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[215] == 0) dataGridViewParentForm7S.Rows[2].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[216] == 0) dataGridViewParentForm7S.Rows[3].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[217] == 0) dataGridViewParentForm7S.Rows[3].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[218] == 0) dataGridViewParentForm7S.Rows[4].Cells[0].ReadOnly = true;
                     if (table_porog[219] == 0) dataGridViewParentForm7S.Rows[4].Cells[1].ReadOnly = true;
                     if (table_porog[220] == 0) dataGridViewParentForm7S.Rows[5].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[221] == 0) dataGridViewParentForm7S.Rows[5].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[222] == 0) dataGridViewParentForm7S.Rows[6].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[223] == 0) dataGridViewParentForm7S.Rows[6].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[224] == 0) dataGridViewParentForm7S.Rows[7].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[225] == 0) dataGridViewParentForm7S.Rows[7].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[226] == 0) dataGridViewParentForm7S.Rows[8].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[227] == 0) dataGridViewParentForm7S.Rows[8].Cells[1].Style.BackColor = Color.Red;
				 }
			 Rad1_Table_Read = true;
		 }
		 private void radio2_table()
		 {
			 DataTable dt_radio2_table = new DataTable("DispatchDataTable");                                                // Создаем таблицу
			 DataColumn noise_min      = new DataColumn("мин. Минимальный уровень порога шумов без сигнала", typeof(ushort)); // Определяем, создаем столбцы
			 DataColumn noise_min1     = new DataColumn("минC. Минимальный уровень порога шумов с сигналом", typeof(ushort)); // Определяем, создаем столбцы

			 dt_radio2_table.Columns.AddRange(new DataColumn[]                                                                // Создаем столбцы 
			{ noise_min, noise_min1 });

			 //Формируем строку из данных в текстовых полях
			 DataRow row      = dt_radio2_table.NewRow();                                      // Формируем строку из данных в текстовых полях
			 row[noise_min]   = table_porog[228];                                             // Записываем данные в первую строку, певого столбца таблицы
			 row[noise_min1]  = table_porog[229];                                              // Записываем данные в первую строку, второго столбца таблицы
			 dt_radio2_table.Rows.Add(row);
			 DataRow row1     = dt_radio2_table.NewRow();
			 row1[noise_min]  = table_porog[230];
			 row1[noise_min1] = table_porog[231];
			 dt_radio2_table.Rows.Add(row1);
			 DataRow row2     = dt_radio2_table.NewRow();
			 row2[noise_min]  = table_porog[232];
			 row2[noise_min1] = table_porog[233];
			 dt_radio2_table.Rows.Add(row2);
			 DataRow row3     = dt_radio2_table.NewRow();
			 row3[noise_min]  = table_porog[234];
			 row3[noise_min1] = table_porog[235];
			 dt_radio2_table.Rows.Add(row3);
			 DataRow row4     = dt_radio2_table.NewRow();
			 row4[noise_min]  = table_porog[236];
			 row4[noise_min1] = table_porog[237];
			 dt_radio2_table.Rows.Add(row4);
			 DataRow row5     = dt_radio2_table.NewRow();
			 row5[noise_min]  = table_porog[238];
			 row5[noise_min1] = table_porog[239];
			 dt_radio2_table.Rows.Add(row5);
			 DataRow row6     = dt_radio2_table.NewRow();
			 row6[noise_min]  = table_porog[240];
			 row6[noise_min1] = table_porog[241];
			 dt_radio2_table.Rows.Add(row6);
			 DataRow row7     = dt_radio2_table.NewRow();
			 row7[noise_min]  = table_porog[242];
			 row7[noise_min1] = table_porog[243];
			 dt_radio2_table.Rows.Add(row7);
			 DataRow row8     = dt_radio2_table.NewRow();
			 row8[noise_min]  = table_porog[244];
			 row8[noise_min1] = table_porog[245];
			 dt_radio2_table.Rows.Add(row8);
             DataRow row9 = dt_radio2_table.NewRow();
             row9[noise_min] = table_porog[246];
             row9[noise_min1] = table_porog[247];
             dt_radio2_table.Rows.Add(row9);
			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу

			 dataGridViewParentForm8.DataSource = dt_radio2_table;

			 //Создаем таблицу
			 DataTable dt_radio2_tableS = new DataTable("MTTDataTable");
			 DataColumn noise_minS      = new DataColumn("мин. Минимальный уровень порога сигнала", typeof(ushort));     //Создаем столбцы
			 DataColumn noise_maxS      = new DataColumn("макс. Максимальный уровень порога сигнала", typeof(ushort));   //Создаем столбцы

			 dt_radio2_tableS.Columns.AddRange(new DataColumn[] {
			 noise_minS,
			 noise_maxS});

			 //Формируем строку из данных в текстовых полях
			 DataRow rowS      = dt_radio2_tableS.NewRow();
			 rowS[noise_minS]  = table_porog[248];
			 rowS[noise_maxS]  = table_porog[249];
			 dt_radio2_tableS.Rows.Add(rowS);
			 DataRow row1S     = dt_radio2_tableS.NewRow();
			 row1S[noise_minS] = table_porog[250];
			 row1S[noise_maxS] = table_porog[251];
			 dt_radio2_tableS.Rows.Add(row1S);
			 DataRow row2S     = dt_radio2_tableS.NewRow();
			 row2S[noise_minS] = table_porog[252];
			 row2S[noise_maxS] = table_porog[253];
			 dt_radio2_tableS.Rows.Add(row2S);
			 DataRow row3S     = dt_radio2_tableS.NewRow();
			 row3S[noise_minS] = table_porog[254];
			 row3S[noise_maxS] = table_porog[255];
			 dt_radio2_tableS.Rows.Add(row3S);
			 DataRow row4S     = dt_radio2_tableS.NewRow();
			 row4S[noise_minS] = table_porog[256];
			 row4S[noise_maxS] = table_porog[257];
			 dt_radio2_tableS.Rows.Add(row4S);
			 DataRow row5S     = dt_radio2_tableS.NewRow();
			 row5S[noise_minS] = table_porog[258];
			 row5S[noise_maxS] = table_porog[259];
			 dt_radio2_tableS.Rows.Add(row5S);
			 DataRow row6S     = dt_radio2_tableS.NewRow();
			 row6S[noise_minS] = table_porog[260];
			 row6S[noise_maxS] = table_porog[261];
			 dt_radio2_tableS.Rows.Add(row6S);
			 DataRow row7S     = dt_radio2_tableS.NewRow();
			 row7S[noise_minS] = table_porog[262];
			 row7S[noise_maxS] = table_porog[263];
			 dt_radio2_tableS.Rows.Add(row7S);
			 DataRow row8S     = dt_radio2_tableS.NewRow();
			 row8S[noise_minS] = table_porog[264];
			 row8S[noise_maxS] = table_porog[265];
			 dt_radio2_tableS.Rows.Add(row8S);

			 //Генерируем событие с именованным аргументом
			 //в класс аргумента передаем созданную таблицу
			 dataGridViewParentForm8S.DataSource = dt_radio2_tableS;

			 if(checkBox1.Checked)
				 {
                     if (table_porog[248] == 0) dataGridViewParentForm8S.Rows[0].Cells[0].ReadOnly = true;
                     if (table_porog[249] == 0) dataGridViewParentForm8S.Rows[0].Cells[1].ReadOnly = true;
                     if (table_porog[250] == 0) dataGridViewParentForm8S.Rows[1].Cells[0].ReadOnly = true;
                     if (table_porog[251] == 0) dataGridViewParentForm8S.Rows[1].Cells[1].ReadOnly = true;
                     if (table_porog[252] == 0) dataGridViewParentForm8S.Rows[2].Cells[0].ReadOnly = true;
                     if (table_porog[253] == 0) dataGridViewParentForm8S.Rows[2].Cells[1].ReadOnly = true;
                     if (table_porog[254] == 0) dataGridViewParentForm8S.Rows[3].Cells[0].ReadOnly = true;
                     if (table_porog[255] == 0) dataGridViewParentForm8S.Rows[3].Cells[1].ReadOnly = true;
                     if (table_porog[256] == 0) dataGridViewParentForm8S.Rows[4].Cells[0].ReadOnly = true;
                     if (table_porog[257] == 0) dataGridViewParentForm8S.Rows[4].Cells[1].ReadOnly = true;
                     if (table_porog[258] == 0) dataGridViewParentForm8S.Rows[5].Cells[0].ReadOnly = true;
                     if (table_porog[259] == 0) dataGridViewParentForm8S.Rows[5].Cells[1].ReadOnly = true;
                     if (table_porog[260] == 0) dataGridViewParentForm8S.Rows[6].Cells[0].ReadOnly = true;
                     if (table_porog[261] == 0) dataGridViewParentForm8S.Rows[6].Cells[1].ReadOnly = true;
                     if (table_porog[262] == 0) dataGridViewParentForm8S.Rows[7].Cells[0].ReadOnly = true;
                     if (table_porog[263] == 0) dataGridViewParentForm8S.Rows[7].Cells[1].ReadOnly = true;
                     if (table_porog[264] == 0) dataGridViewParentForm8S.Rows[8].Cells[0].ReadOnly = true;
                     if (table_porog[265] == 0) dataGridViewParentForm8S.Rows[8].Cells[1].ReadOnly = true;


                     if (table_porog[248] == 0) dataGridViewParentForm8S.Rows[0].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[249] == 0) dataGridViewParentForm8S.Rows[0].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[250] == 0) dataGridViewParentForm8S.Rows[1].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[251] == 0) dataGridViewParentForm8S.Rows[1].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[252] == 0) dataGridViewParentForm8S.Rows[2].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[253] == 0) dataGridViewParentForm8S.Rows[2].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[254] == 0) dataGridViewParentForm8S.Rows[3].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[255] == 0) dataGridViewParentForm8S.Rows[3].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[256] == 0) dataGridViewParentForm8S.Rows[4].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[257] == 0) dataGridViewParentForm8S.Rows[4].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[258] == 0) dataGridViewParentForm8S.Rows[5].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[259] == 0) dataGridViewParentForm8S.Rows[5].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[260] == 0) dataGridViewParentForm8S.Rows[6].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[261] == 0) dataGridViewParentForm8S.Rows[6].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[262] == 0) dataGridViewParentForm8S.Rows[7].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[263] == 0) dataGridViewParentForm8S.Rows[7].Cells[1].Style.BackColor = Color.Red;
                     if (table_porog[264] == 0) dataGridViewParentForm8S.Rows[8].Cells[0].Style.BackColor = Color.Red;
                     if (table_porog[265] == 0) dataGridViewParentForm8S.Rows[8].Cells[1].Style.BackColor = Color.Red;
				 }
			 Rad2_Table_Read = true;
		 }
#endregion

// Проверка ввода чисел в таблицы
#region check_table  

		 // Вывод сообщения об ошибке ввода чисел в таблицы
		 private void dataGridViewParentForm2_DataError(object sender, DataGridViewDataErrorEventArgs e)
		 {
			 MessageBox.Show("Разрешено вводит только числа", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
			 return;
		 }
 
		// Проверка ввода чисел в таблицы
		 private void dataGridViewParentForm2_EditingControlShowing(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm2S_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm3_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm3S_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm4_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm4S_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm5_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm5S_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm6_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm6S_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm7_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm7S_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm8_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }
		 private void dataGridViewParentForm8S_KeyPress(object sender, KeyPressEventArgs e)
		 {
			 if ((e.KeyChar <= 47 || e.KeyChar >= 58) && e.KeyChar != 8)
				 e.Handled = true;
		 }

#endregion

		 void Bw_DoWork_MODBUS(object sender, DoWorkEventArgs e)
		 {
			 BackgroundWorker bw_modbus_test = (BackgroundWorker)sender;
			 int data = 0;
			 int iterations = (int)e.Argument + 1;
			 int i = 0;
			 short[] readVals = new short[125];
			 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

			 
			 // Получить имя файла
			 startWrReg = 120;
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 23);                     // Контроль имени файла
			 if ((res == BusProtocolErrors.FTALK_SUCCESS))
			 {
				 test_end1();
				 Thread.Sleep(100);
				 startRdReg = 112;                                                            // 40112 Адрес дата/время для получения имени файла
				 numRdRegs = 4;
				 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
 
				 if ((res == BusProtocolErrors.FTALK_SUCCESS))
				 {

					 toolStripStatusLabel6.Text = "";
					 toolStripStatusLabel6.Text = (toolStripStatusLabel6.Text + readVals[0]);
					 if (readVals[1] < 10)
					 {
						 toolStripStatusLabel6.Text += ("0" + readVals[1]);
					 }
					 else
					 {
						 toolStripStatusLabel6.Text += (readVals[1]);
					 }
					 if (readVals[2] < 10)
					 {
						 toolStripStatusLabel6.Text += ("0" + readVals[2]);
					 }
					 else
					 {
						 toolStripStatusLabel6.Text += (readVals[2]);
					 }
					 if (readVals[3] < 10)
					 {
						 toolStripStatusLabel6.Text += ("0" + readVals[3] + ".KAM");
					 }
					 else
					 {
						 toolStripStatusLabel6.Text += (readVals[3] + ".KAM");
					 }
						   
	   

				 }
				 else
				 {
					 MODBUS_SUCCESS = false;
					 toolStripStatusLabel1.Text = "    MODBUS ERROR (3) ";
					 toolStripStatusLabel4.Text = ("Ошибка получения имени файла! ");  // Обработка ошибки.
					 stop_test_modbus1();
					 Thread.Sleep(2000);
				 }

			 }
			 else
			 {
				 MODBUS_SUCCESS = false;
				 toolStripStatusLabel1.Text = "    MODBUS ERROR(2) ";
				 toolStripStatusLabel4.Text = ("Команда получения имени файла не выполнена !");  // Обработка ошибки.
				 stop_test_modbus1();
				 Thread.Sleep(2000);
			 }
		   
			 //----------------------------------------------

			 do
			 {
				 toolStripStatusLabel8.Text = ("  Проверка связи по протоколу MODBUS");
				 startRdReg = 46;                                                                    // 40046 Адрес дата/время контроллера  
				 numRdRegs = 6;
				 res = myProtocol.readMultipleRegisters(1, startRdReg, readVals, numRdRegs);
				 if ((res == BusProtocolErrors.FTALK_SUCCESS))
				 {
					 MODBUS_SUCCESS = true;                                                          // Признак выполнения контроля связи MODBUS
					 toolStripStatusLabel1.Text = "    MODBUS ON    ";
					// toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5 УСТАНОВЛЕНА !");     // Обработка ошибки.
					 s_txt80 = (readVals[0] + "." + readVals[1] + "." + readVals[2] + "   " + readVals[3] + ":" + readVals[4] + ":" + readVals[5]);
				 }
				 else
				 {
					 MODBUS_SUCCESS             = false;
					 Com2_SUCCESS               = false;
					 toolStripStatusLabel1.Text = "    MODBUS ERROR(1m) ";
					 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
				 }


				 if (bw_modbus_test.CancellationPending)
				 {
					 e.Cancel = true;
				 }

				 bw_modbus_test.ReportProgress(i);
				 Thread.Sleep(100);
				 i++;
				 if (i > 100) i = 0;
				 data += 50;
				 if (data > 1000) data = 50;

			 } while (!e.Cancel);

			 e.Result = data;
		 }
		 private void Bw_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
		 {
			 if (e.Cancelled)
			 {
				 // progressLabel.Text = "Выполнение задачи отменено";
				 progressBar2.Value = progressBar2.Minimum;
				 Test_MODBUS = false;                                               // Проверка MODBUS завершена   
				 if (Tab_index == 3) toolStripStatusLabel8.Text = ("Установка параметров подключения Камертон 5.0");
				 timer1.Stop();
			 }
			 else if (e.Result != null)
			 {
				 Test_MODBUS = false;                                               // Проверка MODBUS завершена    
				 timer1.Stop();
				 if (Tab_index == 3) toolStripStatusLabel8.Text = ("Установка параметров подключения Камертон 5.0");
				 //  progressLabel.Text = "Выполнение задачи завершено";
			 }

		 }
		 private void Bw_ProgressChanged(object sender, ProgressChangedEventArgs e)
		 {

			 label80.Text = DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture);
			 label83.Text = s_txt80;
			 toolStripStatusLabel2.Text = ("Время : " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture));
			 label150.Text = toolStripStatusLabel6.Text;
			 label98.Text = string.Format("{0}%", e.ProgressPercentage);
			 progressBar2.Value = e.ProgressPercentage;

			 if (MODBUS_SUCCESS)
			 {
				 toolStripStatusLabel1.BackColor = Color.Lime;
				 if (serial2_connect_ok == true)
				 {
					 Com2_SUCCESS = true;
				 }

			 }
			 else
			 {
				 toolStripStatusLabel1.BackColor = Color.Red;
			 }
			 if (Com2_SUCCESS)
			 {
				 toolStripStatusLabel4.BackColor = Color.Lime;
				 toolStripStatusLabel4.ForeColor = Color.Black;
			 }
			 else
			 {
				 toolStripStatusLabel4.ForeColor = Color.Red;
				 toolStripStatusLabel4.BackColor = Color.White;
			 }
		 }
		 private void start_test_modbus1()
		 {
			 if (serial_connect_ok)                                                                    // Признак успешного подключения serial_connect
			 {
				 timer1.Start();
				 if (!_bw_modbus_test.IsBusy)
				 {
					 // _bw_modbus_test.RunWorkerAsync(progressBar2.Maximum);
					 Test_MODBUS = true;                                               // Проверка MODBUS стартовала     
					 _bw_modbus_test.RunWorkerAsync(100);
				 }
			 }
		 }
		 private void stop_test_modbus1()
		 {
			 if (_bw_modbus_test.IsBusy)
			 {
				 _bw_modbus_test.CancelAsync();
				 //timer1.Stop(); ;                                     //  Сканирование MODBUS остановлено
			 }
			 timer1.Stop();
		 }

		 void Bw_DoWorkAll_Test(object sender, DoWorkEventArgs e)
		 {
			 BackgroundWorker bw_DoWorkAll_Test = (BackgroundWorker)sender;                     // Получает или задает значение, показывающее, 
			 int data = 0;                                                                      // может ли объект BackgroundWorker сообщать о ходе выполнения.
			 int iterations = (int)e.Argument + 1;
			 short[] readVals = new short[125];
			 int startRdReg;
			 int numRdRegs;
			 int TestNst;
			 int Visual_N_Test = 1;
			 bool TestRepeatStart = true;
			 startWrReg = 120;
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 23);                      // Запрос имени текущего файла 
			 if ((res == BusProtocolErrors.FTALK_SUCCESS))
			 {
				 Thread.Sleep(1000);
				 startRdReg = 112;                                                            // 40112 Адрес дата/время для получения имени файла
				 numRdRegs = 4;
				 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
				 if ((res == BusProtocolErrors.FTALK_SUCCESS))
				 {

					 toolStripStatusLabel6.Text = "";
					 toolStripStatusLabel6.Text = (toolStripStatusLabel6.Text + readVals[0]);
					 if (readVals[1] < 10)
					 {
						 toolStripStatusLabel6.Text += ("0" + readVals[1]);
					 }
					 else
					 {
						 toolStripStatusLabel6.Text += (readVals[1]);
					 }
					 if (readVals[2] < 10)
					 {
						 toolStripStatusLabel6.Text += ("0" + readVals[2]);
					 }
					 else
					 {
						 toolStripStatusLabel6.Text += (readVals[2]);
					 }
					 if (readVals[3] < 10)
					 {
						 toolStripStatusLabel6.Text += ("0" + readVals[3] + ".KAM");
					 }
					 else
					 {
						 toolStripStatusLabel6.Text += (readVals[3] + ".KAM");
					 }
					 toolStripStatusLabel6.Text += ("  Полный тест");


					 do
					 {
						 startRdReg = 46;                                                     // 40046 Адрес дата/время контроллера  
						 numRdRegs = 8;
						 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
						 if ((res == BusProtocolErrors.FTALK_SUCCESS))
						 {
							 MODBUS_SUCCESS = true;
							 toolStripStatusLabel1.Text = "    MODBUS ON    ";
							 toolStripStatusLabel2.Text = (readVals[0] + "." + readVals[1] + "." + readVals[2] + "   " + readVals[3] + ":" + readVals[4] + ":" + readVals[5]);

							 TestNst = test_step[TestN];
							 if (Visual_N_Test != TestRepeatCount || TestRepeatStart == true)
							 {
                                 s_txt7 = ("\r\n   Tест № " + TestRepeatCount + "\t   Дата:  "+ DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
                                 Visual_N_Test = TestRepeatCount;
								 TestRepeatStart = false;
							 }
							 //.....
							 switch (test_step[TestN])                                              // Определить № теста
							 {

								 default:
								 case 0:
									 if (checkBoxPower.Checked || radioButton1.Checked)
                                     {
                                       //  timer1.Start();
										 All_Test_run = true;
										 test_power();
										 Thread.Sleep(200);
									 }
									 break;

								 case 1:
									 if (checkBoxSensors1.Checked || radioButton1.Checked)
									 {
                                        // timer1.Start();
										 All_Test_run = true;
										 sensor_off();
										 Thread.Sleep(200);
									 }
									 break;

								 case 2:
									 if (checkBoxSensors2.Checked || radioButton1.Checked)
                                     {
                                        // timer1.Start();
										 All_Test_run = true;
										 sensor_on();
										 Thread.Sleep(200);
									 }
									 break;
								 case 3:
									 if (checkBoxSenGar1instr.Checked || radioButton1.Checked)
                                     {
                                        // timer1.Start();
										 All_Test_run = true;
										 test_instruktora();
										 Thread.Sleep(200);
									 }
									 break;
								 case 4:
									 if (checkBoxSenGar1disp.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_dispetchera();
										 Thread.Sleep(200);
									 }
									 break;
								 case 5:
									 if (checkBoxSenTrubka.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_MTT();
										 Thread.Sleep(200);
									 }
									 break;
								 case 6:
									 if (checkBoxSenTangRuch.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_tangR();
										 Thread.Sleep(200);
									 }
									 break;
								 case 7:
									 if (checkBoxSenTangN.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_tangN();
										// Thread.Sleep(100);
									 }
									 break;
								 case 8:
									 if (checkBoxSenGGS.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 testGGS();
										 Thread.Sleep(200);
									 }
									 break;
								 case 9:
									 if (checkBoxSenGGRadio1.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_GG_Radio1();
										 Thread.Sleep(200);
									 }
									 break;
								 case 10:
									 if (checkBoxSenGGRadio2.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_GG_Radio2();
										 Thread.Sleep(200);
									 }
									 break;
								 case 11:
									 if (checkBoxSenMicrophon.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_mikrophon();
										 Thread.Sleep(200);
									 }
									 break;
								 case 12:
									 if (checkBoxDisp.Checked || radioButton1.Checked)
									 {
										 All_Test_run = true;
										 test_valueDisp();
										 Thread.Sleep(200);
									 }
									 break;
							 }

							 //.....
							 // Ожидание завершения выполнения теста
								 int count_ret = 0;
								 ushort[] waitVals = new ushort[2];
								 int waitRdRegs = 2;
								 int waitRdReg = 120;
								 do
								 {
									 res = myProtocol.readMultipleRegisters(slave, waitRdReg, waitVals, waitRdRegs);      // Ожидание кода подтверждения окончания проверки  Адрес передачи подтверждения 40120

									 if ((res == BusProtocolErrors.FTALK_SUCCESS))
									 {
										 toolStripStatusLabel1.Text = ("    MODBUS ON    " + count_ret);
										 MODBUS_SUCCESS = true;
										 count_ret++;
									 }

									 Thread.Sleep(200);
								 } while (waitVals[0] != 0);      
			 
							 // Проверка на наличие ошибок
								 startCoil = 120;                                                                        // regBank.add(120) Флаг индикации возникновения любой ошибки
								 numCoils = 2;
								 res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);                        // Проверить Адрес 120  индикации возникновения любой ошибки
								 if (coilArr[0] == true) //есть ошибка
								 {
									 // Обработка ошибки.
									 s_txt48 += ("Ошибка! " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\t");
									 s_txt8   = ("Ошибка! " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
                                     error_list();
								 }
								 Thread.Sleep(200);
								 s_txt7 += ("Выполнение команды завершено" + "\r\n");
                                 All_Test_run = false;
 								 Thread.Sleep(200);

							 TestN++;
							 if (radioButton1.Checked & TestN >= TestStep)                                    // Условие однократной проверки
							 {
								 TestN = 0;
								 e.Cancel = true;
								 return;
							 }
							 if (radioButton2.Checked & TestN >= TestStep)                                    
							 {                                                                                  // Условие многократной проверки. Повтор многократного теста
								 TestN = 0;
								 TestRepeatCount++;
								 if (TestRepeatCount > 32768) TestRepeatCount = 1;
								 Thread.Sleep(300);
							 }

							 if (MaxTestCount != 0)
							 {
								 if (TestRepeatCount > MaxTestCount)                                             // Условие заданного количества проверок
								 {
									 TestN = 0;
									 e.Cancel = true;
									 return;
								 }
							 }
						 }

						 else
						 {
							 MODBUS_SUCCESS             = false;
							 Com2_SUCCESS               = false;
							 toolStripStatusLabel1.Text = "    MODBUS ERROR(1) ";
							 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
							 Thread.Sleep(200);
							 e.Cancel = true;
							 return;
						 }

						 if (bw_DoWorkAll_Test.CancellationPending)
						 {
							 s_txt7 = "";
							 s_txt8 = "";
							 s_txt9 = "";
							 s_txt48 = "";
							 e.Cancel = true;
							 return;
						 }

						 bw_DoWorkAll_Test.ReportProgress(TestRepeatCount);
						 data += 50;
						 if (data > 1000) data = 50;
						// test_end();
					 } while (!e.Cancel);



					 e.Result = data;
				 }
				 else
				 {
					 MODBUS_SUCCESS = false;
					 toolStripStatusLabel1.Text = "    MODBUS ERROR (3) ";
					 toolStripStatusLabel4.Text = ("Ошибка получения имени файла! ");  // Обработка ошибки.
					 Thread.Sleep(2000);
				 }
			 }
			 else
			 {
				 MODBUS_SUCCESS = false;
				 toolStripStatusLabel1.Text = "    MODBUS ERROR(2) ";
				 toolStripStatusLabel4.Text = ("Команда получения имени файла не выполнена !");  // Обработка ошибки.
				 Thread.Sleep(2000);
			 }
		 }
		 private void Bw_DoWorkAll_TestRunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
		 {
			 if (e.Cancelled)   // Завершение программы
			 {
				 Thread.Sleep(300);
				 stop_test();
				 textBox7.SelectionStart = textBox7.Text.Length;
				 textBox7.ScrollToCaret();
				 textBox8.SelectionStart = textBox8.Text.Length;
				 textBox8.ScrollToCaret();
				 textBox9.SelectionStart = textBox9.Text.Length;
				 textBox9.ScrollToCaret();
				 textBox48.SelectionStart = textBox48.Text.Length;
				 textBox48.ScrollToCaret();
				 s_txt7 = "";
				 s_txt8 = "";
				 s_txt9 = "";
				 s_txt48 = "";

				 Button_All_Test_Stop = false;                                               // Признак для управления кнопкой "Стоп"
				 button9.Enabled = false;                                                    // Отключить кнопку  
				 Thread.Sleep(200);
				 label54.Visible = false;
				 groupBox19.Text = ("Ход выполнения теста ");
				 button11.Enabled = true;
				 All_Test_run = false;
				 timerVisualAll_Test.Stop();
				 startCoil = 8;                                                                       // Управление питанием платы "Камертон"
				 res = myProtocol.writeCoil(slave, startCoil, false);                                 // Выключить питание платы "Камертон"
				 start_test_modbus1();
			 }
			 else if (e.Result != null)
			 {
				 Thread.Sleep(300);
				 stop_test();
               //  timerVisualAll_Test.Stop();
				 textBox7.SelectionStart = textBox7.Text.Length;
				 textBox7.ScrollToCaret();
				 textBox8.SelectionStart = textBox8.Text.Length;
				 textBox8.ScrollToCaret();
				 textBox9.SelectionStart = textBox9.Text.Length;
				 textBox9.ScrollToCaret();
				 textBox48.SelectionStart = textBox48.Text.Length;
				 textBox48.ScrollToCaret();
				 s_txt7 = "";
				 s_txt8 = "";
				 s_txt9 = "";
				 s_txt48 = "";
				 Button_All_Test_Stop = false;                                                        // Признак для управления кнопкой "Стоп"
				 button9.Enabled = false;                                                             // Отключить кнопку  
				 Thread.Sleep(500);
				 groupBox19.Text = ("Ход выполнения теста ");
				 button11.Enabled = true;
				 timerVisualAll_Test.Stop();
				 startCoil = 8;                                                                       // Управление питанием платы "Камертон"
				 res = myProtocol.writeCoil(slave, startCoil, false);                                 // Выключить питание платы "Камертон"
				 start_test_modbus1();
			 }

		 }
		 private void Bw_DoWorkAll_TestProgressChanged(object sender, ProgressChangedEventArgs e)
		 {

			 label150.Text = toolStripStatusLabel6.Text;

			 groupBox19.Text = ("Ход выполнения теста № " + string.Format("{0}", e.ProgressPercentage));
			 if (MODBUS_SUCCESS)
			 {
				 toolStripStatusLabel1.BackColor = Color.Lime;
				 if (serial2_connect_ok == true)
				 {
					 Com2_SUCCESS = true;
				 }
			 }
			 else
			 {
				 toolStripStatusLabel1.BackColor = Color.Red;
			 }
			 if (Com2_SUCCESS)
			 {
				 toolStripStatusLabel4.BackColor = Color.Lime;
				 toolStripStatusLabel4.ForeColor = Color.Black;
			 }
			 else
			 {
				 toolStripStatusLabel4.ForeColor = Color.Red;
				 toolStripStatusLabel4.BackColor = Color.White;
			 }

		   }
		 private void start_DoWorkAll_Test()
		 {
			 timerVisualAll_Test.Start();                       // Запустить таймер вывода текстов о ходе выполнения тестов 
			 if (!_bw_All_test.IsBusy)
			 {
				 _bw_All_test.RunWorkerAsync(TestRepeatCount);    // Выполнить указанное количество тестов
			 }
		 }
		 private void stop_DoWorkAll_Test()
		 {
			 if (_bw_All_test.IsBusy)
			 {
				 _bw_All_test.CancelAsync();
                 timerVisualAll_Test.Stop();
			 }
		 }

		 void Bw_DoWorkAll_Test1(object sender, DoWorkEventArgs e)            // Индикатор выполнения теста
		 {
			 BackgroundWorker bw_DoWorkAll_Test1 = (BackgroundWorker)sender;
			 int data = 0;
			 int i = 0;
			 int iterations = (int)e.Argument + 1;

			 do
			 {

				 if (bw_DoWorkAll_Test1.CancellationPending)
				 {
					 e.Cancel = true;
				 }

				 bw_DoWorkAll_Test1.ReportProgress(i);
				 Thread.Sleep(10);
				 i++;
				 if (i > 99) i = 0;
				 data += 50;
				 if (data > 1000) data = 50;

			 } while (!e.Cancel);

			 e.Result = data;
		 }
		 private void Bw_DoWorkAll_Test1RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
		 {
			 if (e.Cancelled)
			 {
				 progressBar4.Value = progressBar4.Minimum;
			 }
			 else if (e.Result != null)
			 {
				 //label53.Text = "Выполнение задачи завершено";
			 }

		 }
		 private void Bw_DoWorkAll_Test1ProgressChanged(object sender, ProgressChangedEventArgs e)
		 {
			 progressBar4.Value = e.ProgressPercentage;
		 }
		 private void start_DoWorkAll_Test1()
		 {
			 if (!_bw_All_test1.IsBusy)
			 {
				 _bw_All_test1.RunWorkerAsync(progressBar4.Maximum);
			 }
		 }
		 private void stop_DoWorkAll_Test1()
		 {
			 if (_bw_All_test1.IsBusy)
			 {
				 _bw_All_test1.CancelAsync();
			 }
		 }

		 void Bw_set_byte(object sender, DoWorkEventArgs e)
		 {
			 timer_param_set2.Stop();
			 BackgroundWorker bw_set_byte = (BackgroundWorker)sender;
			 int iterations = (int)e.Argument + 1;
			 int i = 0;
			 int data = 0;
			 short[] readVals = new short[125];
			 toolStripStatusLabel2.Text = "";
			 byte_set_run = true;
			 coil_Button[8] = true;
			 do
             {
                 startRdReg = 46;                                                                        // 40046 Адрес дата/время контроллера  
                 numRdRegs = 6;
                 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;                                                              // Признак выполнения контроля связи MODBUS
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                     //toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5 УСТАНОВЛЕНА !");       // 
                     toolStripStatusLabel2.Text = (readVals[0] + "." + readVals[1] + "." + readVals[2] + "   " + readVals[3] + ":" + readVals[4] + ":" + readVals[5]);
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     //  Com2_SUCCESS               = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR(byte_set1) ";
                     // toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }

                 
                 if (ggs_mute_on)
                 {
                     if ((myProtocol != null))
                     {
                         startCoil = 42;                                                                  // Регистр 42 управления кнопкой 
                         res = myProtocol.writeCoil(slave, startCoil, true);                              // Включить кнопку ГГС (mute) вкл.
                         startWrReg = 120;                                                                // 
                         res = myProtocol.writeSingleRegister(slave, startWrReg, 53);                     // 
                         test_end1();
                     }
                     else
                     {
                         Com2_SUCCESS = false;
                         toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                         toolStripStatusLabel4.ForeColor = Color.Red;
                         Thread.Sleep(100);
                     }
                     ggs_mute_on = false;                                                                 // Признак включения ggs_mute
                 }
            
                 if (ggs_mute_off)
                 {
                     if ((myProtocol != null))
                     {
                         startCoil = 42;                                                                  // Регистр 42 управления кнопкой 
                         res = myProtocol.writeCoil(slave, startCoil, false);                             // Выключить кнопку ГГС (mute) выкл.
                         startWrReg = 120;                                                                // 
                         res = myProtocol.writeSingleRegister(slave, startWrReg, 53);                     // 
                         test_end1();
                     }
                     else
                     {
                         Com2_SUCCESS = false;
                         toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                         toolStripStatusLabel4.ForeColor = Color.Red;
                         Thread.Sleep(100);
                     }
                     ggs_mute_off = false;                                                                // Признак выключения ggs_mute
                 }
               if (radio_on)
                 {
                     if ((myProtocol != null))
                     {
                         startCoil = 41;                                                                  // Регистр 41 управления кнопкой 
                         res = myProtocol.writeCoil(slave, startCoil, true);                              // Включить кнопку Радиопередача
                         startWrReg = 120;                                                                // 
                         res = myProtocol.writeSingleRegister(slave, startWrReg, 52);                     // 
                         test_end1();
                     }
                     else
                     {
                         Com2_SUCCESS = false;
                         toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                         toolStripStatusLabel4.ForeColor = Color.Red;
                         Thread.Sleep(100);
                     }
                     radio_on = false;                                                                    // Признак выключения радиопередачи
                 }

                 if (radio_off)
                 {
                     if ((myProtocol != null))
                     {
                         startCoil = 41;                                                                  // Регистр 41 управления кнопкой 
                         res = myProtocol.writeCoil(slave, startCoil, false);                             // Выключить кнопку Радиопередача
                         startWrReg = 120;                                                                // 
                         res = myProtocol.writeSingleRegister(slave, startWrReg, 52);                     // 
                         test_end1();
                     }
                     else
                     {
                         Com2_SUCCESS = false;
                         toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                         toolStripStatusLabel4.ForeColor = Color.Red;
                         Thread.Sleep(100);
                     }
                     radio_off = false;
                 }

                 if(set_sound_level)
                     {
                        if ((myProtocol != null))
                        {
                            startWrReg = 60;                                                                 // 40060 Адрес хранения величины сигнала
                            res = myProtocol.writeSingleRegister(slave, startWrReg, (short)tempK_lev);       //
                            startWrReg = 120;                                                                // 
                            res = myProtocol.writeSingleRegister(slave, startWrReg, 40);                     // Установить резистором №1,№2  уровень сигнала
                            test_end1();
                        }
                       else
                        {
                            Com2_SUCCESS = false;
                            toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                            toolStripStatusLabel4.ForeColor = Color.Red;
                            Thread.Sleep(100);
                        }
                         set_sound_level = false;
                     }

                 if (set_display)
                 {
                     if ((myProtocol != null))
                     {
                         startWrReg = 61;                                                                   // 40061 Адрес хранения величины сигнала
                         res = myProtocol.writeSingleRegister(slave, startWrReg, tempK);
                         startWrReg = 120;                                                                  // 
                         res = myProtocol.writeSingleRegister(slave, startWrReg, 18);                       // 
                         test_end1();
                         startRdReg = 62;
                         numRdRegs = 2;
                         res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);    // 40062
                         if ((res == BusProtocolErrors.FTALK_SUCCESS))
                         {
                             toolStripStatusLabel1.Text = "    MODBUS ON    ";
                             toolStripStatusLabel1.BackColor = Color.Lime;
                             s1_disp = readVals[0].ToString();                                              // Преобразование числа в строку
                             if (readVals[1] != 0)
                             {
                                 s2_disp = readVals[1].ToString();                                          // Преобразование числа в строку
                                 disp_mks = true;
                             }
                             else
                             {
                                 disp_mks = false;
                             }

                         }
                         Thread.Sleep(100);
                         test_end1();
                     }
                     else
                     {
                         Com2_SUCCESS = false;
                         toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                         toolStripStatusLabel4.ForeColor = Color.Red;
                         Thread.Sleep(100);
                     }

                     set_display = false;
                 }

                 ushort[] writeVals = new ushort[20];
                 ushort[] readVolt = new ushort[10];
                 numRdRegs = 4;

                 startWrReg = 120;
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 17);                             // Провести измерение питания
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     // Com2_SUCCESS               = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set2)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }
                 test_end1();

                 // *********************** Проверить питание ********************************
                 writeVals[0] = 150;                                                  //  Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 1684;                                                 //  Адрес блока памяти  для передачи в ПК уровней порогов.
                 writeVals[2] = 10;                                                   //  Длина блока регистров для передачи в ПК уровней порогов.


                 startWrReg = 124;                                                    //  Отправить параметры блока получения данных из памяти 
                 int numWrRegs = 3;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();

                 startWrReg = 120;
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 15);        // Передать информацию в регистры
                 test_end1();

                 startRdReg = 150;
                 numRdRegs = 10;
                 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVolt, numRdRegs);
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     s12_Radio2 = readVolt[0] * 2.61 / 100;
                     s12_Power = readVolt[2] * 2.61 / 100;
                     s12_GGS = readVolt[4] * 2.61 / 100;
                     s12_Radio1 = readVolt[6] * 2.61 / 100;
                     s12_Led_Mic = readVolt[8] / 100;
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set3)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");          // Обработка ошибки.
                     e.Cancel = true;
                 }


                 //-----------------------------------------------------------------------------------------
                 //****************************** Получить состояние регистров Аудио-1 ***************************

                 byte_set_ready = false;
                 startRdReg = 1;
                 numRdRegs = 7;
                 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals_byte, numRdRegs);         //  Считать число из регистров по адресу  40001 -40007
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                     byte_set_ready = true;
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set4)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");       // Обработка ошибки.
                     e.Cancel = true;
                 }

                 // -------------------------------------------------------------------------------------------------------
                 //********************* Передать  состояние кнопок в Камертон 5.0 ********************************


                 bool[] coilArr = new bool[64];
                 startCoil = 1;
                 numCoils = 24;

                 res = myProtocol.forceMultipleCoils(slave, startCoil, coil_Button, numCoils);              // 15 (0F) Записать бит false или true  по адресу 0-9999 
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set5)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");         // Обработка ошибки.
                     e.Cancel = true;
                 }

                 //  Thread.Sleep(50);

                 for (int i_coil = 0; i_coil < 16; i_coil++)
                 {
                     coilArr[i_coil] = coil_Button[24 + i_coil];
                 }

                 startCoil = 25;
                 numCoils = 9;
                 res = myProtocol.forceMultipleCoils(slave, startCoil, coilArr, numCoils);                  // 

                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set6)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }
                 //  Thread.Sleep(50);


                 //-----------------------------------------------------------------------

                 //*********    Считывание состояния реле и оптронов  ******************

                 Status_Rele_ready = false;

                 startCoil = 1;  //  mb.Coil(1-8);   Отображение соостояния реле 1-8
                 numCoils = 8;
                 res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                     for (int i_coil = 0; i_coil < 8; i_coil++)
                     {
                         coilArr_Status_Rele[i_coil] = coilArr[i_coil];
                     }
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set7)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }
                 Thread.Sleep(50);

                 startCoil = 9;  //  regBank.add(9-16);   Отображение соостояния реле 9-16
                 numCoils = 8;
                 res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                     for (int i_coil = 0; i_coil < 8; i_coil++)
                     {
                         coilArr_Status_Rele[i_coil + 8] = coilArr[i_coil];
                     }

                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set8)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }

                 //  Thread.Sleep(50);

                 startCoil = 17;  //  regBank.add(17-24);   Отображение соостояния реле 17-24
                 numCoils = 8;
                 res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                     for (int i_coil = 0; i_coil < 8; i_coil++)
                     {
                         coilArr_Status_Rele[i_coil + 16] = coilArr[i_coil];
                     }

                 }

                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set9)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }
                 //  Thread.Sleep(50);

                 startCoil = 25;  //  regBank.add(00009-16);   Отображение соостояния реле 25-32
                 numCoils = 8;
                 res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                     for (int i_coil = 0; i_coil < 8; i_coil++)
                     {
                         coilArr_Status_Rele[i_coil + 24] = coilArr[i_coil];
                     }

                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set10)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }

                 //   Thread.Sleep(50);

                 startCoil = 81;  // Флаг 
                 numCoils = 3;
                 res = myProtocol.readInputDiscretes(slave, startCoil, coilArrFlag, numCoils);
                 if ((res == BusProtocolErrors.FTALK_SUCCESS))
                 {
                     MODBUS_SUCCESS = true;
                     toolStripStatusLabel1.Text = "    MODBUS ON    ";
                 }
                 else
                 {
                     MODBUS_SUCCESS = false;
                     Com2_SUCCESS = false;
                     toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set11)  ";
                     toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
                     e.Cancel = true;
                 }

                 //   Thread.Sleep(50);
                 Status_Rele_ready = true;
                 //----------------------------------------------------------------------------------------------------


                 if (bw_set_byte.CancellationPending)
                 {
                     e.Cancel = true;
                 }

                 bw_set_byte.ReportProgress(i);
                 i++;
                 if (i > 100) i = 0;
                 data += 50;
                 if (data > 1000) data = 50;
                 // Thread.Sleep(50);
             } while (!e.Cancel);
			 Thread.Sleep(50);
			 e.Result = data;
			 byte_set_run = false;
		 }
		 private void Bw_set_byteCompleted(object sender, RunWorkerCompletedEventArgs e)
		 {
			 if (e.Cancelled)
			 {
				 // byte_set_run = false;
				 progressBar1.Value = progressBar1.Minimum;
				 timer_byte_set.Stop();
				 if (!byte_set_run) toolStripStatusLabel8.Text = ("Проверка состояния сенсоров Аудио-1 окончена");
				 if (Tab_index == 3) toolStripStatusLabel8.Text = ("Установка параметров подключения Камертон 5.0");
			 }
			 else if (e.Result != null)
			 {
				 if (!byte_set_run) toolStripStatusLabel8.Text = ("Проверка состояния сенсоров Аудио-1 окончена");
				 if (Tab_index == 3) toolStripStatusLabel8.Text = ("Установка параметров подключения Камертон 5.0");
				 timer_byte_set.Stop();
			 }

		 }
		 private void Bw_set_byteProgressChanged(object sender, ProgressChangedEventArgs e)
		 {
			 label114.Text = string.Format("{0}%", e.ProgressPercentage);
			 progressBar1.Value = e.ProgressPercentage;
		 }
		 private void start_bw_set_byte()
		 {
			 if (serial_connect_ok)                                                                    // Признак успешного подключения serial_connect
			 {
				 button25.Enabled = true;
				 button24.Enabled = true;
				 toolStripStatusLabel8.Text = ("Проверка состояния сенсоров Аудио-1");
				 timer_byte_set.Start();
				 if (!_bw_set_byte.IsBusy)
				 {
					 _bw_set_byte.RunWorkerAsync(100);
				 }
			 }
		 }
		 private void stop_bw_set_byte()
		 {
			 if (_bw_set_byte.IsBusy)
			 {
				 _bw_set_byte.CancelAsync();
				 timer_byte_set.Stop();
				 // byte_set_run = false;
			 }
		 }
  
		 private void stop_test()
		{

			if ((myProtocol != null))
			{
				ushort[] writeVals = new ushort[20];
				bool[] coilArr = new bool[34];
				slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
				button9.BackColor = Color.LightSalmon;
				button11.BackColor = Color.Lime;
				label92.Text = ("");
				progressBar2.Value = 0;
				startWrReg = 120;
				res = myProtocol.writeSingleRegister(slave, startWrReg, 13);          // Команда на закрытие файла отправлена
				textBox9.Text += ("Команда на закрытие файла отправлена" + "\r\n");
				textBox7.Refresh();
				textBox9.Refresh();
				textBox7.Text += "Тест окончен!";
				textBox8.Text += ("\r\n" + "Тест модуля Аудио-1 окончен!   Дата " + DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
                textBox48.Text += ("\r\n" + "Тест модуля Аудио-1 окончен!   Дата " + DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
                textBox7.SelectionStart = textBox7.Text.Length;
				textBox7.ScrollToCaret();
				textBox8.SelectionStart = textBox8.Text.Length;
				textBox8.ScrollToCaret();
				textBox9.SelectionStart = textBox9.Text.Length;
				textBox9.ScrollToCaret();

				if (radioButton1.Checked)                                // Условие однократной проверки
				{
					pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
					pathString = System.IO.Path.Combine(pathString, fileName);
					File.WriteAllText(pathString, textBox48.Text, Encoding.GetEncoding("UTF-8"));
					if (File.Exists(pathString))
					{
						textBox48.Text = File.ReadAllText(pathString);
					}
					else
					{
						MessageBox.Show("Файл НЕ существует!  " + pathString, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Information);
					}
					// Сформировать файл отчета
				}
				else
				{
					pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
					System.IO.Directory.CreateDirectory(pathString);
					pathString = System.IO.Path.Combine(pathString, fileName);
					File.WriteAllText(pathString, textBox48.Text, Encoding.GetEncoding("UTF-8"));
					Read_File();                                                      // Вывести информацию из файла в окно
				}
				//startCoil = 8;                                                        // Управление питанием платы 
				//res = myProtocol.writeCoil(slave, startCoil, false);                  // Отключить питание платы 
			}

			else
			{
				textBox9.Text += ("Ошибка!  Тестируемый модуль не подключен" + "\r\n");
			}
		}
         //private void stop_test_potok()
         //{
         //    if ((myProtocol != null))
         //    {
         //        ushort[] writeVals = new ushort[20];
         //        bool[] coilArr     = new bool[34];
         //        startWrReg         = 120;
         //        res                = myProtocol.writeSingleRegister(slave, startWrReg, 13);          // Команда на закрытие файла отправлена
         //        s_txt9            += ("Команда на закрытие файла отправлена" + "\r\n");
         //        s_txt7            += "Тест окончен!";
         //        s_txt8            += ("\r\n" + "Тест модуля Аудио-1 окончен!   Дата " + DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
         //        s_txt48           += ("\r\n" + "Тест модуля Аудио-1 окончен!   Дата " + DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");

         //        if (radioButton1.Checked)                                // Условие однократной проверки
         //        {
         //            pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
         //            pathString = System.IO.Path.Combine(pathString, fileName);
         //            File.WriteAllText(pathString, textBox48.Text, Encoding.GetEncoding("UTF-8"));
         //            if (File.Exists(pathString))
         //            {
         //                textBox48.Text = File.ReadAllText(pathString);
         //            }
         //            else
         //            {
         //                MessageBox.Show("Файл НЕ существует!  " + pathString, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Information);
         //            }
         //            // Сформировать файл отчета
         //        }
         //        else
         //        {
         //            pathString = System.IO.Path.Combine(folderName, (("RusError " + DateTime.Now.ToString("yyyy.MM.dd", CultureInfo.CurrentCulture))));
         //            System.IO.Directory.CreateDirectory(pathString);
         //            pathString = System.IO.Path.Combine(pathString, fileName);
         //            File.WriteAllText(pathString, textBox8.Text, Encoding.GetEncoding("UTF-8"));
         //            Read_File();                                                      // Вывести информацию из файла в окно
         //        }
         //        startCoil = 8;                                                        // Управление питанием платы "Камертон"
         //        res = myProtocol.writeCoil(slave, startCoil, false);                  // Отключить питание платы "Камертон"
         //    }
         //    else
         //    {
         //        s_txt9 += ("Ошибка!  Тестируемый модуль не подключен" + "\r\n");
         //    }


         //}

		 private void button15_Click(object sender, EventArgs e)               // Test EEPROM
		 {
			 stop_test_modbus1();                                                       // Отключить сканирование MODBUS 
			 Thread.Sleep(400);
			 startWrReg = 120;                                                          // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 33);               //  Записать последовательность чисел в EEPROM Камертон 5.0
			 Thread.Sleep(400);
			 timer_param_set.Start();
		 }
		 private void button31_Click(object sender, EventArgs e)               // Кнопка "Просмотр EEPROM"
		 {
			 richTextBox2.Text = "";
			 for (int i = 0; i < 267; i++)                                              // Чтение блока регистров 
			 {
				 richTextBox2.Text += (i+201 + " - " + table_porog[i].ToString() + "\r\n");
			 }
		 }
		 private void button56_Click(object sender, EventArgs e)
			 {
				 ushort.Parse(textBox6.Text);
				 ushort start_mem = Convert.ToUInt16(textBox6.Text);
				 ushort.Parse(textBox3.Text);
				 ushort end_mem = Convert.ToUInt16(textBox3.Text);

				 ushort[] writeVals = new ushort[20];
			   
				 int numWrRegs      = 2;
				 writeVals[0] = start_mem;                                            //  Адрес блока регистров для передачи в ПК уровней порогов.
				 writeVals[1] = end_mem;                                              //  Адрес блока памяти  для передачи в ПК уровней порогов.

				 startWrReg = 124;                                                    //  Отправить параметры блока получения данных из памяти 
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 test_end1();
				 startWrReg = 120;                                                    // адрес блока команд
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 32);         // Команда - Передать информацию в регистры
				 test_end1();

			 }
		 private void button93_Click(object sender, EventArgs e)               // Кнопка "Чтение EEPROM"
			 {
				 ushort.Parse(textBox1.Text);
				 ushort adr_mem = Convert.ToUInt16(textBox1.Text);
				 ushort.Parse(textBox2.Text);
				 ushort blok_mem = Convert.ToUInt16(textBox2.Text);
				 error_list1(adr_mem, blok_mem);                                   // 1- Блок памяти, 2 - Количесво блоков по 16 адресов памяти
			 }

		 private void button94_Click(object sender, EventArgs e)               // Кнопка "По умолчанию"  таблицу По умолчанию  
			 {
				 button94.Enabled = false;
				 button94.BackColor = Color.LightSalmon;
				 button94.Text = "Загрузка";
				 button94.Refresh();
				 Thread.Sleep(400);
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 20);      //  Записать последовательность чисел в EEPROM Камертон 5.0
				 Thread.Sleep(400);
				 test_end1();
				 get_porog_memory();                                               // Получить уровни порогов из Камертон 5.0
				 instruk_table1();                                                 // Отобразить таблицу порогов  инструктора
				 dispatch_table();                                                 // Отобразить таблицу порогов диспетчера
				 mtt_table();                                                      // Отобразить таблицу порогов MTT
				 mic_table();                                                      // Отобразить таблицу порогов микрофона
				 ggs_table();                                                      // Отобразить таблицу порогов GGS
				 radio1_table();                                                   // Отобразить таблицу порогов Radio1
				 radio2_table();              
				 button94.Enabled = true;
				 button94.BackColor = Color.LightGreen;
				 button94.Text = "По умолчанию";
				 button94.Refresh();
			 }
		 private void button95_Click(object sender, EventArgs e)               // Кнопка "Сохранить"  таблицу инструктора
			 {

			 if(Insrt_Table_Read)
				 {
					 button95.Enabled = false;
					 button95.BackColor = Color.LightSalmon;
					 button95.Text = "Загрузка";
					 button95.Refresh();

					 ushort[] writeVals = new ushort[200];
					 ushort[] readVals = new ushort[125];
					 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
					 int res;
					 int startWrReg;
					 int numWrRegs;

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                            // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 0;                                              // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 20;                                             // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                              // 40130 Адрес дата 
					 int step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 // Отправить команду получения данных из памяти

					 for(int i_mem = 0; i_mem < 10; i_mem++)
						 {
							 writeVals[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[0].Value;
							 step_mem++;
							 writeVals[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[1].Value;
							 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 20;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 40;                                                // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                                 // 40130 Адрес дата 
					 step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                    //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

					 for(int i_mem = 0; i_mem < 9; i_mem++)
						 {
						 writeVals[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[0].Value;
						 step_mem++;
						 writeVals[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[1].Value;
						 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 18;                                                  //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 button95.Enabled = true;
					 button95.BackColor = Color.LightGreen;
					 button95.Text = "Сохранить";
					 button95.Refresh();
				 }
			 else
				 {
					  MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				 }
			 }
		 private void button96_Click(object sender, EventArgs e)               // Кнопка "Сохранить"  таблицу диспетчера
			 {

				 if (Disp_Table_Read)
				 {
					 button96.Enabled = false;
					 button96.BackColor = Color.LightSalmon;
					 button96.Text = "Загрузка";
					 button96.Refresh();

					 ushort[] writeVals = new ushort[200];
					 ushort[] readVals = new ushort[125];
					 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
					 int res;
					 int startWrReg;
					 int numWrRegs;

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 76;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                                 // 40130 Адрес дата 
					 int step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                    //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 // Отправить команду получения данных из памяти

					 for(int i_mem = 0; i_mem < 10; i_mem++)
						 {
							 writeVals[step_mem] = (ushort)dataGridViewParentForm3.Rows[i_mem].Cells[0].Value;
							 step_mem++;
							 writeVals[step_mem] = (ushort)dataGridViewParentForm3.Rows[i_mem].Cells[1].Value;
							 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 20;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 116;                                           // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                                 // 40130 Адрес дата 
					 step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                    //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

					 for(int i_mem = 0; i_mem < 9; i_mem++)
						 {
							 writeVals[step_mem] = (ushort)dataGridViewParentForm3S.Rows[i_mem].Cells[0].Value;
							 step_mem++;
							 writeVals[step_mem] = (ushort)dataGridViewParentForm3S.Rows[i_mem].Cells[1].Value;
							 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 18;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 button96.Enabled = true;
					 button96.BackColor = Color.LightGreen;
					 button96.Text = "Сохранить";
					 button96.Refresh();
				 }
			 else
				 {
				 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				 }

			  }
		 private void button97_Click(object sender, EventArgs e)               // Кнопка "Сохранить"  таблицу МТТ
			 {

			 if(Mtt_Table_Read)
				 {
					 button97.Enabled = false;
					 button97.BackColor = Color.LightSalmon;
					 button97.Text = "Загрузка";
					 button97.Refresh();

					 ushort[] writeVals = new ushort[200];
					 ushort[] readVals = new ushort[125];
					 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
					 int res;
					 int startWrReg;
					 int numWrRegs;

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 152;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                                 // 40130 Адрес дата 
					 int step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                    //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 // Отправить команду получения данных из памяти

					 for(int i_mem = 0; i_mem < 10; i_mem++)
						 {
						 writeVals[step_mem] = (ushort)dataGridViewParentForm4.Rows[i_mem].Cells[0].Value;
						 step_mem++;
						 writeVals[step_mem] = (ushort)dataGridViewParentForm4.Rows[i_mem].Cells[1].Value;
						 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 20;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 192;                                               // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                                 // 40130 Адрес дата 
					 step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                    //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

					 for(int i_mem = 0; i_mem < 9; i_mem++)
						 {
						 writeVals[step_mem] = (ushort)dataGridViewParentForm4S.Rows[i_mem].Cells[0].Value;
						 step_mem++;
						 writeVals[step_mem] = (ushort)dataGridViewParentForm4S.Rows[i_mem].Cells[1].Value;
						 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 18;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 button97.Enabled = true;
					 button97.BackColor = Color.LightGreen;
					 button97.Text = "Сохранить";
					 button97.Refresh();
				 }
			 else
				 {
					 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				 }
			 }
		 private void button98_Click(object sender, EventArgs e)               // Кнопка "Сохранить"  таблицу микрофон
			 {

				 if (Mic_Table_Read)
				 {
					 button98.Enabled = false;
					 button98.BackColor = Color.LightSalmon;
					 button98.Text = "Загрузка";
					 button98.Refresh();

					 ushort[] writeVals = new ushort[200];
					 ushort[] readVals = new ushort[125];
					 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
					 int res;
					 int startWrReg;
					 int numWrRegs;

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 228;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                                 // 40130 Адрес дата 
					 int step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                    //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 // Отправить команду получения данных из памяти

					 for(int i_mem = 0; i_mem < 10; i_mem++)
						 {
						 writeVals[step_mem] = (ushort)dataGridViewParentForm5.Rows[i_mem].Cells[0].Value;
						 step_mem++;
						 writeVals[step_mem] = (ushort)dataGridViewParentForm5.Rows[i_mem].Cells[1].Value;
						 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 20;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 // Подготовка параметров для считывания данных из памяти EEPROM
					 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 268;                                               // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                                 // 40130 Адрес дата 
					 step_mem = 0;

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                    //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

					 for(int i_mem = 0; i_mem < 9; i_mem++)
						 {
						 writeVals[step_mem] = (ushort)dataGridViewParentForm5S.Rows[i_mem].Cells[0].Value;
						 step_mem++;
						 writeVals[step_mem] = (ushort)dataGridViewParentForm5S.Rows[i_mem].Cells[1].Value;
						 step_mem++;
						 }

					 // Отправить параметры блока получения данных из памяти 
					 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 18;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 button98.Enabled = true;
					 button98.BackColor = Color.LightGreen;
					 button98.Text = "Сохранить";
					 button98.Refresh();
				 }
			 else
				 {
				 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				 }
			 }
		 private void button99_Click(object sender, EventArgs e)               // Кнопка "Сохранить"  таблицу ГГС
			 {
				 if (Ggs_Table_Read)
				 {
				 button99.Enabled = false;
				 button99.BackColor = Color.LightSalmon;
				 button99.Text = "Загрузка";
				 button99.Refresh();

				 ushort[] writeVals = new ushort[200];
				 ushort[] readVals = new ushort[125];
				 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
				 int res;
				 int startWrReg;
				 int numWrRegs;

				 // Подготовка параметров для считывания данных из памяти EEPROM
				 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
				 writeVals[1] = 304;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
				 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
				 startRdReg = 130;                                                 // 40130 Адрес дата 
				 int step_mem = 0;

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 3;                                                    //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 // Отправить команду получения данных из памяти

				 for(int i_mem = 0; i_mem < 10; i_mem++)
					 {
					 writeVals[step_mem] = (ushort)dataGridViewParentForm6.Rows[i_mem].Cells[0].Value;
					 step_mem++;
					 writeVals[step_mem] = (ushort)dataGridViewParentForm6.Rows[i_mem].Cells[1].Value;
					 step_mem++;
					 }

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 20;                                                   //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 test_end1();
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
				 test_end1();

				 // Подготовка параметров для считывания данных из памяти EEPROM
				 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
				 writeVals[1] = 344;                                           // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
				 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
				 startRdReg = 130;                                                 // 40130 Адрес дата 
				 step_mem = 0;

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 3;                                                    //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

				 for(int i_mem = 0; i_mem < 9; i_mem++)
					 {
					 writeVals[step_mem] = (ushort)dataGridViewParentForm6S.Rows[i_mem].Cells[0].Value;
					 step_mem++;
					 writeVals[step_mem] = (ushort)dataGridViewParentForm6S.Rows[i_mem].Cells[1].Value;
					 step_mem++;
					 }

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 18;                                                   //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 test_end1();
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
				 test_end1();

				 button99.Enabled = true;
				 button99.BackColor = Color.LightGreen;
				 button99.Text = "Сохранить";
				 button99.Refresh();
				 }
			 else
				 {
				 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				 }
			 }
		 private void button100_Click(object sender, EventArgs e)              // Кнопка "Сохранить"  таблицу Радио1
			 {

				 if (Rad1_Table_Read)
				 {
				 button100.Enabled = false;
				 button100.BackColor = Color.LightSalmon;
				 button100.Text = "Загрузка";
				 button100.Refresh();

				 ushort[] writeVals = new ushort[200];
				 ushort[] readVals = new ushort[125];
				 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
				 int res;
				 int startWrReg;
				 int numWrRegs;

				 // Подготовка параметров для считывания данных из памяти EEPROM
				 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
				 writeVals[1] = 380;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
				 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
				 startRdReg = 130;                                                 // 40130 Адрес дата 
				 int step_mem = 0;

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 3;                                                    //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 // Отправить команду получения данных из памяти

				 for(int i_mem = 0; i_mem < 10; i_mem++)
					 {
					 writeVals[step_mem] = (ushort)dataGridViewParentForm7.Rows[i_mem].Cells[0].Value;
					 step_mem++;
					 writeVals[step_mem] = (ushort)dataGridViewParentForm7.Rows[i_mem].Cells[1].Value;
					 step_mem++;
					 }

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 20;                                                   //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 test_end1();
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
				 test_end1();

				 // Подготовка параметров для считывания данных из памяти EEPROM
				 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
				 writeVals[1] = 420;                                           // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
				 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
				 startRdReg = 130;                                                 // 40130 Адрес дата 
				 step_mem = 0;

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 3;                                                    //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

				 for(int i_mem = 0; i_mem < 9; i_mem++)
					 {
					 writeVals[step_mem] = (ushort)dataGridViewParentForm7S.Rows[i_mem].Cells[0].Value;
					 step_mem++;
					 writeVals[step_mem] = (ushort)dataGridViewParentForm7S.Rows[i_mem].Cells[1].Value;
					 step_mem++;
					 }

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 18;                                                   //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 test_end1();
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
				 test_end1();

				 button100.Enabled = true;
				 button100.BackColor = Color.LightGreen;
				 button100.Text = "Сохранить";
				 button100.Refresh();
				 }
			 else
				 {
					MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				 }
			 }
		 private void button101_Click(object sender, EventArgs e)              // Кнопка "Сохранить"  таблицу Радио2
			 {

				 if (Rad2_Table_Read)
				 {
				 button101.Enabled = false;
				 button101.BackColor = Color.LightSalmon;
				 button101.Text = "Загрузка";
				 button101.Refresh();

				 ushort[] writeVals = new ushort[200];
				 ushort[] readVals = new ushort[125];
				 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
				 int res;
				 int startWrReg;
				 int numWrRegs;

				 // Подготовка параметров для считывания данных из памяти EEPROM
				 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
				 writeVals[1] = 456;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
				 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
				 startRdReg = 130;                                                 // 40130 Адрес дата 
				 int step_mem = 0;

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 3;                                                    //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 // Отправить команду получения данных из памяти

				 for(int i_mem = 0; i_mem < 10; i_mem++)
					 {
					 writeVals[step_mem] = (ushort)dataGridViewParentForm8.Rows[i_mem].Cells[0].Value;
					 step_mem++;
					 writeVals[step_mem] = (ushort)dataGridViewParentForm8.Rows[i_mem].Cells[1].Value;
					 step_mem++;
					 }

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 20;                                                   //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 test_end1();
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
				 test_end1();

				 // Подготовка параметров для считывания данных из памяти EEPROM
				 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
				 writeVals[1] = 496;                                               // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
				 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
				 startRdReg = 130;                                                 // 40130 Адрес дата 
				 step_mem = 0;

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 3;                                                    //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

				 for(int i_mem = 0; i_mem < 9; i_mem++)
					 {
					 writeVals[step_mem] = (ushort)dataGridViewParentForm8S.Rows[i_mem].Cells[0].Value;
					 step_mem++;
					 writeVals[step_mem] = (ushort)dataGridViewParentForm8S.Rows[i_mem].Cells[1].Value;
					 step_mem++;
					 }

				 // Отправить параметры блока получения данных из памяти 
				 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
				 numWrRegs = 18;                                                   //
				 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
				 test_end1();
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
				 test_end1();

				 button101.Enabled = true;
				 button101.BackColor = Color.LightGreen;
				 button101.Text = "Сохранить";
				 button101.Refresh();
				 }
			 else
				 {
					  MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
				 }
			 }

         private void button103_Click(object sender, EventArgs e)              // Кнопка "Сохранить"  таблицу в Камертон 5.0
         {
            button103.Enabled = false;
            button103.BackColor = Color.LightSalmon;
            button103.Text = "Загрузка";
            button103.Refresh();
            if (All_Table_Read)
            {
                save_tab_instr();
                save_tab_disp();
                save_tab_mtt();
                save_tab_mic();
                save_tab_ggs();
                save_tab_radio1();
                save_tab_radio2();
            }
            else
            {
                MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            button103.Enabled = true;
            button103.BackColor = Color.LightGreen;
            button103.Text = "Сохранить в Камертон 5.0";
            button103.Refresh();
         }

         private void save_tab_instr()
         {
        
           //  if (dataGridViewParentForm2.Rows[2].Cells[1].Displayed == true)
  
                 if (Insrt_Table_Read)
                 {
                     button95.Enabled = false;
                     button95.BackColor = Color.LightSalmon;
                     button95.Text = "Загрузка";
                     button95.Refresh();

                     ushort[] writeVals = new ushort[200];
                     ushort[] readVals = new ushort[125];
                     slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                     int res;
                     int startWrReg;
                     int numWrRegs;

                     // Подготовка параметров для считывания данных из памяти EEPROM
                     writeVals[0] = 130;                                            // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                     writeVals[1] = 0;                                              // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                     writeVals[2] = 20;                                             // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                     startRdReg = 130;                                              // 40130 Адрес дата 
                     int step_mem = 0;

                     // Отправить параметры блока получения данных из памяти 
                     startWrReg = 127;                                                //  Отправить параметры блока получения данных из памяти 
                     numWrRegs = 3;                                                   //
                     res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                     // Отправить команду получения данных из памяти

                     for (int i_mem = 0; i_mem < 10; i_mem++)
                     {
                         writeVals[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[0].Value;
                         step_mem++;
                         writeVals[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[1].Value;
                         step_mem++;
                     }

                     // Отправить параметры блока получения данных из памяти 
                     startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                     numWrRegs = 20;                                                   //
                     res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                     test_end1();
                     startWrReg = 120;                                                 // 
                     res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                     test_end1();

                     // Подготовка параметров для считывания данных из памяти EEPROM
                     writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                     writeVals[1] = 40;                                                // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                     writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                     startRdReg = 130;                                                 // 40130 Адрес дата 
                     step_mem = 0;

                     // Отправить параметры блока получения данных из памяти 
                     startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                     numWrRegs = 3;                                                    //
                     res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

                     for (int i_mem = 0; i_mem < 9; i_mem++)
                     {
                         writeVals[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[0].Value;
                         step_mem++;
                         writeVals[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[1].Value;
                         step_mem++;
                     }

                     // Отправить параметры блока получения данных из памяти 
                     startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                     numWrRegs = 18;                                                  //
                     res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                     test_end1();
                     startWrReg = 120;                                                 // 
                     res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                     test_end1();

                     button95.Enabled = true;
                     button95.BackColor = Color.LightGreen;
                     button95.Text = "Сохранить";
                     button95.Refresh();
                 }
                 else
                 {
                     MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
                 }
         }
         private void save_tab_disp()
         {
             if (Disp_Table_Read)
             {
                 button96.Enabled = false;
                 button96.BackColor = Color.LightSalmon;
                 button96.Text = "Загрузка";
                 button96.Refresh();

                 ushort[] writeVals = new ushort[200];
                 ushort[] readVals = new ushort[125];
                 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                 int res;
                 int startWrReg;
                 int numWrRegs;

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 76;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 int step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 // Отправить команду получения данных из памяти

                 for (int i_mem = 0; i_mem < 10; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm3.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm3.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 20;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 116;                                           // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

                 for (int i_mem = 0; i_mem < 9; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm3S.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm3S.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 18;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 button96.Enabled = true;
                 button96.BackColor = Color.LightGreen;
                 button96.Text = "Сохранить";
                 button96.Refresh();
             }
             else
             {
                 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
             }
         }
         private void save_tab_mtt()
         {
             if (Mtt_Table_Read)
             {
                 button97.Enabled = false;
                 button97.BackColor = Color.LightSalmon;
                 button97.Text = "Загрузка";
                 button97.Refresh();

                 ushort[] writeVals = new ushort[200];
                 ushort[] readVals = new ushort[125];
                 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                 int res;
                 int startWrReg;
                 int numWrRegs;

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 152;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 int step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 // Отправить команду получения данных из памяти

                 for (int i_mem = 0; i_mem < 10; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm4.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm4.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 20;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 192;                                               // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

                 for (int i_mem = 0; i_mem < 9; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm4S.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm4S.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 18;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 button97.Enabled = true;
                 button97.BackColor = Color.LightGreen;
                 button97.Text = "Сохранить";
                 button97.Refresh();
             }
             else
             {
                 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
             }
         }
         private void save_tab_mic()
         {
             if (Mic_Table_Read)
             {
                 button98.Enabled = false;
                 button98.BackColor = Color.LightSalmon;
                 button98.Text = "Загрузка";
                 button98.Refresh();

                 ushort[] writeVals = new ushort[200];
                 ushort[] readVals = new ushort[125];
                 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                 int res;
                 int startWrReg;
                 int numWrRegs;

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 228;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 int step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 // Отправить команду получения данных из памяти

                 for (int i_mem = 0; i_mem < 10; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm5.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm5.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 20;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 268;                                               // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

                 for (int i_mem = 0; i_mem < 9; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm5S.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm5S.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 18;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 button98.Enabled = true;
                 button98.BackColor = Color.LightGreen;
                 button98.Text = "Сохранить";
                 button98.Refresh();
             }
             else
             {
                 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
             }
         }
         private void save_tab_ggs()
         {
             if (Ggs_Table_Read)
             {
                 button99.Enabled = false;
                 button99.BackColor = Color.LightSalmon;
                 button99.Text = "Загрузка";
                 button99.Refresh();

                 ushort[] writeVals = new ushort[200];
                 ushort[] readVals = new ushort[125];
                 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                 int res;
                 int startWrReg;
                 int numWrRegs;

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 304;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 int step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 // Отправить команду получения данных из памяти

                 for (int i_mem = 0; i_mem < 10; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm6.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm6.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 20;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 344;                                           // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

                 for (int i_mem = 0; i_mem < 9; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm6S.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm6S.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 18;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 button99.Enabled = true;
                 button99.BackColor = Color.LightGreen;
                 button99.Text = "Сохранить";
                 button99.Refresh();
             }
             else
             {
                 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
             }
         }
         private void save_tab_radio1()
         {
             if (Rad1_Table_Read)
             {
                 button100.Enabled = false;
                 button100.BackColor = Color.LightSalmon;
                 button100.Text = "Загрузка";
                 button100.Refresh();

                 ushort[] writeVals = new ushort[200];
                 ushort[] readVals = new ushort[125];
                 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                 int res;
                 int startWrReg;
                 int numWrRegs;

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 380;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 int step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 // Отправить команду получения данных из памяти

                 for (int i_mem = 0; i_mem < 10; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm7.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm7.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 20;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 420;                                           // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

                 for (int i_mem = 0; i_mem < 9; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm7S.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm7S.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 18;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 button100.Enabled = true;
                 button100.BackColor = Color.LightGreen;
                 button100.Text = "Сохранить";
                 button100.Refresh();
             }
             else
             {
                 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
             }
         }
         private void save_tab_radio2()
         {
             if (Rad2_Table_Read)
             {
                 button101.Enabled = false;
                 button101.BackColor = Color.LightSalmon;
                 button101.Text = "Загрузка";
                 button101.Refresh();

                 ushort[] writeVals = new ushort[200];
                 ushort[] readVals = new ushort[125];
                 slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                 int res;
                 int startWrReg;
                 int numWrRegs;

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 456;                                                // 76  - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 20;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 int step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 // Отправить команду получения данных из памяти

                 for (int i_mem = 0; i_mem < 10; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm8.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm8.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 20;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 // Подготовка параметров для считывания данных из памяти EEPROM
                 writeVals[0] = 130;                                               // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
                 writeVals[1] = 496;                                               // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
                 writeVals[2] = 18;                                                // 18  - Длина блока регистров для передачи в ПК уровней порогов.
                 startRdReg = 130;                                                 // 40130 Адрес дата 
                 step_mem = 0;

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 127;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 3;                                                    //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

                 for (int i_mem = 0; i_mem < 9; i_mem++)
                 {
                     writeVals[step_mem] = (ushort)dataGridViewParentForm8S.Rows[i_mem].Cells[0].Value;
                     step_mem++;
                     writeVals[step_mem] = (ushort)dataGridViewParentForm8S.Rows[i_mem].Cells[1].Value;
                     step_mem++;
                 }

                 // Отправить параметры блока получения данных из памяти 
                 startWrReg = 130;                                                 //  Отправить параметры блока получения данных из памяти 
                 numWrRegs = 18;                                                   //
                 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                 test_end1();
                 startWrReg = 120;                                                 // 
                 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
                 test_end1();

                 button101.Enabled = true;
                 button101.BackColor = Color.LightGreen;
                 button101.Text = "Сохранить";
                 button101.Refresh();
             }
             else
             {
                 MessageBox.Show("Таблица уровней порогов НЕ ЗАГРУЖЕНА!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
             }
         }
         private void save_tab_array()
         {
             int step_mem = 0;
             for (int i_mem = 0; i_mem < 10; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm2.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 9; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm2S.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }


             for (int i_mem = 0; i_mem < 10; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm3.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm3.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 9; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm3S.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm3S.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }


             for (int i_mem = 0; i_mem < 10; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm4.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm4.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 9; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm4S.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm4S.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 10; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm5.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm5.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 9; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm5S.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm5S.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 10; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm6.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm6.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 9; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm6S.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm6S.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 10; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm7.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm7.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 9; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm7S.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm7S.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 10; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm8.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm8.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
             for (int i_mem = 0; i_mem < 9; i_mem++)
             {
                 table_porog[step_mem] = (ushort)dataGridViewParentForm8S.Rows[i_mem].Cells[0].Value;
                 step_mem++;
                 table_porog[step_mem] = (ushort)dataGridViewParentForm8S.Rows[i_mem].Cells[1].Value;
                 step_mem++;
             }
         }



#region oscilloscope

		 private void button35_Click(object sender, EventArgs e)                    // Записать номер аналогово канала
			 {
				 /*
					КОМАНДА 34   
				   * comboBox3
				   * regBank.get(40040)
				   0 - FrontL    A10
				   1 - FrontR    A11
				   2 - LineL     A8channel
				   3 - LineR     A9
				   4 - mag/radio A3
				   5 - mag/phone A4
				   6 - ГГС       A7
				   7 - Радио1    A5
				   8 - Радио2    A6
				   9 - W         A12
				 */

				 ushort  osc_channel = 0;
				 switch(comboBox2.SelectedIndex)
					 {
					 default:
					 case 0:
						 osc_channel = 10;
						 break;
					 case 1:
						 osc_channel = 11;
						 break;
					 case 2:
						 osc_channel = 8;
						 break;
					 case 3:
						 osc_channel = 9;
						 break;
					 case 4:
						 osc_channel = 3;
						 break;
					 case 5:
						 osc_channel = 4;
						 break;
					 case 6:
						 osc_channel = 7;
						 break;
					 case 7:
						 osc_channel = 5;
						 break;
					 case 8:
						 osc_channel = 6;
						 break;
					 case 9:
						 osc_channel = 12;
						 break;
					 }

				 startWrReg = 40;                                                          // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, osc_channel);       //  Отправить номер аналогового канала Камертон 5.0
				 //  Записать номер аналогового канала Камертон 5.0
				 startWrReg = 120;                                                         // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 34);              //  Записать номер аналогового канала Камертон 5.0

			 }

		 private void button55_Click(object sender, EventArgs e)                    // oscilloscope1.ShowScope();
			 {
				 Oscilloscope_run = true;
				 oscilloscope1.Caption = "Kamerton 5.0";

				 sampTrigThreshold1 = Convert.ToByte(cTrigLevel1.Value);
				 startWrReg = 60;                                                                // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, sampTrigThreshold1);    //  Записать уровень сигнала №1 в Камертон 5.0
				 startWrReg = 120;                                                               // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 38);                    //  Выполнить запись уровень сигнала №1 в Камертон 5.0
			   
				 sampTrigThreshold2 = Convert.ToByte(cTrigLevel2.Value);
				 startWrReg = 64;                                                                // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, sampTrigThreshold2);    //  Записать уровень сигнала №2 в Камертон 5.0
				 startWrReg = 120;                                                               // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 39);                    //  Выполнить запись уровень сигнала №2 в Камертон 5.0
				 check_sound();
				 startCoil = 8;                                                                  // Управление питанием платы "Камертон"
				 res = myProtocol.writeCoil(slave, startCoil, true);                             // Включить питание платы "Камертон"
				//stop_test_modbus1();                                                           // Отключить сканирование MODBUS 
				 startCoil = 40;
				 res = myProtocol.writeCoil(slave, startCoil, true);                             // Отправить разрешение на измерение
				 Thread.Sleep(400);
				 startWrReg = 120;                                                               // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 35);                    // Включить осциллограф
				 oscilloscope1.ShowScope();
				 oscilloscope1.UpdateScope();
				 oscilloscope1.ClearScope();
				 oscilloscope1.CellSize = 10;
				 oscilloscope1.VerticalOffset1 = -500;
				 oscilloscope1.AmplitudeScale1 = 50;
				 oscilloscope1.Left = loc_x;
				 oscilloscope1.Top = loc_y;

				 
			 }

		 private void button36_Click(object sender, EventArgs e)                    // oscilloscope1.HideScope();
			 {
 
			   //stop_test_modbus1();                                               // Отключить сканирование MODBUS 
				 startCoil = 40;
				 res = myProtocol.writeCoil(slave, startCoil, false);               // Прекратить измерение осциллографом
				 Thread.Sleep(200);
				 Oscilloscope_run = false;
				 Thread.Sleep(100);
				 oscilloscope1.HideScope();
				 timerPowerOff.Start();
			 }
		 private void check_sound()
		{
			ushort sound_n = 0;
			if (radioButton3.Checked)
			{
				sound_n = 0;
			}
			else if (radioButton4.Checked)
			{
				sound_n = 1;
			}
			else if (radioButton5.Checked)
			{
				sound_n = 2;
			}
			else if (radioButton6.Checked)
			{
				sound_n = 3;
			}
			else if (radioButton7.Checked)
			{
				sound_n = 4;
			}
			else if (radioButton8.Checked)
			{
				sound_n = 5;
			}
			else if (radioButton9.Checked)
			{
				sound_n = 6;
			}
			else if (radioButton10.Checked)
			{
				sound_n = 7;
			}
	   
			startWrReg = 41;                                                          // 
			res = myProtocol.writeSingleRegister(slave, startWrReg,  sound_n);        //  Записать номер частоты в Камертон 5.0
			startWrReg = 120;                                                         // 
			res = myProtocol.writeSingleRegister(slave, startWrReg, 37);              //  Выполнить запись номера частоты в Камертон 5.0

		}

		 private void cTrigLevel1_Scroll(object sender, EventArgs e)
			 {
				 sampTrigThreshold1 = Convert.ToByte(cTrigLevel1.Value);
				// trackChange1 = true;
				 label101.Text = string.Format("{0:0}%", sampTrigThreshold1/2.55);
				 label101.Refresh();
				 label134.Text = string.Format("{0:0}", sampTrigThreshold1 );
				 label134.Refresh();
				 startWrReg = 60;                                                                // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, sampTrigThreshold1);    //  Записать уровень сигнала №1 в Камертон 5.0
				 startWrReg = 120;                                                               // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 38);                    //  Выполнить запись уровень сигнала №1 в Камертон 5.0
			 }

		 private void cTrigLevel2_Scroll(object sender, EventArgs e)
			 {
				 sampTrigThreshold2 = Convert.ToByte(cTrigLevel2.Value);
				// trackChange2 = true;
				 label103.Text = string.Format("{0:0}%", sampTrigThreshold2/2.55);
				 label103.Refresh();
				 label136.Text = string.Format("{0:0}", sampTrigThreshold2 );
				 label136.Refresh();
				 startWrReg = 64;                                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, sampTrigThreshold2);     //  Записать уровень сигнала №2 в Камертон 5.0
				 startWrReg = 120;                                                                // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 39);                     //  Выполнить запись уровень сигнала №2 в Камертон 5.0
			 }
		
		 private void radioButton3_CheckedChanged(object sender, EventArgs e)   // 500 - 1000
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 0);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void radioButton4_CheckedChanged(object sender, EventArgs e)   // 500 - 3500
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 1);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void radioButton5_CheckedChanged(object sender, EventArgs e)   // 100 - 5500
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 2);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void radioButton6_CheckedChanged(object sender, EventArgs e)   // 100 - 1000
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 3);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void radioButton7_CheckedChanged(object sender, EventArgs e)   // 500
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 4);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void radioButton8_CheckedChanged(object sender, EventArgs e)   // 1000
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 5);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void radioButton9_CheckedChanged(object sender, EventArgs e)   // 2000
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 6);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void radioButton10_CheckedChanged(object sender, EventArgs e)  // 3500
		 {
			 startWrReg = 41;                                                   // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 7);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                  // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);       //  Выполнить запись номера частоты в Камертон 5.0
		 }

		 private void set_sound(ushort sound_chanal)
		 {
			 startWrReg = 41;                                                              // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, sound_chanal);        //  Записать номер частоты в Камертон 5.0
			 startWrReg = 120;                                                             // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 37);                  //  Выполнить запись номера частоты в Камертон 5.0
		 }
		 private void set_chanal(ushort in_chanal)
		 {
			 for (int i = 1; i < 28; i++)
			 {
				 coil_Button[i] = false;
			 }
			 Thread.Sleep(100);
			 switch (in_chanal)
			 {
				 default:


				 case 0:
					 coil_Button[8] = true;           // Включить питание +12в.
					 coil_Button[2] = true;   // Микрофон инструктора
					 coil_Button[5] = true;
					 coil_Button[27] = true;
					 break;
				 case 1:
					 coil_Button[8] = true;           // Включить питание +12в.
					 coil_Button[1] = true;  // Микрофон диспетчера
					 coil_Button[10] = true;
					 break;
				 case 2:
					 coil_Button[8] = true;           // Включить питание +12в.
					 coil_Button[3] = true;   // Микрофон трубка
					 break;
				 case 3:
					 coil_Button[8] = true;           // Включить питание +12в.
					 coil_Button[9] = true;   // Микрофон 
					 coil_Button[3] = true;
					 coil_Button[16] = true;
					 break;
				 case 4:
					 coil_Button[8] = true;           // Включить питание +12в.
					 coil_Button[6] = true;   // ГГС
					 break;
				 case 5:
					 coil_Button[8] = true;           // Включить питание +12в.
					 coil_Button[4] = true;   // Радио 1
					 break;
				 case 6:
					 coil_Button[8] = true;           // Включить питание +12в.
					 coil_Button[7] = true;   // Радио 2
					 break;
				 case 7:
					 for (int i = 1; i < 28; i++)
					 {
						 coil_Button[i] = false;
					 }
					 break;

			 }
			 Thread.Sleep(100);
			 startCoil = 1;
			 numCoils = 28;

			 res = myProtocol.forceMultipleCoils(slave, startCoil, coil_Button, numCoils);               // 15 (0F) Записать бит false или true  по адресу 0-9999 
			 if ((res == BusProtocolErrors.FTALK_SUCCESS))
			 {
				 MODBUS_SUCCESS = true;
				 toolStripStatusLabel1.Text = "    MODBUS ON    ";
				 Thread.Sleep(200);
				 startWrReg = 120;                                                 // 
				 res = myProtocol.writeSingleRegister(slave, startWrReg, 44);      //  Записать блок в EEPROM Камертон 5.0
			  //   test_end1();
				  

			 }
			 else
			 {
				 MODBUS_SUCCESS             = false;
				 Com2_SUCCESS               = false;
				 toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set5)  ";
				 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");         // Обработка ошибки.
			 }
			 Thread.Sleep(500);

		 }
		 private void save_in_instr(int par2, ushort par3)
		 {
			ushort[] writeVals = new ushort[200];
		//	ushort[] readVals = new ushort[125];
			slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
			int res;
			int startWrReg;
			int numWrRegs;

			 switch (par2)                                                // Определить № теста
			 {

				 default:
	
				 case 1:
					 writeVals[0] = 130;                                            // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 0;                                              // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 1;                                              // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                              // 40130 Адрес дата 
	
					 //  Записать где будет информация                                                             Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

					 writeVals[0] = par3;                                             // записать уровень сигнала (резистора)
					 startWrReg = 130;                                                //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 1;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();
					 break;
				 case 2:                           // Отправить таблицу инструктора
			   
					 writeVals[0] = 130;                                            // 130 - Адрес блока регистров для передачи в ПК уровней порогов.
					 writeVals[1] = 2;                                              // 0   - Адрес блока памяти +200 для передачи в ПК уровней порогов.
					 writeVals[2] = 1;                                              // 18  - Длина блока регистров для передачи в ПК уровней порогов.
					 startRdReg = 130;                                              // 40130 Адрес дата 
	
					 //  Записать где будет информация                                                             Отправить параметры блока получения данных из памяти 
					 startWrReg = 127;                                                //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 3;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);

					 writeVals[0] = par3;                                             // записать уровень сигнала (резистора)
					 startWrReg = 130;                                                //  Отправить параметры блока получения данных из памяти 
					 numWrRegs = 1;                                                   //
					 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
					 test_end1();
					 startWrReg = 120;                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 21);      //  Записать блок в EEPROM Камертон 5.0
					 test_end1();

					 break;
			 }
		 }
		 private void save_in_resist(int par2, ushort par3)
		 {
			 switch (par2)                                                // Определить № теста
			 {

				 default:

				 case 1:
					 startWrReg = 60;                                                                // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, par3);                  //  Записать уровень сигнала №1 в Камертон 5.0
					 startWrReg = 120;                                                               // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 38);                    //  Выполнить запись уровень сигнала №1 в Камертон 5.0
					 break;
				 case 2:                          
					 startWrReg = 64;                                                                 // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, par3);                   //  Записать уровень сигнала №2 в Камертон 5.0
					 startWrReg = 120;                                                                // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 39);                     //  Выполнить запись уровень сигнала №2 в Камертон 5.0
					 break;
			 }

		 }
		 private void save_in_Oscilloscope(int par2, ushort osc_channel)
		 {
			 startCoil = 8;                                                            // Управление питанием платы "Камертон"
			 res = myProtocol.writeCoil(slave, startCoil, true);                       // Включить питание платы "Камертон"
			 startWrReg = 120;                                                         // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 44);              //  Записать блок в EEPROM Камертон 5.0
			 Thread.Sleep(1000);
			 startWrReg = 40;                                                          // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, osc_channel);     //  Отправить номер аналогового канала Камертон 5.0
			 //  Записать номер аналогового канала Камертон 5.0
			 startWrReg = 120;                                                         // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 34);              //  Записать номер аналогового канала Камертон 5.0
			 startCoil = 40;
			 res = myProtocol.writeCoil(slave, startCoil, true);                       // Отправить разрешение на измерение
			 Thread.Sleep(400);
			 startWrReg = 120;                                                         // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 35);              // Включить осциллограф
		 }
		 private void on_off_Oscilloscope(int par2, ushort par3)
		 {
			 switch (par2)                                                                      // Определить Вкл / Откл
			 {

				 default:

				 case 1:
					  startCoil = 8;                                                            // Управление питанием платы "Камертон"
					  res = myProtocol.writeCoil(slave, startCoil, true);                       // Включить питание платы "Камертон"
					  startWrReg = 120;                                                         // 
					  res = myProtocol.writeSingleRegister(slave, startWrReg, 44);              //  Записать блок в EEPROM Камертон 5.0
					  Thread.Sleep(1000);
					  startCoil = 40;
					  res = myProtocol.writeCoil(slave, startCoil, true);                       // Отправить разрешение на измерение
					  Thread.Sleep(400);
				 
					  startWrReg = 120;                                                         // 
					  res = myProtocol.writeSingleRegister(slave, startWrReg, 35);              // Включить осциллограф
					  startWrReg = 120;                                                          // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 44);               //  Записать блок в 
					  break;
				 case 2:
					 startCoil = 8;                                                             // Управление питанием платы "Камертон"
					 res = myProtocol.writeCoil(slave, startCoil, false);                       // Отключить питание платы "Камертон"
					 startWrReg = 120;                                                          // 
					 res = myProtocol.writeSingleRegister(slave, startWrReg, 44);               //  Записать блок в EEPROM Камертон 5.0
					 Thread.Sleep(400);
					 startCoil = 40;
					 res = myProtocol.writeCoil(slave, startCoil, false);                       // Прекратить измерение осциллографом
					 Thread.Sleep(200);
					 timerPowerOff.Start();
					 break;
			 }

		 }
		 private void set_rele_upr(ushort par3, bool end_run)                                      //
		 {

			 startCoil = 8;                                              // Управление питанием платы "Камертон"
			 res = myProtocol.writeCoil(slave, startCoil, true);        // Выключить питание платы "Камертон"
			 Thread.Sleep(400);

			 res = myProtocol.writeCoil(slave, par3, end_run);                                     // Отправить разрешение 

			 startWrReg = 120;                                                                     // 
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 45);                          //  Записать блок в регистр

			 int i;

			 startRdReg = 1;
			 numRdRegs = 7;
			 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals_byte, numRdRegs);         //  Считать число из регистров по адресу  40001 -40007
			 if ((res == BusProtocolErrors.FTALK_SUCCESS))
			 {
				 for (int bite_x = 0; bite_x < 7; bite_x++)
				 {
					 decimalNum = readVals_byte[bite_x];
					 while (decimalNum > 0)
					 {
						 binaryHolder = decimalNum % 2;
						 binaryResult += binaryHolder;
						 decimalNum = decimalNum / 2;
					 }

					 int len_str = binaryResult.Length;

					 while (len_str < 8)
					 {
						 binaryResult += 0;
						 len_str++;
					 }

					 //****************** Перемена битов ***************************
					 //binaryArray = binaryResult.ToCharArray();
					 //Array.Reverse(binaryArray);
					 //binaryResult = new string(binaryArray);
					 //*************************************************************
					 for (i = 0; i < 8; i++)                         // 
					 {
						 if (binaryResult[i] == '1')
						 {
							 Dec_bin[i + (8 * bite_x)] = true;
						 }
						 else
						 {
							 Dec_bin[i + (8 * bite_x)] = false;
						 }
					 }
					 binaryResult = "";
				 }

				 if (Dec_bin[41] == false) // 30041  флаг подключения гарнитуры инструктора 2 наушниками
				 {
					 CallBackMy2.callbackEventHandler("Таблица инструктора", 2, 1, 1, false);
				 }
				 else
				 {
					 CallBackMy2.callbackEventHandler("Таблица инструктора", 2, 1, 1, true);
				 }

				 if (Dec_bin[42] == false) // 30042  флаг подключения гарнитуры инструктора
				 {
					 CallBackMy2.callbackEventHandler("Таблица инструктора", 2, 2, 1, false);
				 }
				 else
				 {
					 CallBackMy2.callbackEventHandler("Таблица инструктора", 2, 2, 1, true);
				 }

			 }
			 else
			 {
				 MODBUS_SUCCESS             = false;
				 Com2_SUCCESS               = false;
				 toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set4)  ";
				 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");       // Обработка ошибки.
			 }

		 }

		 private void checkedListBox1_ItemCheck(object sender, ItemCheckEventArgs e)
		 {
			 // Расширить и уточнить включение реле и другиз кнопок
			 var list = sender as CheckedListBox;
			 if (e.NewValue == CheckState.Checked)
				 foreach (int index in list.CheckedIndices)
					 if (index != e.Index)
					 {
						 list.SetItemChecked(index, false);
					 }

			 // int selected = checkedListBox1.SelectedIndex;
			 // label12.Text = checkedListBox1.Items[selected].ToString();
			 // label12.Text += string.Format("{0:0}", checkedListBox1.SelectedIndex);

			 for (int i = 1; i < 10; i++)
			 {
				 coil_Button[i] = false;
			 }
		//     coil_Button[8] = true;           // Включить питание +12в.

			 switch (checkedListBox1.SelectedIndex)
			 {
				 default:
				 case 0:
					 coil_Button[2] = true;   // Микрофон инструктора
					 coil_Button[5] = true;
					 break;
				 case 1:
					 coil_Button[1] = true;   // Микрофон диспетчера
					 coil_Button[10] = true;
					 break;
				 case 2:
					 coil_Button[3] = true;   // Микрофон трубка
					 break;
				 case 3:
					 coil_Button[9] = true;   // Микрофон 
					 break;
				 case 4:
					 coil_Button[25] = true;  // ГГС
					 coil_Button[6]  = true;   // ГГС
					 break;
				 case 5:
					 coil_Button[4] = true;   // Радио 1
					 break;
				 case 6:
					 coil_Button[7] = true;   // Радио 2
					 break;
				 case 7:                      // Отключить все
					 for (int i = 1; i < 10; i++)
					 {
						 coil_Button[i] = false;
					 }
					 break;
			 }

			 startCoil = 1;
			 numCoils = 24;

			 res = myProtocol.forceMultipleCoils(slave, startCoil, coil_Button, numCoils);              
			 if ((res == BusProtocolErrors.FTALK_SUCCESS))
			 {
				 MODBUS_SUCCESS = true;
				 toolStripStatusLabel1.Text = "    MODBUS ON    ";
			 }
			 else
			 {
				 MODBUS_SUCCESS             = false;
				 Com2_SUCCESS               = false;
				 toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set5 Osc)  ";
				 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");         // Обработка ошибки.
			 }

			 for(int i_coil = 0; i_coil < 16; i_coil++)
				 {
					 coilArr[i_coil] = coil_Button[24 + i_coil];
				 }

			 startCoil = 25;
			 numCoils = 9;
			 res = myProtocol.forceMultipleCoils(slave, startCoil, coilArr, numCoils);                  // 

			 if((res == BusProtocolErrors.FTALK_SUCCESS))
				 {
					 MODBUS_SUCCESS = true;
					 toolStripStatusLabel1.Text = "    MODBUS ON    ";
				 }
			 else
				 {
					 MODBUS_SUCCESS             = false;
					 Com2_SUCCESS               = false;
					 toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set6 Osc)  ";
					 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
				 }

		 }

		 private void Form1_LocationChanged(object sender, EventArgs e)
			 {
				 loc_x = this.Location.X;
				 loc_y = this.Location.Y;
				 loc_x += 25;
				 loc_y += 70;

				 if(loc_x < 20)
					 {
					 loc_x = 20;
					 }

				 else if(loc_x >= Form_Width)
					 {
					 loc_x = Form_Width;
					 }
				 if (loc_y < 56) loc_y = 56;

				 else if (loc_y > Form_Height)
				 {
					 loc_y = Form_Height;
				 }
			 
				 oscilloscope1.Left = loc_x;
				 oscilloscope1.Top  = loc_y;
 
			 }

		 private void checkedListBox1_SelectedIndexChanged(object sender, EventArgs e)
			 {
				 // Расширить и уточнить включение реле и другиз кнопок
				 var list = sender as CheckedListBox;
				 int selected = checkedListBox1.SelectedIndex;
				 label12.Text = checkedListBox1.Items[selected].ToString();
				 label12.Text += string.Format("{0:0}", checkedListBox1.SelectedIndex);

				 for(int i = 1; i < 10; i++)
					 {
					 coil_Button[i] = false;
					 }
				 coil_Button[8] = true;           // Включить питание +12в.
				 switch(checkedListBox1.SelectedIndex)
					 {
					 default:
					 case 0:
							coil_Button[2] = true;   // Микрофон инструктора
							coil_Button[5] = true;
							coil_Button[27] = true;
						  break;
					 case 1:
						 coil_Button[1] = true;  // Микрофон диспетчера
						 coil_Button[10] = true;
						 break;
					 case 2:
						 coil_Button[3] = true;   // Микрофон трубка
						 break;
					 case 3:
						 coil_Button[9] = true;   // Микрофон 
						 coil_Button[3] = true;
						 coil_Button[16] = true;
						 break;
					 case 4:
						 coil_Button[6] = true;   // ГГС
						 break;
					 case 5:
						 coil_Button[4] = true;   // Радио 1
						 break;
					 case 6:
						 coil_Button[7] = true;   // Радио 2
						 break;
					 }

				 startCoil = 1;
				 numCoils = 10;

				 res = myProtocol.forceMultipleCoils(slave, startCoil, coil_Button, numCoils);              // 15 (0F) Записать бит false или true  по адресу 0-9999 
				 if((res == BusProtocolErrors.FTALK_SUCCESS))
					 {
					 MODBUS_SUCCESS = true;
					 toolStripStatusLabel1.Text = "    MODBUS ON    ";
					 }
				 else
					 {
					 MODBUS_SUCCESS             = false;
					 Com2_SUCCESS               = false;
					 toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set5)  ";
					 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");         // Обработка ошибки.
					 }
			 }

#endregion

		 private void button32_Click(object sender, EventArgs e)               // Чтение EEPROM. Проверка и настройка регистров передачи информации
		 {

			 ushort[] writeVals = new ushort[20];
			 ushort[] readVolt = new ushort[10];
			 int numRdRegs = 4;
			 int numWrRegs = 3;
			 startWrReg = 120;
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 17);                         // Провести измерение питания
			 if ((res == BusProtocolErrors.FTALK_SUCCESS))
			 {
				 MODBUS_SUCCESS = true;
				 toolStripStatusLabel1.Text = "    MODBUS ON    ";
			 }
			 else
			 {
				 MODBUS_SUCCESS             = false;
				 Com2_SUCCESS               = false;
				 toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set2)  ";
				 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");  // Обработка ошибки.
			 }
			 test_end1();
			 // *********************** Проверить питание ********************************
			 writeVals[0] = 130;                                                  //  Адрес блока регистров для передачи в ПК уровней порогов.
			 writeVals[1] = 1684;                                                 //  Адрес блока памяти  для передачи в ПК уровней порогов.
			 writeVals[2] = 10;                                                   //  Длина блока регистров для передачи в ПК уровней порогов.


			 startWrReg = 124;                                                    //  Отправить параметры блока получения данных из памяти 
			 numWrRegs = 3;                                                       //
			 res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
			 test_end1();
			 Thread.Sleep(200);
			 startWrReg = 120;
			 res = myProtocol.writeSingleRegister(slave, startWrReg, 15);        // Передать информацию в регистры
			 test_end1();

			 startRdReg = 130;
			 numRdRegs = 10;
			 res = myProtocol.readMultipleRegisters(slave, startRdReg, readVolt, numRdRegs);
			 if ((res == BusProtocolErrors.FTALK_SUCCESS))
			 {
	
				 richTextBox2.Text = "";
				 for (int i = 0; i < 10; i++)                // Чтение блока регистров 
				 {
					 double xx = readVolt[i] * 2.70 / 100;

					 richTextBox2.Text += (i + " - " + xx.ToString() + "\r\n");
				 }

			 }
			 else
			 {
				 MODBUS_SUCCESS             = false;
				 Com2_SUCCESS               = false;
				 toolStripStatusLabel1.Text = "    MODBUS ERROR (byte_set3)  ";
				 toolStripStatusLabel4.Text = ("Связь с прибором КАМЕРТОН 5  НЕ УСТАНОВЛЕНА !");          // Обработка ошибки.
			 }


		 }

		 void Reload(string param, int par1, int par2, ushort par3, bool end_run)
		 {
			 //Здесь чего нибудь делаем.
			 //Это непосредственно то что выполнится по событию.
			switch (par1)                                                // Определить № комманды
			 {

				 default:
				 case 0:
					 //
					 break;
				 case 1:                         
					  get_porog_memory();          // Получить таблицу порогов
					  CallBackMy2.callbackEventHandler("Таблица инструктора", 0, 1, 1,true);
					  break;
				 case 2:                           // Отправить таблицу инструктора
					  CallBackMy2.callbackEventHandler("Таблица инструктора", 1, par2, table_porog[par2], true);
					 break;
				 case 3:                                             // Сообщение о выполнении  записи таблицы порогов
					 if      (par2 == 1) Insrt_Table_Read = end_run;
					 else if (par2 == 2) Disp_Table_Read  = end_run;
					 else if (par2 == 3) Mtt_Table_Read   = end_run;
					 else if (par2 == 4) Mic_Table_Read   = end_run;
					 else if (par2 == 5) Ggs_Table_Read   = end_run;
					 else if (par2 == 6) Rad1_Table_Read  = end_run;
					 else if (par2 == 7) Rad2_Table_Read  = end_run;
					 break;
				 case 4:
					 set_sound(par3);
					 break;
				 case 5:
					 set_chanal(par3);
					 break;
				 case 6:                                       // Записать уровень входного сигнала
					 save_in_instr(par2, par3);
					 break;
				 case 7:                                       // Установить параметры резисторов
					 save_in_resist(par2, par3);
					 break;
				 case 8:                                       // Установить вход осциллографа
					 save_in_Oscilloscope(par2, par3);         //
					 break;
				 case 9:
					  on_off_Oscilloscope(par2, par3);         //
					 break;
				 case 10:
					 serial_connect2();                        //
					 break;
				 case 11:
					 set_rele_upr(par3, end_run);                 //
					 break;
				 //case 12:
				 //    //
				 //    break;
				 //case 13:
				 //    //
				 //    break;
				 //case 14:
				 //    //
				 //    break;
				 //case 15:
				 //    //
				 //    break;

			 }

		 }

		 private void button67_Click(object sender, EventArgs e)               // Чтение списка файлов с HDD
		 {
			// Stream myStream = null;
			 OpenFileDialog openFileDialog1 = new OpenFileDialog();

			 openFileDialog1.InitialDirectory = pathString;
			// openFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
			 openFileDialog1.Filter = "txt files (*.KAM)|*.txt|All files (*.*)|*.*";

			 openFileDialog1.FilterIndex = 2;
			 openFileDialog1.RestoreDirectory = true;

			 if (openFileDialog1.ShowDialog() == DialogResult.OK)
			 {
				 try
				 {
							pathStringHDD = openFileDialog1.FileName;
					
								 if ((File.Exists(folderNotepadName)))
								 {
									 //openFileDialog1.FileName
									 System.Diagnostics.Process.Start((@"C:\Program Files\Notepad++\notepad++.exe"), pathStringHDD);
								 }
								 else
								 {
									 folderFormatName = @"C:\Program Files (x86)\Notepad++\notepad++.exe";
									 if ((File.Exists(folderFormatName)))
									 {
										 System.Diagnostics.Process.Start((@"C:\Program Files (x86)\Notepad++\notepad++.exe"), pathStringHDD);
									 }
									 else
									 {
										 MessageBox.Show("                         Внимание!\r\n\r\n Программа редактирования не установлена", "Вызов программы редактирования", MessageBoxButtons.OK, MessageBoxIcon.Warning);
									 }

								 }
   
					
				 }
				 catch (Exception ex)
				 {
					 MessageBox.Show("Ошибка открытия файла" + ex.Message);
				 }
			 }
		 }

         private void button82_Click(object sender, EventArgs e)               // Чтение порогов из файлов
         {
             OpenFileDialog openFileDialog1 = new OpenFileDialog();
             openFileDialog1.InitialDirectory = pathStringINI;
             openFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*|ini files (*.ini)|*.ini";
             openFileDialog1.FilterIndex = 3;
             openFileDialog1.RestoreDirectory = true;

             if (openFileDialog1.ShowDialog() == DialogResult.OK)
             {
                 try
                 {
                     using (StreamReader sr = new StreamReader(openFileDialog1.FileName))
                     {
                         string line;
                         int i = 0;

                         while ((line = sr.ReadLine()) != null)
                         {
                            table_porog[i] = ushort.Parse(line);
                            i++;
                         }
                      }

                     instruk_table1();
                     dispatch_table();
                     mtt_table();
                     mic_table();
                     ggs_table();
                     radio1_table();
                     radio2_table();
                     All_Table_Read = true;
                     Insrt_Table_Read = true;
                     Disp_Table_Read = true;
                     Mtt_Table_Read = true;
                     Ggs_Table_Read = true;
                     Rad1_Table_Read = true;
                     Rad2_Table_Read = true;

                     string message = "Для применения настроек, необходимо записать их в Камертон 5.0 \r\n\r                      Больше не показывать это сообщение? ";
                     string caption = "Настройки порогов";
                     MessageBoxButtons buttons = MessageBoxButtons.YesNo;
                     DialogResult result;
 
                     if (Message_show)
                     { 
                         result = MessageBox.Show(message, caption, buttons);

                         if (result == System.Windows.Forms.DialogResult.Yes)
                         {
                             Message_show = false;
                         }
                     }
                  }
                 catch (Exception ex)
                 {
                     MessageBox.Show("Error: Could not read file from disk. Original error: " + ex.Message);
                 }
             }
         }

   

         private void button68_Click(object sender, EventArgs e)                 // Сохранить в файл
         {

             SaveFileDialog SaveFileDialog1 = new SaveFileDialog();
             SaveFileDialog1.CreatePrompt = true;
             SaveFileDialog1.OverwritePrompt = true;
             SaveFileDialog1.FileName = "";
             SaveFileDialog1.DefaultExt = "ini";
             SaveFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*|ini files (*.ini)|*.ini";
             SaveFileDialog1.InitialDirectory = pathStringINI;
             SaveFileDialog1.FilterIndex = 3;
             DialogResult result = SaveFileDialog1.ShowDialog();
             Stream fileStream;
             string val_porog = "";
             save_tab_array();
             if (result == DialogResult.OK)
             {
                   try
                 {
                     save_tab_array();
                    for (int i = 0; i < table_porog.Length; i++)
                     {
                         val_porog += table_porog[i].ToString();
                         val_porog += "\r\n";
                     }

                      // convert string to stream
                     byte[] buffer = Encoding.ASCII.GetBytes(val_porog);
                    //write to file
                     MemoryStream userInput = new MemoryStream(buffer);

                     // Open the file, copy the contents of memoryStream to fileStream,
                     // and close fileStream. Set the memoryStream.Position value to 0 
                     // to copy the entire stream. 
                     // Открыть файл, скопировать содержимое MemoryStream для FileStream,
                     // И близко FileStream. Установите значение memoryStream.Position до 0
                     // Скопировать весь поток.

                     fileStream = SaveFileDialog1.OpenFile();
                     userInput.ToString();
                     userInput.Position = 0;
                     userInput.WriteTo(fileStream);
                     fileStream.Close();
                 }
                   catch (Exception ex)
                   {
                       MessageBox.Show("Error: Could not save file from disk. Original error: " + ex.Message);
                   }
             }
             
         }

         private void checkBox1_CheckedChanged_1(object sender, EventArgs e)
         {
             if (All_Table_Read)
             {
                 instruk_table1();                                                 // Отобразить таблицу порогов  инструктора
                 dispatch_table();                                                 // Отобразить таблицу порогов диспетчера
                 mtt_table();                                                      // Отобразить таблицу порогов MTT
                 mic_table();                                                      // Отобразить таблицу порогов микрофона
                 ggs_table();                                                      // Отобразить таблицу порогов GGS
                 radio1_table();                                                   // Отобразить таблицу порогов Radio1
                 radio2_table();                                                   // Отобразить таблицу порогов Radio2
             }
          }

   
      
	 }

	public static class CallBackMy
	{
		public delegate void callbackEvent(string what, int par1, int par2, ushort par3, bool end_run);
		public static callbackEvent callbackEventHandler;
	}
 }





