using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Threading;
using System.Management;



namespace ComPortAutodetect
    {
    public partial class Form1 : Form
        {
        SerialPort currentPort; // Create Serial Port
        bool portFound; // bool for noting if the serial port has been found
        string port_name = ""; // name of the prot, will be set if found in arduino detect function

        public Form1()
            {

               // SetComPort(); // Look for the serial port as the program starts
                Thread t = new Thread(new ThreadStart(SplashScreen));
                t.Start();
               // Thread.Sleep(4000);
                InitializeComponent();
                t.Abort();

            }

        // Splash Screen for the application
        public void SplashScreen()
            {
            // Application.Run(new Form2());
            }



        private void Form1_Load(object sender, EventArgs e)
            {
            /*
            if(portFound)
                {
                serialPort1.PortName = port_name; // Assign port name to the serial port
                serialPort1.BaudRate = 9600;
                sts_lbl.Text = "CONNECTED";
                sts_lbl.BackColor = Color.LimeGreen;
                }

            else // Board not found
                {
                sts_lbl.Text = "DISCONNECTED";
                sts_lbl.BackColor = Color.Red;
                MessageBox.Show("USB Not Connected or Driver not Detected.Check for Connection Settings", "Connection Warning!", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            */
            }


        // *************************************************************************************
        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
            {
            SerialPort port = (SerialPort)sender;
            string data = port.ReadExisting();
            // check here what data you received, if any
            }

        //**************************************************************************
        // Set Comm Port

        private void SetComPort()
            {

              try
                {
                string[] ports = SerialPort.GetPortNames();//creat array with all port names on computer
                foreach(string port in ports)//for every port (string) from the ports array.
                    {
                    currentPort = new SerialPort(port, 115200);//create com port using current name from the array
                    //of all com ports, baud rate is 9600

                    label1.Text += (port);//debuging

                   // MessageBox.Show(port);//debuging

                    if(DetectArduino())//check is the arduino is on this current port
                        {

                        label1.Text += ("   Arduino Found!!! \r\n");//debuging
                       // MessageBox.Show("Arduino Found!!!");//debuging
                        port_name = port.ToString();//convert the port to a string and then set port_name
                        portFound = true;//set port flag to true
                        break;
                        }
                    else
                        {

                        label1.Text += ("  Arduino Not Found \r\n");//debugging
                        // MessageBox.Show("Arduino Not Found");//debugging
                        portFound = false;//set port flag to false
                        }
                    }
                }
            catch(Exception e)
                {

               // label1.Text += ("Error!");//show error message is something happend
               // MessageBox.Show("Error!");//show error message is something happend
                }
            }

        //***********************************************************************

        ///***********************************************************************
        // Detect Arduino Code

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
                buffer[4] = Convert.ToByte(0);//empty command

                int intReturnASCII = 0;
                char charReturnValue = (Char)intReturnASCII;

                currentPort.Open();//open com port connection
                currentPort.Write(buffer, 0, 5);//send the message from the buffer array
                Thread.Sleep(1000);//wait, give time for replay

                int count = currentPort.BytesToRead;//get lenght of replay message
                string returnMessage = "";
                while(count > 0)
                    {
                    intReturnASCII = currentPort.ReadByte();

                    returnMessage = returnMessage + Convert.ToChar(intReturnASCII);//creat message letter by letter
                    //to create a string
                    //   MessageBox.Show(returnMessage);//debuging
                    count--;
                    }

                if(returnMessage.Contains("HELLO"))//The Arduino said hello
                    {
                    currentPort.Close();//close the serial com port connection
                    label1.Text += ("  " + returnMessage + "  ");//debuging
                    return true;
                    }
                else//device was either not an arduino, or it didn't say hello
                    {
                    currentPort.Close();//close the serial com port connection
                    return false;
                    }
                }
            catch(Exception e)
                {
                  label1.Text += ("Error, Device not connected \r\n");
                //MessageBox.Show("Error, Device not connected");
                return false;
                }
            }

        private void button1_Click(object sender, EventArgs e)
            {

               label1.Text = "";
               label1.Refresh();
               SetComPort();
            }


        }
    }
