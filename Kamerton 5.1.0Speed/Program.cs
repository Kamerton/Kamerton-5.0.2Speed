// Программа управления Камертон5.0
// дата создания 17.09.2015 г.

using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Threading;
using System.Linq;
using KamertonTest;


namespace KamertonTest
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        /// 

        // static public Form1 frm1;
         static public Form2 frm2;
         static public Form3 frm3;
         static public Form4 frm4;
         static public Form5 frm5;
         static public Form6 frm6;
         static public Form7 frm7;
         static public Form8 frm8;





               // Запуск программы и защита от повторного запуска программы
        [STAThread]
        static void Main()
        {     
                string key = "Kamerton501";                 //пишем тут любой ключ,это для проверки(вместо названия программы)
                using(Mutex mutex = new Mutex(false, key))
                {
                if(!mutex.WaitOne(0, false))
                {
                    MessageBox.Show("Программа уже открыта!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
                }
                else
                {
                 //   GC.Collect();
                    Application.EnableVisualStyles();
                    Application.SetCompatibleTextRenderingDefault(false);
                 // frm1 = new Form1();
                    frm2 = new Form2();
                    frm3 = new Form3();
                    frm4 = new Form4();
                    frm5 = new Form5();
                    frm6 = new Form6();
                    frm7 = new Form7();
                    frm8 = new Form8();
  
                 //   Application.Run(frm1);
                    Application.Run(new Form1());
                 }
              } 

        }
    }
}