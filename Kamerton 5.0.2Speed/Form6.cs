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
    public partial class Form6 : Form
        {
            //Событие для передачи данных
            public event EventHandler<UserEventArgs> sendDataFromFormEvent6;
            public event EventHandler<UserEventArgs> sendDataFromFormEventS6;
        public Form6()
            {
                InitializeComponent();
            }

 
        private void button1_Click(object sender, EventArgs e)
            {
                this.Close();
            }

        private void Form6_Load(object sender, EventArgs e)
            {
                
            }

        private void ggs_save_Click(object sender, EventArgs e)
        {

            //Создаем таблицу
            DataTable dt_Form6 = new DataTable("InstruktorDataTable");

            //Создаем столбцы
            DataColumn noise_min = new DataColumn("шумы мин.", typeof(string));
            DataColumn noise_min1 = new DataColumn("минC.", typeof(string));
            // DataColumn noise_max = new DataColumn("шумы макс.", typeof(string));

            dt_Form6.Columns.AddRange(new DataColumn[] 
            {noise_min,
             noise_min1});

            //Формируем строку из данных в текстовых полях
            DataRow row = dt_Form6.NewRow();
            row[noise_min] = textBox24.Text;
            row[noise_min1] = textBox38.Text;
            dt_Form6.Rows.Add(row);
            DataRow row1 = dt_Form6.NewRow();
            row1[noise_min] = textBox23.Text;
            row1[noise_min1] = textBox29.Text;
            dt_Form6.Rows.Add(row1);
            DataRow row2 = dt_Form6.NewRow();
            row2[noise_min] = textBox21.Text;
            row2[noise_min1] = textBox28.Text;
            dt_Form6.Rows.Add(row2);
            DataRow row3 = dt_Form6.NewRow();
            row3[noise_min] = textBox20.Text;
            row3[noise_min1] = textBox27.Text;
            dt_Form6.Rows.Add(row3);
            DataRow row4 = dt_Form6.NewRow();
            row4[noise_min] = textBox19.Text;
            row4[noise_min1] = textBox22.Text;
            dt_Form6.Rows.Add(row4);
            DataRow row5 = dt_Form6.NewRow();
            row5[noise_min] = textBox18.Text;
            row5[noise_min1] = textBox11.Text;
            dt_Form6.Rows.Add(row5);
            DataRow row6 = dt_Form6.NewRow();
            row6[noise_min] = textBox17.Text;
            row6[noise_min1] = textBox9.Text;
            dt_Form6.Rows.Add(row6);
            DataRow row7 = dt_Form6.NewRow();
            row7[noise_min] = textBox16.Text;
            row7[noise_min1] = textBox8.Text;
            dt_Form6.Rows.Add(row7);
            DataRow row8 = dt_Form6.NewRow();
            row8[noise_min] = textBox15.Text;
            row8[noise_min1] = textBox7.Text;
            dt_Form6.Rows.Add(row8);

            //Генерируем событие с именованным аргументом
            //в класс аргумента передаем созданную таблицу
            if (sendDataFromFormEvent6 != null)
                sendDataFromFormEvent6(this, new UserEventArgs(dt_Form6));

            //Создаем таблицу
            DataTable dt_Form6S = new DataTable("InstruktorDataTable");

            //Создаем столбцы
            DataColumn noise_minS = new DataColumn("мин.", typeof(string));
            DataColumn noise_maxS = new DataColumn("макс.", typeof(string));

            dt_Form6S.Columns.AddRange(new DataColumn[] {
            noise_minS,
            noise_maxS});

            //Формируем строку из данных в текстовых полях
            DataRow rowS = dt_Form6S.NewRow();
            rowS[noise_minS] = textBox6.Text;
            rowS[noise_maxS] = textBox37.Text;
            dt_Form6S.Rows.Add(rowS);
            DataRow row1S = dt_Form6S.NewRow();
            row1S[noise_minS] = textBox12.Text;
            row1S[noise_maxS] = textBox36.Text;
            dt_Form6S.Rows.Add(row1S);
            DataRow row2S = dt_Form6S.NewRow();
            row2S[noise_minS] = textBox14.Text;
            row2S[noise_maxS] = textBox35.Text;
            dt_Form6S.Rows.Add(row2S);
            DataRow row3S = dt_Form6S.NewRow();
            row3S[noise_minS] = textBox13.Text;
            row3S[noise_maxS] = textBox34.Text;
            dt_Form6S.Rows.Add(row3S);
            DataRow row4S = dt_Form6S.NewRow();
            row4S[noise_minS] = textBox10.Text;
            row4S[noise_maxS] = textBox33.Text;
            dt_Form6S.Rows.Add(row4S);
            DataRow row5S = dt_Form6S.NewRow();
            row5S[noise_minS] = textBox3.Text;
            row5S[noise_maxS] = textBox32.Text;
            dt_Form6S.Rows.Add(row5S);
            DataRow row6S = dt_Form6S.NewRow();
            row6S[noise_minS] = textBox2.Text;
            row6S[noise_maxS] = textBox31.Text;
            dt_Form6S.Rows.Add(row6S);
            DataRow row7S = dt_Form6S.NewRow();
            row7S[noise_minS] = textBox1.Text;
            row7S[noise_maxS] = textBox30.Text;
            dt_Form6S.Rows.Add(row7S);
            DataRow row8S = dt_Form6S.NewRow();
            row8S[noise_minS] = textBox5.Text;
            row8S[noise_maxS] = textBox4.Text;
            dt_Form6S.Rows.Add(row8S);

            //Генерируем событие с именованным аргументом
            //в класс аргумента передаем созданную таблицу
            if (sendDataFromFormEventS6 != null)
                sendDataFromFormEventS6(this, new UserEventArgs(dt_Form6S));

            this.Close();

        }

    

        private void label57_Click(object sender, EventArgs e)
        {

        }
 
        }
    }
