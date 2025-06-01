using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
//using System.Linq;
using System.Text;
using System.Windows.Forms;


namespace DPerceptionDemo
{
    public partial class Form2 : Form
    {
        public Form1 parents;

        public Form2()
        {
            InitializeComponent();
            this.ShowInTaskbar = false;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            Visible = false;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            parents.SerialWrite(textBox2.Text+"\n");
        }

        private void textBox2_Click(object sender, EventArgs e)
        {
        }
        private void textBox2_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                button2_Click(sender, e);
//                textBox2.Text = "";
            }
        }
        private void textBox2_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == (char)Keys.Enter)
            {
                e.Handled = true;
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            label1.Text = "Baud rate : "+parents.serialPort1.BaudRate.ToString();
        }

        private void button3_Click(object sender, EventArgs e)
        {
            textBox3.Text = "";
        }
    }
}
