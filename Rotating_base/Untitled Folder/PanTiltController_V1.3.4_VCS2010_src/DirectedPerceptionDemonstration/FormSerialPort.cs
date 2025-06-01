using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
//using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;

namespace DPerceptionDemo
{
    public partial class FormSerialPort : Form
    {
        public Form1 parents;

        public FormSerialPort()
        {
            InitializeComponent();
        }

        private void FormSerialPort_Load(object sender, EventArgs e)
        {
            // Get a list of serial port names.
            string[] ports = SerialPort.GetPortNames();
            // Display each port name to the console.
            listBox1.Items.Clear();
            foreach (string port in ports)
            {
                listBox1.Items.Add(port);
                //portToolStripMenuItem.DropDownItems.Add(port);
            }
            listBox1.Focus();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            parents.SerialPortClose();
            string selectcom = (string)listBox1.SelectedItem;
            parents.SerialPortOpen(selectcom);
            //portToolStripMenuItem = false;
            Close();
        }

        private void button2_Click(object sender, EventArgs e)
        {
            Close();
        }

        private void listBox1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                EventArgs temp = new EventArgs();
                button1_Click( sender, temp);
            }
        }

        private void listBox1_DoubleClick(object sender, EventArgs e)
        {
            button1_Click(sender, e);
        }
    }
}
