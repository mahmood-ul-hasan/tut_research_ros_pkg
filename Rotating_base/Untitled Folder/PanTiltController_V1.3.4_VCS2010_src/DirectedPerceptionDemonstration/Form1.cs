//Directed Perception PTU controlprogram
//2008/00/00
//
//2009/11/15 Ring buffer 修正：シリアル最後の1文字読めない現象修正
//2010/7/3   Playback 常に有効
//           Serialmonitor listbox->textbox
//2010/7/4   SerialPanel auto minimize & no taskbar
//20101104   V1.3 use xmlfile for backup datas
//20160624   use delegate serial stream
//20161101   use lock(comsync) for SerialStream,SerialStream2 
//20161101   Serial buffer size management.
//20161104   GetCommandReturn 条件修正
//20161107   serial str初期化位置変更

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
//using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
//using System.Runtime.InteropServices;//for [DllImport

using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace DPerceptionDemo
{
    public partial class Form1 : Form
    {
        Form2 f_2;
        RingBuffer binarySerial = new RingBuffer(1000);
        RingBuffer binarySerial_display = new RingBuffer(1000);

        object comsync = new object();//use by lock for serialstream

        int PresetTime;
        int PresetMax;
        bool PresetDo;
        string SerialCOM="COM1";
        string SerialRate = "9600";
        volatile string SerialStream = "";
        volatile string SerialStream2 ="";
        int CommandWaitTimer = 0;
        bool SetSpeedMode = false;

        string backupPanspeed = "";
        string backupTiltspeed = "";

        string NOPORTmsg = "----";

        //TCP/IP data receive==============
        String thread_message = "";
        String IP_message = "";
        // リスナースレッド
        private Thread th = null;
        private TcpListener server = null;
        TcpClient client = null;
        //TCP/IP data receive==============

        CheckBox[] preset_c = new CheckBox[20];
        RadioButton[] preset_r = new RadioButton[20];
        TextBox[] preset_p = new TextBox[20];
        TextBox[] preset_t = new TextBox[20];
//=========== precision timer (and using System.Runtime.InteropServices;)
/*        [DllImport("kernel32.dll")]
        extern static short QueryPerformanceCounter(ref long x);
        [DllImport("kernel32.dll")]
        extern static short QueryPerformanceFrequency(ref long x);
*/
//============ Form =====================================================================
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            f_2 = new Form2();//==============Serial Monitor Setup ==========================
            f_2.parents = this;
            f_2.Left = this.Right;
            f_2.Top = this.Top;
            f_2.Show();
            f_2.Visible = false;

            binarySerial.Flush();

            preset_r[0] = radioButton1;
            preset_r[1] = radioButton2;
            preset_r[2] = radioButton3;
            preset_r[3] = radioButton4;
            preset_r[4] = radioButton5;
            preset_r[5] = radioButton6;
            preset_r[6] = radioButton7;
            preset_r[7] = radioButton8;
            preset_r[8] = radioButton9;
            preset_r[9] = radioButton10;
            preset_c[0] = checkBox1;
            preset_c[1] = checkBox2;
            preset_c[2] = checkBox3;
            preset_c[3] = checkBox4;
            preset_c[4] = checkBox5;
            preset_c[5] = checkBox6;
            preset_c[6] = checkBox7;
            preset_c[7] = checkBox8;
            preset_c[8] = checkBox9;
            preset_c[9] = checkBox10;
            preset_p[0] = textBox19;
            preset_p[1] = textBox21;
            preset_p[2] = textBox23;
            preset_p[3] = textBox25;
            preset_p[4] = textBox27;
            preset_p[5] = textBox29;
            preset_p[6] = textBox31;
            preset_p[7] = textBox33;
            preset_p[8] = textBox35;
            preset_p[9] = textBox37;
            preset_t[0] = textBox20;
            preset_t[1] = textBox22;
            preset_t[2] = textBox24;
            preset_t[3] = textBox26;
            preset_t[4] = textBox28;
            preset_t[5] = textBox30;
            preset_t[6] = textBox32;
            preset_t[7] = textBox34;
            preset_t[8] = textBox36;
            preset_t[9] = textBox38;
            PresetMax = 10;

//=============read parameter file
            readparameters("default.cfg");

            SerialPortOpen( SerialCOM, SerialRate );

            saveFileDialog1.InitialDirectory = System.IO.Directory.GetCurrentDirectory();
            openFileDialog1.InitialDirectory = System.IO.Directory.GetCurrentDirectory();

            rscmd_ret_timer.Enabled = true;

            TCPIPserver();//start or stop TCP/IP server by checkbox13
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
//TCP/IP server cleanup
            if (server != null)
            {
                server.Stop();
                server = null;
            }
            if (th != null)
            {
                th.Abort();
                th = null;
            }
// parameter save
//            Properties.Settings.Default.Save();

            writeparameters("default.cfg");
        }

        private void Form1_Resize(object sender, EventArgs e)
        {
            if (this.WindowState == FormWindowState.Minimized)
            {
                f_2.WindowState = FormWindowState.Minimized;
            }
            else if (this.WindowState == FormWindowState.Normal)
            {
                //f_2.WindowState = FormWindowState.Normal;
            }
        }
//=============== File Menu =================================================================
        private void quitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.Close();
        }
//=============== Menu =================================================================
//=============== Port Menu
        //Open
        private void toolStripMenuItem1_Click(object sender, EventArgs e)
        {
            FormSerialPort f_3 = new FormSerialPort();
            f_3.parents = this;
            f_3.StartPosition = FormStartPosition.CenterScreen;
            f_3.ShowDialog();
        }
        //Close
        private void toolStripMenuItem2_Click(object sender, EventArgs e)
        {
            SerialPortClose();
        }

        //9600bps
        private void toolStripMenuItem4_Click(object sender, EventArgs e)
        {
            SerialRate = "9600";
            SerialPortBaudRate(SerialRate);

        }
        //19200bps
        private void toolStripMenuItem5_Click(object sender, EventArgs e)
        {
            SerialRate = "19200";
            serialPort1.BaudRate = Convert.ToInt32(SerialRate);
            serialPort1.DiscardInBuffer();
            serialPort1.DiscardOutBuffer();
        }
        //38400bps
        private void toolStripMenuItem6_Click(object sender, EventArgs e)
        {
            SerialRate = "38400";
            serialPort1.BaudRate = Convert.ToInt32(SerialRate);
            serialPort1.DiscardInBuffer();
            serialPort1.DiscardOutBuffer();
        }
        //57600bps
        private void bpsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            SerialRate = "57600";
            serialPort1.BaudRate = Convert.ToInt32(SerialRate);
            serialPort1.DiscardInBuffer();
            serialPort1.DiscardOutBuffer();
        }
        //115200bps
        private void bpsToolStripMenuItem1_Click(object sender, EventArgs e)
        {
            SerialRate = "115200";
            serialPort1.BaudRate = Convert.ToInt32(SerialRate);
            serialPort1.DiscardInBuffer();
            serialPort1.DiscardOutBuffer();
        }
/*        //230400bps
        private void bpsToolStripMenuItem2_Click(object sender, EventArgs e)
        {
            SerialRate = "230400";
            int BRate = Convert.ToInt32(SerialRate);
            serialPort1.BaudRate = BRate;
            serialPort1.DiscardInBuffer();
            serialPort1.DiscardOutBuffer();
        }*/
//==============Window Menu
        private void serialMonitorToolStripMenuItem_Click(object sender, EventArgs e)
        {
            f_2.Visible = !f_2.Visible;
            f_2.Top = this.Top;
            f_2.Left = this.Right;
        }

        private void helpHToolStripMenuItem1_Click(object sender, EventArgs e)
        {
            Help.ShowHelp(this, "PanTiltController.chm");
        }

        private void aboutToolStripMenuItem_Click(object sender, EventArgs e)
        {
            AboutBox1 aboutForm = new AboutBox1();
            aboutForm.ShowDialog();
        }

//============ Serial thread ==============================================================
        private void serialPort1_DataReceived_1(object sender, SerialDataReceivedEventArgs e)
        {
            Int32 indata;

            while (serialPort1.BytesToRead > 0)
            {
                string str = "";
                while (serialPort1.BytesToRead > 0)
                {
                    indata = serialPort1.ReadByte();
                    str=str+ Convert.ToChar( indata );
                    binarySerial.Write( indata );
                    binarySerial_display.Write(indata);
                }
                lock (comsync)
                {
//                    SerialStream = SerialStream + str;
                    SerialStream = SerialStream + str;
                    SerialStream2 = SerialStream2 + str;
                }
            }

        }


        public void SerialPortOpen(String str1, String str2)
        {
            try
            {
                if ( str1 != null)
                {
                    serialPort1.PortName = str1;
                }
                if (str2 != null)
                {
                    serialPort1.BaudRate = Convert.ToInt32(str2);
                }

                serialPort1.Open();
                
                f_2.textBox3.AppendText("Port " + serialPort1.PortName + " Opened successful.\n");
                f_2.textBox3.SelectionStart = f_2.textBox3.Text.Length;
                f_2.textBox3.ScrollToCaret();

                toolStripStatusLabel2.Text = serialPort1.PortName;
                SerialCOM = serialPort1.PortName;
            }
            catch
            {
                f_2.textBox3.AppendText("Fail to Open COM Port " + serialPort1.PortName +"\n");
                f_2.textBox3.SelectionStart = f_2.textBox3.Text.Length;
                f_2.textBox3.ScrollToCaret();

                toolStripStatusLabel2.Text = NOPORTmsg;
            }
        }

        public void SerialPortOpen(String str1)
        {
            try
            {
                if (str1 != null)
                {
                    serialPort1.PortName = str1;
                }
                serialPort1.Open();

                f_2.textBox3.AppendText("Port " + serialPort1.PortName + " Opened successful.\n");
                f_2.textBox3.SelectionStart = f_2.textBox3.Text.Length;
                f_2.textBox3.ScrollToCaret();

                toolStripStatusLabel2.Text = serialPort1.PortName;
                SerialCOM = serialPort1.PortName;
            }
            catch
            {
                f_2.textBox3.AppendText("Fail to Open COM Port " + serialPort1.PortName + "\n");
                f_2.textBox3.SelectionStart = f_2.textBox3.Text.Length;
                f_2.textBox3.ScrollToCaret();

                toolStripStatusLabel2.Text = NOPORTmsg;
            }
        }

        public void SerialPortClose()
        {
            try
            {
                f_2.textBox3.AppendText("Close COM Port\n");
                f_2.textBox3.SelectionStart = f_2.textBox3.Text.Length;
                f_2.textBox3.ScrollToCaret();

                serialPort1.Close();
                toolStripStatusLabel2.Text = NOPORTmsg;
                SerialCOM = "";
            }
            catch
            {
            }
        }

        public void SerialPortBaudRate( String baudrate )
        {
            try
            {
                serialPort1.BaudRate = Convert.ToInt32(SerialRate);
                serialPort1.DiscardInBuffer();
                serialPort1.DiscardOutBuffer();
            }
            catch
            {
                f_2.textBox3.AppendText("Fail to Set BaudRate " + serialPort1.PortName + "\n");
                f_2.textBox3.SelectionStart = f_2.textBox3.Text.Length;
                f_2.textBox3.ScrollToCaret();
            }
        }

        public void SerialWrite(String str)
        {
            if (serialPort1.IsOpen)
            {
                try
                {
                    serialPort1.Write(str);
                }
                catch
                {
                    MessageBox.Show("Com Error");
                }
            }
            else
            {
                toolStripStatusLabel2.Text = NOPORTmsg;
            }
        }

        public void SerialWrite( byte[] buf, int off, int cnt)
        {
            if (serialPort1.IsOpen)
            {
                try
                {
                    serialPort1.Write( buf, off, cnt);
                }
                catch
                {
                    MessageBox.Show("Com Error");
                }
            }
            else
            {
                toolStripStatusLabel2.Text = NOPORTmsg;
            }
        }
        public void SerialPortFlush()
        {
            try
            {
                serialPort1.DiscardInBuffer();
                serialPort1.DiscardOutBuffer();
            }
            catch
            {
            }
        }

//============ Timer =====================================================================
        delegate void LogwriteDelegate(string str);

        void Logwrite(string str)
        {
            int MAXbuffsize = 5000;
            int DELbuffsize = 2000;
            f_2.textBox3.AppendText(str);
            if (f_2.textBox3.Text.Length > MAXbuffsize)//バッファサイズを超えた分を削除。
            {
                string bufftxt=f_2.textBox3.Text;
                while (bufftxt.Length > DELbuffsize)//バッファサイズがREDUCEDbuffsizeバイトになるまで，先頭行を削除
                {
                    int retindex=bufftxt.IndexOf("\r");//一番最初の改行位置
                    if (retindex>=0)
                    {
                        bufftxt = bufftxt.Remove(0, retindex+1);//先頭行を削除
                    }
                    else
                    {
                        bufftxt = bufftxt.Remove(0, bufftxt.Length - DELbuffsize);//改行がない場合文字数指定で削除
                    }

                }
                f_2.textBox3.Text = bufftxt;
            }
            else
            {
            }
            f_2.textBox3.SelectionStart = f_2.textBox3.Text.Length;
            f_2.textBox3.ScrollToCaret();
        }
//=========== Serial data handring timer (15ms)
        private void timer1_Tick(object sender, EventArgs e)
        {
            lock(comsync){
                if (SerialStream != "")// serial input strings
                {
                    string outstring = SerialStream;
                    SerialStream = "";
                    Invoke(new LogwriteDelegate(Logwrite), new object[] { outstring });
                }
            }
        }
//========== Serial return wait timer (100ms)
        private void timer2_Tick(object sender, EventArgs e)
        {
            CommandWaitTimer++;
        }

//=========== Preset position move (3000ms)
        private void timer3_Tick(object sender, EventArgs e)
        {
            int ExecNumber = PresetTime;
            int i;
            for ( i=0 ; i < PresetMax; i++)
            {
                if (preset_c[ExecNumber].Checked == true)
                {
                    PresetDo = true;
                    preset_r[ExecNumber].Checked = true;
                    PresetTime = ExecNumber;
                    break;
                };
                if (++ExecNumber >= PresetMax ) ExecNumber = 0;
            }
//            preset_r[PresetTime].Checked = true;
            if (++PresetTime >= PresetMax ) PresetTime = 0;

        }
//============ data file ===================================================================
        private bool readparameters( String filename )
        {
            if (Properties.Settings.Default.saved == "saved datas")
            {//read data
                preset_c[0].Checked = Properties.Settings.Default.preset_c0;
                preset_c[1].Checked = Properties.Settings.Default.preset_c1;
                preset_c[2].Checked = Properties.Settings.Default.preset_c2;
                preset_c[3].Checked = Properties.Settings.Default.preset_c3;
                preset_c[4].Checked = Properties.Settings.Default.preset_c4;
                preset_c[5].Checked = Properties.Settings.Default.preset_c5;
                preset_c[6].Checked = Properties.Settings.Default.preset_c6;
                preset_c[7].Checked = Properties.Settings.Default.preset_c7;
                preset_c[8].Checked = Properties.Settings.Default.preset_c8;
                preset_c[9].Checked = Properties.Settings.Default.preset_c9;
                preset_p[0].Text = Properties.Settings.Default.preset_p0;
                preset_p[1].Text = Properties.Settings.Default.preset_p1;
                preset_p[2].Text = Properties.Settings.Default.preset_p2;
                preset_p[3].Text = Properties.Settings.Default.preset_p3;
                preset_p[4].Text = Properties.Settings.Default.preset_p4;
                preset_p[5].Text = Properties.Settings.Default.preset_p5;
                preset_p[6].Text = Properties.Settings.Default.preset_p6;
                preset_p[7].Text = Properties.Settings.Default.preset_p7;
                preset_p[8].Text = Properties.Settings.Default.preset_p8;
                preset_p[9].Text = Properties.Settings.Default.preset_p9;
                preset_t[0].Text = Properties.Settings.Default.preset_t0;
                preset_t[1].Text = Properties.Settings.Default.preset_t1;
                preset_t[2].Text = Properties.Settings.Default.preset_t2;
                preset_t[3].Text = Properties.Settings.Default.preset_t3;
                preset_t[4].Text = Properties.Settings.Default.preset_t4;
                preset_t[5].Text = Properties.Settings.Default.preset_t5;
                preset_t[6].Text = Properties.Settings.Default.preset_t6;
                preset_t[7].Text = Properties.Settings.Default.preset_t7;
                preset_t[8].Text = Properties.Settings.Default.preset_t8;
                preset_t[9].Text = Properties.Settings.Default.preset_t9;
                SerialCOM = Properties.Settings.Default.SerialCOM;
                SerialRate = Properties.Settings.Default.SerialRate;
                f_2.Visible = Properties.Settings.Default.f_2_Visible;
                textBox2.Text = Properties.Settings.Default.movestep;
                textBox46.Text = Properties.Settings.Default.movespeed;
                SetSpeedMode = Properties.Settings.Default.SetSpeedMode;
                radioButton11.Checked = Properties.Settings.Default.movemode;
                textBox13.Text = Properties.Settings.Default.panposition;
                textBox14.Text = Properties.Settings.Default.tiltposition;
                textBox5.Text = Properties.Settings.Default.presettime;
                textBox8.Text = Properties.Settings.Default.commandset1;
                textBox9.Text = Properties.Settings.Default.commandset2;
                textBox10.Text = Properties.Settings.Default.commandset3;
                textBox11.Text = Properties.Settings.Default.commandset4;
                textBox12.Text = Properties.Settings.Default.commandset5;
                checkBox18.Checked = Properties.Settings.Default.tcpipen;
                textBox3.Text = Properties.Settings.Default.tcpipport;
                textBox15.Text = Properties.Settings.Default.auto_p1;
                textBox16.Text = Properties.Settings.Default.auto_t1;
                textBox17.Text = Properties.Settings.Default.auto_p2;
                textBox18.Text = Properties.Settings.Default.auto_t2;
            }
            else
            {//default data
                preset_c[0].Checked = false;
                preset_c[0].Checked = false;
                preset_c[1].Checked = false;
                preset_c[2].Checked = false;
                preset_c[3].Checked = false;
                preset_c[4].Checked = false;
                preset_c[5].Checked = false;
                preset_c[6].Checked = false;
                preset_c[7].Checked = false;
                preset_c[8].Checked = false;
                preset_c[9].Checked = false;
                preset_p[0].Text = "0";
                preset_p[1].Text = "0";
                preset_p[2].Text = "0";
                preset_p[3].Text = "0";
                preset_p[4].Text = "0";
                preset_p[5].Text = "0";
                preset_p[6].Text = "0";
                preset_p[7].Text = "0";
                preset_p[8].Text = "0";
                preset_p[9].Text = "0";
                preset_t[0].Text = "0";
                preset_t[1].Text = "0";
                preset_t[2].Text = "0";
                preset_t[3].Text = "0";
                preset_t[4].Text = "0";
                preset_t[5].Text = "0";
                preset_t[6].Text = "0";
                preset_t[7].Text = "0";
                preset_t[8].Text = "0";
                preset_t[9].Text = "0";
                SerialCOM = "COM1";
                SerialRate = "9600";
                f_2.Visible = true;
                textBox2.Text = "1000";
                textBox46.Text = "100";
                SetSpeedMode = false;
                radioButton11.Checked = true;
                textBox13.Text = "0";
                textBox14.Text = "0";
                textBox5.Text = "3000";
                textBox8.Text = "ci\nta5000\ntb200\ntu4666\nts4666\npa5000\npb200\npu4666\nps4666\n";
                textBox9.Text = "pp2500";
                textBox10.Text = "tp-900";
                textBox11.Text = "ps250n";
                textBox12.Text = "pp0";
                checkBox18.Checked = false;
                textBox3.Text = "9000";
                textBox15.Text = "0";
                textBox16.Text = "0";
                textBox17.Text = "0";
                textBox18.Text = "0";
            }
            return true;
         }

        private bool writeparameters(String filename)
        {
            Properties.Settings.Default.saved = "saved datas";
            Properties.Settings.Default.preset_c0 = preset_c[0].Checked;
            Properties.Settings.Default.preset_c1 = preset_c[1].Checked;
            Properties.Settings.Default.preset_c2 = preset_c[2].Checked;
            Properties.Settings.Default.preset_c3 = preset_c[3].Checked;
            Properties.Settings.Default.preset_c4 = preset_c[4].Checked;
            Properties.Settings.Default.preset_c5 = preset_c[5].Checked;
            Properties.Settings.Default.preset_c6 = preset_c[6].Checked;
            Properties.Settings.Default.preset_c7 = preset_c[7].Checked;
            Properties.Settings.Default.preset_c8 = preset_c[8].Checked;
            Properties.Settings.Default.preset_c9 = preset_c[9].Checked;
            Properties.Settings.Default.preset_p0 = preset_p[0].Text;
            Properties.Settings.Default.preset_p1 = preset_p[1].Text;
            Properties.Settings.Default.preset_p2 = preset_p[2].Text;
            Properties.Settings.Default.preset_p3 = preset_p[3].Text;
            Properties.Settings.Default.preset_p4 = preset_p[4].Text;
            Properties.Settings.Default.preset_p5 = preset_p[5].Text;
            Properties.Settings.Default.preset_p6 = preset_p[6].Text;
            Properties.Settings.Default.preset_p7 = preset_p[7].Text;
            Properties.Settings.Default.preset_p8 = preset_p[8].Text;
            Properties.Settings.Default.preset_p9 = preset_p[9].Text;
            Properties.Settings.Default.preset_t0 = preset_t[0].Text;
            Properties.Settings.Default.preset_t1 = preset_t[1].Text;
            Properties.Settings.Default.preset_t2 = preset_t[2].Text;
            Properties.Settings.Default.preset_t3 = preset_t[3].Text;
            Properties.Settings.Default.preset_t4 = preset_t[4].Text;
            Properties.Settings.Default.preset_t5 = preset_t[5].Text;
            Properties.Settings.Default.preset_t6 = preset_t[6].Text;
            Properties.Settings.Default.preset_t7 = preset_t[7].Text;
            Properties.Settings.Default.preset_t8 = preset_t[8].Text;
            Properties.Settings.Default.preset_t9 = preset_t[9].Text;
            Properties.Settings.Default.SerialCOM = SerialCOM;
            Properties.Settings.Default.SerialRate = SerialRate;
            Properties.Settings.Default.f_2_Visible = f_2.Visible;
            Properties.Settings.Default.movestep = textBox2.Text;
            Properties.Settings.Default.movespeed = textBox46.Text;
            Properties.Settings.Default.SetSpeedMode = SetSpeedMode;
            Properties.Settings.Default.movemode = radioButton11.Checked;
            Properties.Settings.Default.panposition = textBox13.Text;
            Properties.Settings.Default.tiltposition = textBox14.Text;
            Properties.Settings.Default.presettime = textBox5.Text;
            Properties.Settings.Default.commandset1 = textBox8.Text;
            Properties.Settings.Default.commandset2 = textBox9.Text;
            Properties.Settings.Default.commandset3 = textBox10.Text;
            Properties.Settings.Default.commandset4 = textBox11.Text;
            Properties.Settings.Default.commandset5 = textBox12.Text;
            Properties.Settings.Default.tcpipen = checkBox18.Checked;
            Properties.Settings.Default.tcpipport = textBox3.Text;
            Properties.Settings.Default.auto_p1 = textBox15.Text;
            Properties.Settings.Default.auto_t1 = textBox16.Text;
            Properties.Settings.Default.auto_p2 = textBox17.Text;
            Properties.Settings.Default.auto_t2 = textBox18.Text;

            Properties.Settings.Default.Save();
            return true;
        }

        //============= Command ===========================================
//check RS command return message;
        private String GetCommandReturn(String cmd, String ans, String ans2)
        {
            int stpos, endpos;
            int anslength;
            String AnsLine;
            String AnsStr;
            if (ans2 == "") ans2 = "\n";
            lock (comsync)
            {
                SerialStream2 = "";
            }
            SerialWrite(cmd + "\n");

            CommandWaitTimer = 0;
            while (true)
            {
                lock (comsync)
                {
                    stpos = SerialStream2.LastIndexOf(ans);
                }
                if ( stpos != -1 ) break;
                Application.DoEvents();
                if (CommandWaitTimer > 10)
                {
                    return "NoRet";
                }
                System.Threading.Thread.Sleep(5);
            };
            CommandWaitTimer = 0;
            while (true)
            {
                lock (comsync)
                {
                    endpos = SerialStream2.IndexOf(ans2, stpos);
                }
                if ( endpos != -1 ) break;
                Application.DoEvents();
                if (CommandWaitTimer > 10)
                {
                    return "NoRet";
                }
                System.Threading.Thread.Sleep(5);
            };
            
            lock (comsync)
            {
                AnsLine = SerialStream2.Substring(stpos);
            }
            anslength = endpos - stpos - ans.Length;
            AnsStr = AnsLine.Substring(ans.Length, anslength);
            return AnsStr;
        }


        //Binary Mode check RS command return message;
        // error return 30000
        private int CommandGetReturn(byte[] cmd, int num)//cmd 配列から num バイト
        {
            int result;
            byte[] indata = new byte[5];
            byte[] numdata = new byte[5];
            char[] dummydata = new char[5];
            binarySerial.Flush();

            SerialWrite(cmd, 0, num);

            CommandWaitTimer = 0;
            while ( binarySerial.Datasize() < 2)
            {

                if (++CommandWaitTimer > 1000)
                {
                    return -1;
                }
                Application.DoEvents(); 
                System.Threading.Thread.Sleep(1);
            };
            while (binarySerial.Datasize() > 2)
            {
                binarySerial.Read();
            };
            indata[0] = (byte)binarySerial.Read();
            indata[1] = (byte)binarySerial.Read();

            numdata[0] = indata[1];
            numdata[1] = indata[0];

            result = BitConverter.ToInt16(numdata, 0);

            return result;

        }


//Binary Mode check RS Query Pos command return message;
// error return 30000
        private bool CommandGetPos( ref int pan, ref int tilt)//cmd 配列から num バイト
        {
            byte[] indata = new byte[6];
            byte[] numdata = new byte[6];
            char[] dummydata = new char[6];
            binarySerial.Flush();
            byte[] cmd = new byte[5];
            int num;

            cmd[0] = 145;
            num = 1;
            SerialWrite(cmd, 0, num);

            CommandWaitTimer = 0;
            while (binarySerial.Datasize() < 2)
            {

                if (++CommandWaitTimer > 1000)
                {
                    return false;
                }
                Application.DoEvents();
                System.Threading.Thread.Sleep(1);
            };
            while (binarySerial.Datasize() > 2)
            {
                binarySerial.Read();
            };
            indata[0] = (byte)binarySerial.Read();
            indata[1] = (byte)binarySerial.Read();

            cmd[0] = 146;
            num = 1;
            SerialWrite(cmd, 0, num);

            CommandWaitTimer = 0;
            while (binarySerial.Datasize() < 2)
            {
                if (++CommandWaitTimer > 1000)
                {
                    return false;
                }
                Application.DoEvents();
                System.Threading.Thread.Sleep(1);
            };
            while (binarySerial.Datasize() > 2)
            {
                binarySerial.Read();
            };

            indata[2] = (byte)binarySerial.Read();
            indata[3] = (byte)binarySerial.Read();

            numdata[0] = indata[1];
            numdata[1] = indata[0];
            pan = BitConverter.ToInt16(numdata, 0);
            numdata[0] = indata[3];
            numdata[1] = indata[2];
            tilt = BitConverter.ToInt16(numdata, 0);

            return true;
        }

        //Binary Mode check RS command return message;
        private int CommandNoReturn(byte[] cmd, int num)//cmd 配列から num バイト
        {
            int result;

            SerialWrite(cmd, 0, num);

            result = 0;
            return result;
        }
//============ buttons =====================================================================
//============ button high light

        private void label7_MouseEnter(object sender, EventArgs e)
        {
            Label PointedLabel;
            PointedLabel = (Label)sender;
            PointedLabel.BackColor = Color.SkyBlue;
        }

        private void label7_MouseLeave(object sender, EventArgs e)
        {
            Label PointedLabel;
            PointedLabel = (Label)sender;
            PointedLabel.BackColor = SystemColors.ButtonHighlight;
        }
/*=============config1 ===========================================*/
        private void button2_Click_1(object sender, EventArgs e)
        {
            SerialWrite("RE\n");
        }
        private void button52_Click(object sender, EventArgs e)
        {
            SerialWrite("RP\n");
        }
        private void button53_Click(object sender, EventArgs e)
        {
            SerialWrite("RT\n");
        }
        private void button46_Click(object sender, EventArgs e)
        {
            SerialWrite("wpa\n");
        }
        private void button20_Click(object sender, EventArgs e)
        {
            SerialWrite("wpf\n");
        }
        private void button21_Click(object sender, EventArgs e)
        {
            SerialWrite("wph\n");
        }
        private void button22_Click(object sender, EventArgs e)
        {
            SerialWrite("wpq\n");
        }
        private void button23_Click(object sender, EventArgs e)
        {
            SerialWrite("wpe\n");
        }
        private void button47_Click(object sender, EventArgs e)
        {
            SerialWrite("wta\n");
        }
        private void button48_Click(object sender, EventArgs e)
        {
            SerialWrite("wtf\n");
        }
        private void button51_Click(object sender, EventArgs e)
        {
            SerialWrite("wth\n");
        }
        private void button50_Click(object sender, EventArgs e)
        {
            SerialWrite("wtq\n");
        }
        private void button49_Click(object sender, EventArgs e)
        {
            SerialWrite("wte\n");
        }
        private void button69_Click(object sender, EventArgs e)
        {
            SerialWrite("PMH\n");
        }

        private void button70_Click(object sender, EventArgs e)
        {
            SerialWrite("PMR\n");
        }

        private void button71_Click(object sender, EventArgs e)
        {
            SerialWrite("PML\n");
        }

        private void button66_Click(object sender, EventArgs e)
        {
            SerialWrite("TMH\n");
        }

        private void button67_Click(object sender, EventArgs e)
        {
            SerialWrite("TMR\n");
        }

        private void button68_Click(object sender, EventArgs e)
        {
            SerialWrite("TML\n");
        }
        private void button75_Click(object sender, EventArgs e)
        {
            SerialWrite("PHR\n");
        }

        private void button76_Click(object sender, EventArgs e)
        {
            SerialWrite("PHL\n");
        }

        private void button77_Click(object sender, EventArgs e)
        {
            SerialWrite("PHO\n");
        }

        private void button72_Click(object sender, EventArgs e)
        {
            SerialWrite("THR\n");
        }

        private void button73_Click(object sender, EventArgs e)
        {
            SerialWrite("THL\n");
        }

        private void button74_Click(object sender, EventArgs e)
        {
            SerialWrite("THO\n");
        }
        private void label41_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("WP\n");
            SerialWrite("WT\n");
        }

        private void label42_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("PM\n");
            SerialWrite("TM\n");
        }

        private void label43_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("PH\n");
            SerialWrite("TH\n");
        }
/*=============config2 ===========================================*/

        private void label23_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("l\n");
        }
        private void button54_Click(object sender, EventArgs e)//limitSW enable
        {
            SerialWrite("le\n");
        }
        private void button55_Click(object sender, EventArgs e)//limitSW disable
        {
            SerialWrite("ld\n");
        }
        private void button79_Click(object sender, EventArgs e)//Immidiate
        {
            SerialWrite("I\n");
        }

        private void button86_Click(object sender, EventArgs e)//slave
        {
            SerialWrite("S\n");
        }

        private void button87_Click(object sender, EventArgs e)//await
        {
            SerialWrite("A\n");
        }

        private void button82_Click(object sender, EventArgs e)//Resolution
        {
            f_2.Visible = true;
            SerialWrite("PR\n");
            SerialWrite("TR\n");
        }
        private void button81_Click(object sender, EventArgs e)//Position limit
        {
            f_2.Visible = true;
            SerialWrite("PN\n");
            SerialWrite("PX\n");
            SerialWrite("TN\n");
            SerialWrite("TX\n");
        }
        private void button83_Click(object sender, EventArgs e)//Version
        {
            f_2.Visible = true;
            SerialWrite("V\n");
        }
        private void button85_Click(object sender, EventArgs e)//Volt/Tempature
        {
            f_2.Visible = true;
            SerialWrite("O\n");
        }
        private void button84_Click(object sender, EventArgs e)//Unit No.
        {
            f_2.Visible = true;
            SerialWrite("U\n");
        }

        private void button12_Click(object sender, EventArgs e)//Serial speed 9600
        {
            SerialWrite("@(9600,0,F)\n");
        }

        private void button13_Click(object sender, EventArgs e)//Serial speed 19200
        {
            SerialWrite("@(19200,0,F)\n");
        }

        private void button14_Click(object sender, EventArgs e)//Serial speed 38400
        {
            SerialWrite("@(38400,0,F)\n");
        }
        private void button1_Click(object sender, EventArgs e)
        {
            SerialWrite("@(57600,0,F)\n");
        }

        private void button4_Click(object sender, EventArgs e)
        {
            SerialWrite("@(115200,0,F)\n");
        }

        private void button78_Click(object sender, EventArgs e)//Reset enable
        {
            SerialWrite("RE\n");
        }
        private void button80_Click(object sender, EventArgs e)//Reset disable
        {
            SerialWrite("RD\n");
        }
        private void button10_Click(object sender, EventArgs e)//Factory default
        {
            SerialWrite("DF\n");
        }
        private void button33_Click(object sender, EventArgs e)//setup data restore
        {
            SerialWrite("DR\n");
        }
        private void button34_Click(object sender, EventArgs e)//setup data save
        {
            SerialWrite("DS\n");
        }

//============control buttons========================================================
        private void MoveByCursol(int no)
        {
            if (!SetSpeedMode)
            {
                if (radioButton11.Checked)
                {
                    SerialWrite("CI\n");
                }
                else
                {
                    SerialWrite("PS0\n");
                    SerialWrite("TS0\n");
                    SerialWrite("CV\n");
                }
                SetSpeedMode = true;
            }
            if (radioButton11.Checked)
            {
                switch (no)
                {
                    case 0:
                        SerialWrite("pp0\n");
                        SerialWrite("tp0\n");
                        break;
                    case 1:
                        SerialWrite("PO" + textBox2.Text + "\n");
                        break;
                    case 2:
                        SerialWrite("PO-" + textBox2.Text + "\n");
                        break;
                    case 3:
                        SerialWrite("TO" + textBox2.Text + "\n");
                        break;
                    case 4:
                        SerialWrite("TO-" + textBox2.Text + "\n");
                        break;
                }
            }
            else
            {
                switch (no)
                {
                    case 0:
                        SerialWrite("ps0\n");
                        SerialWrite("ts0\n");
                        break;
                    case 1:
                        SerialWrite("Pd" + textBox46.Text + "\n");
                        break;
                    case 2:
                        SerialWrite("Pd-" + textBox46.Text + "\n");
                        break;
                    case 3:
                        SerialWrite("Td" + textBox46.Text + "\n");
                        break;
                    case 4:
                        SerialWrite("Td-" + textBox46.Text + "\n");
                        break;
                }
            }
        }

        private void radioButton11_CheckedChanged(object sender, EventArgs e)
        {
            int n;
            if (!radioButton11.Checked) return;
            SerialWrite("CI\n");
            if (int.TryParse(backupPanspeed, out n) && int.TryParse(backupTiltspeed, out n))
            {
                SerialWrite("PS" + backupPanspeed + "\n");
                SerialWrite("TS" + backupTiltspeed + "\n");
            }
        }
        private void radioButton12_CheckedChanged(object sender, EventArgs e)
        {
            if (!radioButton12.Checked) return;
            backupPanspeed = GetCommandReturn("PS", "Target Pan speed is ", " positions/sec");
            backupTiltspeed = GetCommandReturn("TS", "Target Tilt speed is ", " positions/sec");
            SerialWrite("PS0\n");
            SerialWrite("TS0\n");
            SerialWrite("CV\n");
        }

        private void button5_Click(object sender, EventArgs e)
        {
            MoveByCursol(1);
//            SerialWrite("PO100\n");
//            SerialWrite("PO"+textBox2.Text+"\n");
        }

        private void button6_Click(object sender, EventArgs e)
        {
            MoveByCursol(2);
            //            SerialWrite("PO-100\n");
//            SerialWrite("PO-" + textBox2.Text + "\n");
        }

        private void button7_Click(object sender, EventArgs e)
        {
            MoveByCursol(3);
            //            SerialWrite("TO100\n");
//            SerialWrite("TO" + textBox2.Text + "\n");
        }

        private void button8_Click(object sender, EventArgs e)
        {
            MoveByCursol(4);
            //            SerialWrite("TO-100\n");
//            SerialWrite("TO-" + textBox2.Text + "\n");
        }
        private void button24_Click(object sender, EventArgs e)
        {
            MoveByCursol(0);
//            SerialWrite("pp0\n");
//            SerialWrite("tp0\n");
        }
        private void button3_Click(object sender, EventArgs e)
        {
            SerialWrite("H\n");
            PresetPos.Enabled = false;
        }
        private void button56_Click(object sender, EventArgs e)
        {
            SerialWrite("PP" + textBox13.Text + "\n");
            SerialWrite("TP" + textBox14.Text + "\n");
        }
//現在位置取得
        private void button88_Click(object sender, EventArgs e)
        {

            SerialWrite("EE\nFV\n");
            textBox50.Text = GetCommandReturn("PP", "Current Pan position is ", "\n");
            textBox51.Text = GetCommandReturn("TP", "Current Tilt position is ", "\n");
            try
            {
            textBox50.Text = int.Parse(textBox50.Text).ToString();
            textBox51.Text = int.Parse(textBox51.Text).ToString();
            }
            catch
            {
                textBox50.Text = "";
                textBox51.Text = "";
            }
            textBox52.Text = GetCommandReturn("PS", "Target Pan speed is ", " positions/sec");
            textBox53.Text = GetCommandReturn("TS", "Target Tilt speed is ", " positions/sec");
            try
            {
                textBox52.Text = int.Parse(textBox52.Text).ToString();
                textBox53.Text = int.Parse(textBox53.Text).ToString();
            }
            catch
            {
                textBox52.Text = "";
                textBox53.Text = "";
            }
        }

//===========Preset move ===================================
        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            int CheckedNumber;

            for (CheckedNumber = 0; CheckedNumber < PresetMax; CheckedNumber++)
            {
                if (preset_r[CheckedNumber].Checked)
                {
                    preset_r[CheckedNumber].Checked = false;
                    preset_r[CheckedNumber].Focus();
                    break;
                }
            }
            if (CheckedNumber >= PresetMax)
            {
                return;
            }
            if (PresetPos.Enabled && !PresetDo) return;//Timer3 有効中はイベントは1回のみ

            RadioButton rb;
            rb = (RadioButton)sender;
//            if (==(int.Parse(rb.Text)-1));


            if (checkBox11.Checked)
            {// memory position command
                int n;
                
                if (int.TryParse(textBox41.Text, out n) && int.TryParse(textBox42.Text, out n))
                {
                    preset_p[CheckedNumber].Text = textBox41.Text;
                    preset_t[CheckedNumber].Text = textBox42.Text;
                    preset_c[CheckedNumber].Checked = true;

                }
                if (!PresetPos.Enabled)preset_r[CheckedNumber].Checked = false;
                checkBox11.Checked = false;
            }
            else
            {// playback position command
                SerialWrite("PP" + preset_p[CheckedNumber].Text + "\n");
                SerialWrite("TP" + preset_t[CheckedNumber].Text + "\n");
                PresetDo = false;
            }
        }
        private void button19_Click(object sender, EventArgs e)//動作開始
        {
            if (PresetPos.Enabled)
            {
                button19.Text = "動作開始";
                label4.Text = "Preset";
                PresetPos.Enabled = false;
            }
            else
            {
                button19.Text = "動作停止";
                PresetTime = 0;
                PresetDo = false;
                try
                {
                    PresetPos.Interval = int.Parse(textBox5.Text);
                }
                catch 
                {
                    textBox5.Text = "5000";//kwa20161026
                    PresetPos.Interval = 5000;
                }
                PresetPos.Enabled = true;
                timer3_Tick(sender, e);
            }
        }
        private void checkBox11_CheckedChanged(object sender, EventArgs e)//位置記憶
        {
            if (checkBox11.Checked)
            {
                textBox41.Text = "";
                textBox42.Text = "";
                SerialWrite("EE\nFV\n");
                textBox41.Text = GetCommandReturn("PP", "Current Pan position is ","\n");
                textBox42.Text = GetCommandReturn("TP", "Current Tilt position is ","\n");
                try
                {
                    textBox41.Text = int.Parse(textBox41.Text).ToString();
                    textBox42.Text = int.Parse(textBox42.Text).ToString();
                }
                catch
                {
                    textBox41.Text = "";
                    textBox42.Text = "";
                }
            }
//            int n;
//            if (!int.TryParse(textBox41.Text, out n) || !int.TryParse(textBox42.Text, out n))
//            {
//                checkBox11.Checked = false;
//            }
            int CheckedNumber;
            for (CheckedNumber = 0; CheckedNumber < PresetMax; CheckedNumber++)
            {
                preset_r[CheckedNumber].Checked = false;
            }
        }

//===============parameter ========================================

        private void button25_Click(object sender, EventArgs e)
        {
            SerialWrite("ts" + textBox6.Text + "\n");
        }

        private void button26_Click(object sender, EventArgs e)
        {
            SerialWrite("ps" + textBox6.Text + "\n");
        }

        private void button35_Click(object sender, EventArgs e)
        {
            SerialWrite("td" + textBox6.Text + "\n");
        }

        private void button36_Click(object sender, EventArgs e)
        {
            SerialWrite("pd" + textBox6.Text + "\n");
        }

        private void button27_Click(object sender, EventArgs e)
        {
            SerialWrite("ta" + textBox6.Text + "\n");
        }

        private void button28_Click(object sender, EventArgs e)
        {
            SerialWrite("pa" + textBox6.Text + "\n");
        }

        private void button30_Click(object sender, EventArgs e)
        {
            SerialWrite("pb" + textBox6.Text + "\n");
        }

        private void button29_Click(object sender, EventArgs e)
        {
            SerialWrite("tb" + textBox6.Text + "\n");
        }

        private void button32_Click(object sender, EventArgs e)
        {
            SerialWrite("pu" + textBox6.Text + "\n");
        }

        private void button31_Click(object sender, EventArgs e)
        {
            SerialWrite("tu" + textBox6.Text + "\n");
        }
        private void button65_Click(object sender, EventArgs e)
        {
            SerialWrite("pl" + textBox6.Text + "\n");
        }

        private void button64_Click(object sender, EventArgs e)
        {
            SerialWrite("tl" + textBox6.Text + "\n");
        }
        private void label7_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("ps\n");
            SerialWrite("ts\n");
        }

        private void label11_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("pd\n");
            SerialWrite("td\n");
        }

        private void label8_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("pa\n");
            SerialWrite("ta\n");
        }

        private void label9_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("pb\n");
            SerialWrite("tb\n");
        }

        private void label10_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("pu\n");
            SerialWrite("tu\n");
        }

        private void label29_Click(object sender, EventArgs e)
        {
            f_2.Visible = true;
            SerialWrite("pl\n");
            SerialWrite("tl\n");
        }


//=========commandset============================================================
        private void button45_Click(object sender, EventArgs e)
        {
            foreach (string str in textBox8.Lines)
            {
                SerialWrite( str+ "\n" );
            }
        }
        private void button44_Click(object sender, EventArgs e)
        {
            foreach (string str in textBox9.Lines)
            {
                SerialWrite(str + "\n");
            }
        }
        private void button43_Click(object sender, EventArgs e)
        {
            foreach (string str in textBox10.Lines)
            {
                SerialWrite(str + "\n");
            }
        }
        private void button42_Click(object sender, EventArgs e)
        {
            foreach (string str in textBox11.Lines)
            {
                SerialWrite(str + "\n");
            }
        }
        private void button41_Click(object sender, EventArgs e)
        {
            foreach (string str in textBox12.Lines)
            {
                SerialWrite(str + "\n");
            }
        }

/*=============== AutoScan =============================================*/
        private void checkBox12_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox12.Checked)
            {
                SerialWrite("EE\nFV\n");
                textBox44.Text = GetCommandReturn("PP", "Current Pan position is ", "\n");
                textBox45.Text = GetCommandReturn("TP", "Current Tilt position is ", "\n");
                try
                {
                    textBox44.Text = int.Parse(textBox44.Text).ToString();
                    textBox45.Text = int.Parse(textBox45.Text).ToString();
                }
                catch
                {
                    textBox44.Text = "";
                    textBox45.Text = "";
                }
            }
        }

        private void button59_Click(object sender, EventArgs e)
        {
            if (checkBox12.Checked)
            {// memory position command
                int n;
                if (int.TryParse(textBox44.Text, out n) && int.TryParse(textBox45.Text, out n))
                {
                    textBox15.Text = textBox44.Text;
                    textBox16.Text = textBox45.Text;
                }
                checkBox12.Checked = false;
            }
            else
            {// playback position command
                SerialWrite("PP" + textBox15.Text + "\n");
                SerialWrite("TP" + textBox16.Text + "\n");
            }

        }

        private void button62_Click(object sender, EventArgs e)
        {
            if (checkBox12.Checked)
            {// memory position command
                int n;
                if (int.TryParse(textBox44.Text, out n) && int.TryParse(textBox45.Text, out n))
                {
                    textBox17.Text = textBox44.Text;
                    textBox18.Text = textBox45.Text;
                }
                checkBox12.Checked = false;
            }
            else
            {// playback position command
                SerialWrite("PP" + textBox17.Text + "\n");
                SerialWrite("TP" + textBox18.Text + "\n");
            }

        }

        private void button57_Click(object sender, EventArgs e)
        {
            //            SerialWrite("M" + int.Parse(textBox15.Text).ToString() + "," + int.Parse(textBox17.Text).ToString() + "\n");
            SerialWrite("M" + textBox15.Text + "," + textBox17.Text + "\n");
            SerialWrite("M\n");
        }

        private void button58_Click(object sender, EventArgs e)
        {
            //            SerialWrite("M" + int.Parse(textBox15.Text).ToString() + "," + int.Parse(textBox17.Text).ToString()
            //                + "," + int.Parse(textBox15.Text).ToString() + "," + int.Parse(textBox17.Text).ToString() + "\n");
            SerialWrite("M" + textBox15.Text + "," + textBox17.Text + "," + textBox16.Text + "," + textBox18.Text + "\n");
            SerialWrite("M\n");
        }

        private void button61_Click(object sender, EventArgs e)
        {
            SerialWrite("ME\n");
        }

        private void button60_Click(object sender, EventArgs e)
        {
            SerialWrite("MD\n");
        }



/*TCP/IP=======================================================================*/
        // TCP/IP enable switch
        private void checkBox18_CheckedChanged(object sender, EventArgs e)
        {
            TCPIPserver();
        }

        private void TCPIPserver()
        {
            if (checkBox18.Checked)
            {//start
                if (th != null)
                {
                    if (server != null)
                    {
                        server.Stop();
                        server = null;
                    }
                    th.Abort();
                }
                th = new Thread(new ThreadStart(DataListener));
                th.Start();
                toolStripStatusLabel5.Text = textBox3.Text;
            }
            else
            {//stop
                if (th != null)
                {
                    if (server != null)
                    {
                        server.Stop();
                        server = null;
                    }
                    th.Abort();
                }
                toolStripStatusLabel5.Text = NOPORTmsg;
            }
        }

        // リスナースレッド
        private void DataListener()
        {
            try
            {
                while (true)
                {                    // リスナーを作成する
                    server = new System.Net.Sockets.TcpListener(IPAddress.Any, int.Parse(textBox3.Text));

                    // リスナーを開始
                    server.Start();
                    thread_message = "サーバーを開始しました";

                    // クライアントからの接続待ち
                    client = server.AcceptTcpClient();
                    thread_message = "接続しました";

                    NetworkStream ns = client.GetStream();

                    // 受信データの読み出し
                    byte[] data = new byte[100];
                    int len = ns.Read(data, 0, data.Length);
                    thread_message = "データ読み取り";
                    string s = System.Text.Encoding.UTF8.GetString(data, 0, len);
                    IP_message = s;

                    client.Close();

                    thread_message = "接続解除";

                    server.Stop();
                }
            }
            catch (System.Threading.ThreadAbortException)
            {
                //スレッドが閉じられた時に発生
                return;
            }
            catch (Exception ex)
            {
                thread_message = ex.Message;
            }
        }

        private void TCPIPcheck_Tick(object sender, EventArgs e)
        {
            if (IP_message != "")
            {
                SerialWrite(IP_message);
                IP_message = "";
            }

            if (checkBox18.Checked)
            {
                if (th == null)
                {
                    TCPIPserver();
                }
            }
        }

        private void settingSToolStripMenuItem_Click(object sender, EventArgs e)
        {

        }


//TCP/IP=end======================================================================
    }

    public class RingBuffer
    {
        int readpoint;
        int writepoint;
        int buffersize;
        int[] buffer;
        int logpoint;
        int[] log;

        public RingBuffer(int size)
        {
            buffersize = size;
            buffer = new int[buffersize];
            log = new int[buffersize];
            Flush();
        }

        public void Flush()
        {
            readpoint = writepoint = logpoint = 0;
        }

        public int Datasize()
        {
            int ans;
            if (writepoint >= readpoint)
            {
                ans = writepoint - readpoint;
            }
            else
            {
                ans = (writepoint + buffersize) - readpoint;
            }
            return ans;

        }

        public int Read()
        {
            int ans;
            if (readpoint != writepoint)
            {
                ans = buffer[readpoint];
                if (++readpoint >= buffersize)
                {
                    readpoint = 0;
                }
            }
            else
            {
                ans = 0;
            }
            return ans;
        }

        public bool Write( int dat)
        {
            int nextwrite;
            bool success;

            nextwrite = writepoint + 1;
            if (nextwrite >= buffersize)
            {
                nextwrite = 0;
            }
            if (readpoint != nextwrite )
            {
                buffer[writepoint] = dat;
                writepoint = nextwrite;
//                log[logpoint++] = dat;
                if (logpoint >= buffersize) logpoint = buffersize;
                success = true;
            }
            else
            {
                success = false;
            }
            return success;
        }
        public int Readlog( int point )
        {
            int ans;
            if ((point < logpoint)&&( point>=0))
            {
                ans = log[point];
            }
            else
            {
                ans = -1;
            }
            return ans;
        }
        public int Readlogpoint()
        {
            return logpoint;
        }


    }

}
