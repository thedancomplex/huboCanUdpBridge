using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Runtime.InteropServices;
using vxlapi_NET20;
using System.Net;
using System.Net.Sockets;

namespace helloCAN
{
    class Program
    {
        public static bool doDebug = false;
        public static bool doDebugSlow = false;
        public static int doDebugSlowi = 0;
        public static int doDebugSlowMax = 1000;
        public static byte RHY = 39;
        public static byte RHR = 40;
        public static byte RHP = 41;
        public static byte RKN = 42;
        public static byte RAP = 43;
        public static byte RAR = 44;
        public static byte LHY = 45;
        public static byte LHR = 46;
        public static byte LHP = 47;
        public static byte LKN = 48;
        public static byte LAP = 49;
        public static byte LAR = 50;
        public static byte RSP = 20;
        public static byte RSR = 21;
        public static byte RSY = 22;
        public static byte REB = 23;
        public static byte LSP = 24;
        public static byte LSR = 25;
        public static byte LSY = 26;
        public static byte LEB = 27;
        public static byte RWY = 0;
        public static byte RW1 = 1;
        public static byte RW2 = 2;
        public static byte LWY = 3;
        public static byte LW1 = 4;
        public static byte LW2 = 5;
        public static byte NKY = 6;
        public static byte NK1 = 7;
        public static byte NK2 = 8;
        public static byte WST = 38;
        public static byte RF1 = 9;
        public static byte RF2 = 10;
        public static byte RF3 = 11;
        public static byte RF4 = 12;
        public static byte RF5 = 13;
        public static byte LF1 = 14;
        public static byte LF2 = 15;
        public static byte LF3 = 16;
        public static byte LF4 = 17;
        public static byte LF5 = 18;

        public static byte SENSOR_FT_RXDF = 0x40;
        public static byte SENSOR_AD_RXDF = 0x50;

        public static byte[,] boardMonotNum = new byte[2,51];
        public static double[,] ratio = new double[5, 51];  // driven, drive, harmonic, enc, quad

        public static Socket txSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

        public static IPEndPoint sendEndPoint = new IPEndPoint(IPAddress.Parse("192.168.0.10"), 11001);
        //public static IPEndPoint sendEndPoint = new IPEndPoint(IPAddress.Parse("192.168.0.30"), 11001);

        [DllImport("kernel32.dll", SetLastError=true)]
        static extern int WaitForSingleObject(int handle, int timeOut);

        public static xlSingleChannelCAN_Port CAN_RxChannel2;
        public static xlSingleChannelCAN_Port CAN_RxChannel;
        public static Thread RxThread;
        public static Thread RxThread2;
        private enum WaitResults : int
        {
            WAIT_OBJECT_0   = 0x0,
            WAIT_ABANDONED  = 0x80,
            WAIT_TIMEOUT    = 0x102,
            INFINITE        = 0xFFFF,
            WAIT_FAILED     = 0xFFFFFFF
        }

        static void setVars()
        {
            boardMonotNum[0,0x00] = 2;
            boardMonotNum[1,0x00] = 39;

            boardMonotNum[0,0x01] = 1;
            boardMonotNum[1,0x01] = 41;

            boardMonotNum[0,0x02] = 1;
            boardMonotNum[1,0x02] = 42;

            boardMonotNum[0, 0x03] = 2;
            boardMonotNum[1, 0x03] = 43;

            boardMonotNum[0, 0x04] = 2;
            boardMonotNum[1, 0x04] = 45;

            boardMonotNum[0, 0x05] = 1;
            boardMonotNum[1, 0x05] = 47;

            boardMonotNum[0, 0x06] = 1;
            boardMonotNum[1, 0x06] = 48;

            boardMonotNum[0, 0x07] = 2;
            boardMonotNum[1, 0x07] = 49;

            boardMonotNum[0, 0x08] = 2;
            boardMonotNum[1, 0x08] = 20;

            boardMonotNum[0, 0x09] = 2;
            boardMonotNum[1, 0x09] = 22;

            boardMonotNum[0, 0x0A] = 2;
            boardMonotNum[1, 0x0A] = 24;

            boardMonotNum[0, 0x0B] = 2;
            boardMonotNum[1, 0x0B] = 26;

            boardMonotNum[0, 0x20] = 3;
            boardMonotNum[1, 0x20] = 0;

            boardMonotNum[0, 0x21] = 3;
            boardMonotNum[1, 0x21] = 3;

            boardMonotNum[0, 0x22] = 3;
            boardMonotNum[1, 0x22] = 6;

            boardMonotNum[0, 0x23] = 1;
            boardMonotNum[1, 0x23] = 38;

            boardMonotNum[0, 0x24] = 5;
            boardMonotNum[1, 0x24] = 9;

            boardMonotNum[0, 0x25] = 5;
            boardMonotNum[1, 0x25] = 14;



            // LEFT GEAR RATIO
            ratio[0, LHY] = 10;
            ratio[1, LHY] = 25;
            ratio[2, LHY] = 100;
            ratio[3, LHY] = 1000;
            ratio[4, LHY] = 4;



            ratio[0, LHR] = 324;
            ratio[1, LHR] = 1024;
            ratio[2, LHR] = 160;
            ratio[3, LHR] = 1000;
            ratio[4, LHR] = 4;



            //ratio[0, LHP] = 10;
            //ratio[1, LHP] = 16;

            ratio[0, LHP] = 16;
            ratio[1, LHP] = 20;
            ratio[2, LHP] = 160;
            ratio[3, LHP] = 1000;
            ratio[4, LHP] = 4;


            ratio[0, LKN] = 16;
            ratio[1, LKN] = 16;
            ratio[2, LKN] = 160;
            ratio[3, LKN] = 1000;
            ratio[4, LKN] = 4;

            ratio[0, LAP] = 10;
            ratio[1, LAP] = 25;
            ratio[2, LAP] = 100;
            ratio[3, LAP] = 1000;
            ratio[4, LAP] = 4;


            ratio[0, LAR] = 324;
            ratio[1, LAR] = 1024;
            ratio[2, LAR] = 100;
            ratio[3, LAR] = 1000;
            ratio[4, LAR] = 4;


            // right GEAR RATIO
            ratio[0, RHY] = 10;
            ratio[1, RHY] = 25;
            ratio[2, RHY] = 100;
            ratio[3, RHY] = 1000;
            ratio[4, RHY] = 4;



            ratio[0, RHR] = 324;
            ratio[1, RHR] = 1024;
            //ratio[0, RHR] = 10;
            //ratio[1, RHR] = 25;
            ratio[2, RHR] = 160;
            ratio[3, RHR] = 1000;
            ratio[4, RHR] = 4;



            //ratio[0, RHP] = 10;
            //ratio[1, RHP] = 16;

            ratio[0, RHP] = 16;
            ratio[1, RHP] = 20;

            ratio[2, RHP] = 160;
            ratio[3, RHP] = 1000;
            ratio[4, RHP] = 4;


            ratio[0, RKN] = 16;
            ratio[1, RKN] = 16;
            ratio[2, RKN] = 160;
            ratio[3, RKN] = 1000;
            ratio[4, RKN] = 4;

            ratio[0, RAP] = 10;
            ratio[1, RAP] = 25;
            ratio[2, RAP] = 100;
            ratio[3, RAP] = 1000;
            ratio[4, RAP] = 4;


            ratio[0, RAR] = 324;
            ratio[1, RAR] = 1024;
            ratio[2, RAR] = 100;
            ratio[3, RAR] = 1000;
            ratio[4, RAR] = 4;

            // WASTE
            ratio[0, WST] = 10;
            ratio[1, WST] = 25;
            ratio[2, WST] = 100;
            ratio[3, WST] = 1000;
            ratio[4, WST] = 4;

            // RIGHT TOP
            ratio[0, RSP] = 11;
            ratio[1, RSP] = 16;
            ratio[2, RSP] = 100;
            ratio[3, RSP] = 1000;
            ratio[4, RSP] = 4;

            ratio[0, RSR] = 1;
            ratio[1, RSR] = 1;
            ratio[2, RSR] = 100;
            ratio[3, RSR] = 1000;
            ratio[4, RSR] = 4;

            ratio[0, RSY] = 1;
            ratio[1, RSY] = 1;
            ratio[2, RSY] = 100;
            ratio[3, RSY] = 1000;
            ratio[4, RSY] = 4;

            ratio[0, REB] = 20;
            ratio[1, REB] = 24;
            ratio[2, REB] = 100;
            ratio[3, REB] = 1000;
            ratio[4, REB] = 4;


            ratio[0, RWY] = 1;
            ratio[1, RWY] = 1;
            ratio[2, RWY] = 100;
            ratio[3, RWY] = 32;
            ratio[4, RWY] = 4;

            ratio[0, RW1] = 1;
            ratio[1, RW1] = 1;
            ratio[2, RW1] = 1;
            ratio[3, RW1] = 32;
            ratio[4, RW1] = 4;


            ratio[0, RW2] = 1;
            ratio[1, RW2] = 1;
            ratio[2, RW2] = 1;
            ratio[3, RW2] = 32;
            ratio[4, RW2] = 4;

            // LEFT TOP
            ratio[0, LSP] = 11;
            ratio[1, LSP] = 16;
            ratio[2, LSP] = 100;
            ratio[3, LSP] = 1000;
            ratio[4, LSP] = 4;

            ratio[0, LSR] = 1;
            ratio[1, LSR] = 1;
            ratio[2, LSR] = 100;
            ratio[3, LSR] = 1000;
            ratio[4, LSR] = 4;

            ratio[0, LSY] = 1;
            ratio[1, LSY] = 1;
            ratio[2, LSY] = 100;
            ratio[3, LSY] = 1000;
            ratio[4, LSY] = 4;

            ratio[0, LEB] = 20;
            ratio[1, LEB] = 24;
            ratio[2, LEB] = 100;
            ratio[3, LEB] = 1000;
            ratio[4, LEB] = 4;


            ratio[0, LWY] = 1;
            ratio[1, LWY] = 1;
            ratio[2, LWY] = 100;
            ratio[3, LWY] = 32;
            ratio[4, LWY] = 4;

            ratio[0, LW1] = 1;
            ratio[1, LW1] = 1;
            ratio[2, LW1] = 1;
            ratio[3, LW1] = 32;
            ratio[4, LW1] = 4;


            ratio[0, LW2] = 1;
            ratio[1, LW2] = 1;
            ratio[2, LW2] = 1;
            ratio[3, LW2] = 32;
            ratio[4, LW2] = 4;
        }

        static void Main(string[] args)
        {
            setVars();
            Console.WriteLine("-----------");
            Console.WriteLine("Test CAN R1");

            CAN_RxChannel = new xlSingleChannelCAN_Port("HuboCAN", 1);  // receive port
            CAN_RxChannel2 = new xlSingleChannelCAN_Port("HuboCAN", 0);  // transmit channel
            
            Console.WriteLine("Created Channels");
            CAN_RxChannel2.xlActivate();  // activate channel
            CAN_RxChannel.xlActivate();
            Console.WriteLine("Channel activated");
            CAN_RxChannel2.xlPrintConfig();
            CAN_RxChannel.xlPrintConfig();

            bool runTop = true;
            bool runBottom = true;

            if (runTop)
            {
                RxThread = new Thread(new ThreadStart(rx_thread));
                RxThread.Start();       // start rx thread
                Console.WriteLine("Started RX Thread");
            }

            if (runBottom)
            {
                RxThread2 = new Thread(new ThreadStart(rx_thread2));
                RxThread2.Start();       // start rx thread
                Console.WriteLine("Started RX Thread 2");
            }
            
        }

        public static void sendUDP(byte mNum, double finalDeg, int sens1, int sens2, int sens3, int sens4)
        //public static void sendUDP(int mNum, double finalDeg)
        {
            
            if (mNum >= 0x40 & mNum < 0x50)     // FT sensors
            {
                byte[] s1 = BitConverter.GetBytes(sens1);
                byte[] s2 = BitConverter.GetBytes(sens2);
                byte[] s3 = BitConverter.GetBytes(sens3);
                //byte[] s4 = BitConverter.GetBytes(sens4);

                byte[] tFT = { (byte)mNum };

                byte[] txBuff = new byte[tFT.Length + s1.Length + s2.Length + s3.Length];
                tFT.CopyTo(txBuff, 0);
                s1.CopyTo(txBuff, tFT.Length);
                s2.CopyTo(txBuff, tFT.Length + s1.Length);
                s3.CopyTo(txBuff, tFT.Length + s1.Length + s2.Length);
                txSocket.SendTo(txBuff, sendEndPoint);
                //Console.WriteLine("*");
                if (doDebugSlow & doDebugSlowi == doDebugSlowMax)
                {
                    doDebugSlowi = 0;
                    Console.Write(".");
                }
                else
                {
                    doDebugSlowi = doDebugSlowi + 1;
                }

            }
            if (mNum >= 0x50 & mNum < 0x60)     // ACC/IMU sensors
            {
                byte[] s1 = BitConverter.GetBytes(sens1);
                byte[] s2 = BitConverter.GetBytes(sens2);
                byte[] s3 = BitConverter.GetBytes(sens3);
                byte[] s4 = BitConverter.GetBytes(sens4);

                byte[] tFT = { (byte)mNum };

                byte[] txBuff = new byte[tFT.Length + s1.Length + s2.Length + s3.Length + s4.Length];
                tFT.CopyTo(txBuff, 0);
                s1.CopyTo(txBuff, tFT.Length);
                s2.CopyTo(txBuff, tFT.Length + s1.Length);
                s3.CopyTo(txBuff, tFT.Length + s1.Length + s2.Length);
                s4.CopyTo(txBuff, tFT.Length + s1.Length + s2.Length + s3.Length);

                txSocket.SendTo(txBuff, sendEndPoint);

                if (doDebugSlow & doDebugSlowi == doDebugSlowMax)
                {
                    doDebugSlowi = 0;
                    Console.Write(".");
                }
                else
                {
                    doDebugSlowi = doDebugSlowi + 1;
                }

            }

            
            #region motor deg cmd
            if ((mNum < 53 & mNum >= 0))
            {
                /*
                if (mNum == 3 | mNum == 0)
                {
                    Console.WriteLine("Motor: " + mNum.ToString() + "  @" + finalDeg.ToString());
                }*/
                float tempDeg = (float)finalDeg;
                byte[] tempDegByte = BitConverter.GetBytes(tempDeg);

                byte[] tempMotor = { (byte)mNum };

                byte[] txBuff = new byte[tempMotor.Length + tempDegByte.Length];



                tempMotor.CopyTo(txBuff, 0);

                tempDegByte.CopyTo(txBuff, tempMotor.Length);

                txSocket.SendTo(txBuff, sendEndPoint);

                if (doDebugSlow & doDebugSlowi == doDebugSlowMax)
                {
                    doDebugSlowi = 0;
                    Console.Write(".");
                }
                else
                {
                    doDebugSlowi = doDebugSlowi + 1;
                }
           }
           #endregion 
        }

        public static int getTick(double deg, double drive, double driven, double harmonic, double enc, double quad)
        {
            int tempInt = (int)(deg * ((driven / drive) * harmonic * enc * quad) / 360.0);
            
            return tempInt;
        }
        public static double getDeg(int ticks, double drive, double driven, double harmonic, double enc, double quad)
        {
            //Console.WriteLine(ticks.ToString());
            //double theOut = (double)(ticks / ((driven / drive) * harmonic * enc * quad) * 360.0);
            double theOut = (double)(ticks / ((driven / drive) * harmonic * enc * quad) * 360.0);
            //Console.WriteLine(theOut.ToString());
            //Console.WriteLine("Gain = " + (ticks / theOut * 360).ToString());
            return theOut;// (double)(ticks / ((driven / drive) * harmonic * enc * quad) * 360.0);

        }


        public static void rx_thread()
        {
            xlSingleChannelCAN_Port rxChannel = CAN_RxChannel;
            XLClass.xl_event receiveEvent = new XLClass.xl_event();
            XLClass.XLstatus xlStatus = XLClass.XLstatus.XL_SUCCESS;
            WaitResults waitResult = new WaitResults();
            //CAN_RxChannel.xlCanAddAcceptanceRange(0x60, 0x90);
            //CAN_RxChannel.xlCanAddAcceptanceRange(2, 3);

   


            while (true)
            {
                waitResult = (WaitResults)WaitForSingleObject(rxChannel.eventHandle, 1000);  // event handler
                try
                {
                    if (waitResult != WaitResults.WAIT_TIMEOUT)
                    {
                        xlStatus = XLClass.XLstatus.XL_SUCCESS;
                        while (xlStatus != XLClass.XLstatus.XL_ERR_QUEUE_IS_EMPTY)
                        {
                            xlStatus = rxChannel.xlReceive(ref receiveEvent);

                            if (xlStatus == XLClass.XLstatus.XL_SUCCESS)
                            {
                                byte[] data = receiveEvent.tagData.can_Msg.data;

                                uint msgID = receiveEvent.tagData.can_Msg.id;
                                uint msgDLC = receiveEvent.tagData.can_Msg.dlc;
                                //if ((msgID - 0x10)==28)
                                //Console.WriteLine((msgID).ToString());
                                //Console.WriteLine(msgID.ToString());

                                
                                #region FT Sensor
                                if (msgID >= 0x40 & msgID < 0x50)// & msgDLC == 6)
                                {
                                    uint nFT = msgID - 0x40;        // FT number
                                    // note:    nFT = 1 --> right foot
                                    //          nFT = 2 --> left foot
                                    //          nFT = 6 --> right wrist

                                  
                                    int Mx = BitConverter.ToInt16(data, 0);
                                    int My = BitConverter.ToInt16(data, 2);
                                    int Fz = BitConverter.ToInt16(data, 4);

                                    sendUDP((byte)msgID, 0.0, Mx, My, Fz, 0);        // send the udp message
                                    sendUDP((byte)msgID, 0.0, Mx, My, Fz, 0); //dan
                                    sendUDP((byte)msgID, 0.0, Mx, My, Fz, 0); //dan
                                    sendUDP((byte)msgID, 0.0, Mx, My, Fz, 0); //dan
                                    if (nFT == 1)
                                    {
                                        //uint MMx = BitConverter.ToUInt16(data, 0);
                                        //Console.WriteLine("Mx = " + Mx.ToString());
                                    } 
                                }

                                if (msgID >= 0x50 & msgID < 0x60)
                                {
                                    uint nAD = msgID - 0x50;
                                    // note:    nAD = 6 --> Works but do not know which ones it is yet
                                    //          nAD = 1 --> IMU (gyro = pitch and roll (1 and 2))
                                    //          nAD = 2 --> some other data prob IMU
                                    //          nAD = 5 --> better IMU

                                    int Acc1 = BitConverter.ToInt16(data, 0);
                                    int Acc2 = BitConverter.ToInt16(data, 2);
                                    int Gyro1 = BitConverter.ToInt16(data, 4);
                                    int Gyro2 = BitConverter.ToInt16(data, 6);

                                    sendUDP((byte)msgID, 0.0, Acc1, Acc2, Gyro1, Gyro2);
                                    sendUDP((byte)msgID, 0.0, Acc1, Acc2, Gyro1, Gyro2); //dan
                                    sendUDP((byte)msgID, 0.0, Acc1, Acc2, Gyro1, Gyro2); //dan
                                    sendUDP((byte)msgID, 0.0, Acc1, Acc2, Gyro1, Gyro2); //dan

                                    if (nAD == 1)
                                    {
                                        //Console.WriteLine("Gyro2 = " + Gyro2.ToString());
                                    }
                                }
                                

                                #endregion
                                
                                #region Motor Pos
                                if (msgID >= 0x10 & msgID < 0x40) // & msgDLC == 6)
                                {
                                    uint mNum = msgID - 0x10;

                                    
                                    int[] motor = new int[boardMonotNum[0, mNum]];

                                    byte[] tempDeg = new byte[4];   // temperary hold for degree
                                    int mult = 1;                   // direction holder 
                                    double finalDeg = 0;            // final degree

                                    for (int i = 0; i < motor.Length; i++)      // fill motor numbers
                                    {

                                        motor[i] = boardMonotNum[1, mNum] + i;
                                    }

                                    for (int i = 0; i < motor.Length; i++)
                                    {

                                        if (motor.Length == 2)
                                        {
                                            //Console.Write("2 : ");
                                            tempDeg[0] = data[0 + i * 3];
                                            tempDeg[1] = data[1 + i * 3];
                                            tempDeg[2] = (byte)(data[2 + i * 3] & 0x7F);
                                            tempDeg[3] = 0;

                                            if (data[2 + i * 3] > 127)
                                            {
                                                mult = -1;
                                            }
                                            else
                                            {
                                                mult = 1;
                                            }
                                            int tempInt = BitConverter.ToInt32(tempDeg, 0);
                                            tempInt = tempInt * mult;
                                            //public static int[,] ratio = new int[5, 51];  // driven, drive, harmonic, enc, quad
                                            finalDeg = getDeg(tempInt, ratio[0, motor[i]], ratio[1, motor[i]], ratio[2, motor[i]], ratio[3, motor[i]], ratio[4, motor[i]]);// (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            //finalDeg = (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            //sendUDP(motor[i], finalDeg);
                                        }

                                        else if (motor.Length == 1)
                                        {
                                            //Console.Write("1 : ");
                                            tempDeg[0] = data[0 + i * 3];
                                            tempDeg[1] = data[1 + i * 3];
                                            tempDeg[2] = (byte)(data[2 + i * 3] & 0x7F);
                                            tempDeg[3] = 0;

                                            if (data[2 + i * 3] > 127)
                                            {
                                                mult = -1;
                                            }
                                            else
                                            {
                                                mult = 1;
                                            }
                                            int tempInt = BitConverter.ToInt32(tempDeg, 0);
                                            tempInt = tempInt * mult;
                                            //public static int[,] ratio = new int[5, 51];  // driven, drive, harmonic, enc, quad
                                            //finalDeg = (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            finalDeg = getDeg(tempInt, ratio[0, motor[i]], ratio[1, motor[i]], ratio[2, motor[i]], ratio[3, motor[i]], ratio[4, motor[i]]);
                                           
                                        }

                                        else if (motor.Length == 3)
                                        {
                                            //Console.Write("3 : ");
                                            tempDeg[0] = data[0 + i * 2];
                                            tempDeg[1] = (byte)(data[1 + i * 2] & 0x7F);
                                            tempDeg[2] = 0;
                                            tempDeg[3] = 0;

                                            if (data[1 + i * 2] > 127)
                                            {
                                                mult = -1;
                                            }
                                            else
                                            {
                                                mult = 1;
                                            }
                                            int tempInt = BitConverter.ToInt32(tempDeg, 0);
                                            tempInt = tempInt * mult;
                                            if (mult == -1)
                                            {
                                                tempInt = -(32768 + tempInt);
                                            }
                                            
                                            //public static int[,] ratio = new int[5, 51];  // driven, drive, harmonic, enc, quad
                                            finalDeg = getDeg(tempInt, ratio[0, motor[i]], ratio[1, motor[i]], ratio[2, motor[i]], ratio[3, motor[i]], ratio[4, motor[i]]);
                                            //finalDeg = (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            //sendUDP(motor[i], finalDeg);
                                        }
                                        if (doDebug)
                                            Console.Write("|");
                                        sendUDP((byte)motor[i], finalDeg,0,0,0,0);        // send the udp message
                                        sendUDP((byte)motor[i], finalDeg, 0, 0, 0, 0); //dan
                                        sendUDP((byte)motor[i], finalDeg, 0, 0, 0, 0); //dan
                                        sendUDP((byte)motor[i], finalDeg, 0, 0, 0, 0); //dan
                                    }
                                }
                                #endregion
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                }
            }
            
        }
        public static void rx_thread2()
        {
            xlSingleChannelCAN_Port rxChannel = CAN_RxChannel2;
            XLClass.xl_event receiveEvent = new XLClass.xl_event();
            XLClass.XLstatus xlStatus = XLClass.XLstatus.XL_SUCCESS;
            WaitResults waitResult = new WaitResults();
            //CAN_RxChannel.xlCanAddAcceptanceRange(0x60, 0x90);
            //CAN_RxChannel.xlCanAddAcceptanceRange(2, 3);

            while (true)
            {
                waitResult = (WaitResults)WaitForSingleObject(rxChannel.eventHandle, 1000);  // event handler
                try
                {
                    if (waitResult != WaitResults.WAIT_TIMEOUT)
                    {
                        xlStatus = XLClass.XLstatus.XL_SUCCESS;
                        while (xlStatus != XLClass.XLstatus.XL_ERR_QUEUE_IS_EMPTY)
                        {
                            xlStatus = rxChannel.xlReceive(ref receiveEvent);

                            if (xlStatus == XLClass.XLstatus.XL_SUCCESS)
                            {
                                byte[] data = receiveEvent.tagData.can_Msg.data;

                                uint msgID = receiveEvent.tagData.can_Msg.id;
                                uint msgDLC = receiveEvent.tagData.can_Msg.dlc;
                                //Console.WriteLine((msgID-0x10).ToString());
                                //Console.WriteLine(msgID.ToString());

                                if (msgID >= 0x10 & msgID < 0x61 )//& msgDLC == 6)
                                {
                                    uint mNum = msgID - 0x10;

                                    int[] motor = new int[boardMonotNum[0, mNum]];

                                    byte[] tempDeg = new byte[4];   // temperary hold for degree
                                    int mult = 1;                   // direction holder 
                                    double finalDeg = 0;            // final degree

                                    for (int i = 0; i < motor.Length; i++)      // fill motor numbers
                                    {

                                        motor[i] = boardMonotNum[1, mNum] + i;
                                    }
                                    //if (mNum == 0x23)
                                    //    Console.WriteLine(mNum.ToString());
                                    for (int i = 0; i < motor.Length; i++)
                                    {

                                        if (motor.Length == 2)
                                        {
                                            //Console.Write("2 : ");
                                            tempDeg[0] = data[0 + i * 3];
                                            tempDeg[1] = data[1 + i * 3];
                                            tempDeg[2] = (byte)(data[2 + i * 3] & 0x7F);
                                            tempDeg[3] = 0;

                                            if (data[2 + i * 3] > 127)
                                            {
                                                mult = -1;
                                            }
                                            else
                                            {
                                                mult = 1;
                                            }
                                            int tempInt = BitConverter.ToInt32(tempDeg, 0);
                                            tempInt = tempInt * mult;
                                            //public static int[,] ratio = new int[5, 51];  // driven, drive, harmonic, enc, quad
                                            finalDeg = getDeg(tempInt, ratio[0, motor[i]], ratio[1, motor[i]], ratio[2, motor[i]], ratio[3, motor[i]], ratio[4, motor[i]]);// (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            //finalDeg = (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            //sendUDP(motor[i], finalDeg);
                                        }

                                        else if (motor.Length == 1)
                                        {
                                            //Console.Write("1 : ");
                                            tempDeg[0] = data[0 + i * 3];
                                            tempDeg[1] = data[1 + i * 3];
                                            tempDeg[2] = (byte)(data[2 + i * 3] & 0x7F);
                                            tempDeg[3] = 0;

                                            if (data[2 + i * 3] > 127)
                                            {
                                                mult = -1;
                                            }
                                            else
                                            {
                                                mult = 1;
                                            }
                                            int tempInt = BitConverter.ToInt32(tempDeg, 0);
                                            tempInt = tempInt * mult;
                                            //public static int[,] ratio = new int[5, 51];  // driven, drive, harmonic, enc, quad
                                            //finalDeg = (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            finalDeg = getDeg(tempInt  , ratio[0, motor[i]], ratio[1, motor[i]], ratio[2, motor[i]], ratio[3, motor[i]], ratio[4, motor[i]]);
                                            //         getDeg(int ticks, double drive      , double driven     , double harmonic, double enc, double quad)
                                            /*
                                            if (mNum == 0x23)
                                            {
                                                // dan edit
                                                //Console.WriteLine(tempInt.ToString());
                                                //finalDeg = 25;
                                                //Console.WriteLine(finalDeg.ToString());
                                                Console.WriteLine("mNum: " + mNum.ToString().ToString() + "motor: " + motor[i].ToString() + " @: " + ratio[3, motor[i] + 1].ToString());
                                            }
                                             * */
                                             

                                        }

                                        else if (motor.Length == 3)
                                        {
                                            //Console.Write("3 : ");
                                            tempDeg[0] = data[0 + i * 2];
                                            tempDeg[1] = (byte)(data[1 + i * 2] & 0x7F);
                                            tempDeg[2] = 0;
                                            tempDeg[3] = 0;

                                            if (data[1 + i * 2] > 127)
                                            {
                                                mult = -1;
                                            }
                                            else
                                            {
                                                mult = 1;
                                            }
                                            int tempInt = BitConverter.ToInt32(tempDeg, 0);
                                            tempInt = tempInt * mult;
                                            if (mult == -1)
                                            {
                                                tempInt = -(32768 + tempInt);
                                            }
                                            /*
                                            if (motor[i] == 1)
                                            {
                                                Console.WriteLine(tempInt.ToString());
                                            }*/
                                            //public static int[,] ratio = new int[5, 51];  // driven, drive, harmonic, enc, quad
                                            finalDeg = getDeg(tempInt, ratio[0, motor[i]], ratio[1, motor[i]], ratio[2, motor[i]], ratio[3, motor[i]], ratio[4, motor[i]]);
                                            //finalDeg = (tempInt / (double)((ratio[1, motor[i]] / ratio[0, motor[i]]) * ratio[2, motor[i]] * ratio[3, motor[i]] * ratio[4, motor[i]])) * 360.0;
                                            //sendUDP(motor[i], finalDeg);
                                        }
                                        if (doDebug)
                                            Console.Write("-");
                                        sendUDP((byte)motor[i], finalDeg,0,0,0,0);        // send the udp message
                                        sendUDP((byte)motor[i], finalDeg, 0, 0, 0, 0); //dan
                                        sendUDP((byte)motor[i], finalDeg, 0, 0, 0, 0); //dan
                                        sendUDP((byte)motor[i], finalDeg, 0, 0, 0, 0); //dan
                                    }
                                }
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                }
            }

        }            
    }
}

