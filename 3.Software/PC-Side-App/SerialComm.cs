using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.Windows;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    internal class SerialComm
    {
        public static SerialPort _serialPort;

        public bool connectionLost = false;
        public static void Serial_Comm()
        {
            _serialPort = new SerialPort();
            _serialPort.PortName = "COM9";//Set your board COM
            _serialPort.BaudRate = 9600;  //set your controller baundrate to use communicate
            _serialPort.Close(); // Seri portu kapatın
            _serialPort.RtsEnable = false; // Gücü kesmek için RTS sinyalini devre dışı bırakın
            _serialPort.DtrEnable = false; // Gücü kesmek için DTR sinyalini devre dışı bırakın
            Thread.Sleep(500); // Belirli bir süre bekleyin
            _serialPort.Open();
            _serialPort.RtsEnable = true;
            _serialPort.DtrEnable = true;
            Thread.Sleep(500);
        }

        public void SerialSend(string dataToSend)
        {
            _serialPort.WriteLine(dataToSend);
        }

        public string SerialRead()
        {
            return _serialPort.ReadExisting();
        }

        //public void SerialCommThread() 
        //{ 

        //    while (true)
        //    {
        //        try
        //        {
        //            string handshake = "handshake"; //piconun çalıştığını ve veri almaya hazır olduğunu doğrulamak için "handshake" string'i belirledik
        //            _serialPort.WriteLine(handshake); //konsola "handshake" komutu gönderir
        //            Console.WriteLine(_serialPort.ReadExisting()); //not need to write handshake status to console; so we write just values comes from pico we_1,enc1,e1 etc.
                    
        //        }
        //        catch
        //        {
        //            connectionLost = true; //if there is not connection between pico and pc
        //        }

        //        if (connectionLost)
        //        {
        //            try
        //            {
        //                _serialPort = new SerialPort();
        //                _serialPort.PortName = "COM9";//Set your board COM
        //                _serialPort.BaudRate = 9600;  //set your controller baundrate to use communicate
        //                _serialPort.Close(); // Seri portu kapatın
        //                _serialPort.RtsEnable = false; // Gücü kesmek için RTS sinyalini devre dışı bırakın
        //                _serialPort.DtrEnable = false; // Gücü kesmek için DTR sinyalini devre dışı bırakın
        //                Thread.Sleep(500); // Belirli bir süre bekleyin
        //                _serialPort.Open();
        //                _serialPort.RtsEnable = true;
        //                _serialPort.DtrEnable = true;
        //                string handshake = "handshake"; //piconun çalıştığını ve veri almaya hazır olduğunu doğrulamak için
        //                _serialPort.WriteLine(handshake);
        //                connectionLost = false;
        //            }
        //            catch
        //            {

        //            }
        //            Thread.Sleep(1000); // Belirli bir süre bekleyin
        //        }
        //        Thread.Sleep(50);
        //    }
        //}

    }
}
