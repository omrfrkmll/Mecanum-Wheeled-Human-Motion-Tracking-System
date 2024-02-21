//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
using System;
using System.Collections;
using System.IO;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using Microsoft.Kinect;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        private int personIndex = 0;

        Thread mainThread;

        SerialPort _serialPort;

        SerialComm serialComm;

        bool connectionLost = false;

        double velocityOutput;

        double angleOutput;


        //private PidController angleController;
        //private PidController distanceController;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            try
            {
                Serial_Comm();
            }
            catch
            {

            }
            

            mainThread = new Thread(new ThreadStart(MainLoop));
            mainThread.Start();

            
        //    angleController = new PidController(1.0,0.0,0.0,5,-5);
        //    double angleOut = angleController.ControlVariable();
        }

        //------------------------------------ÖMER FARUK TARAFINDAN EKLEME: PID KONTROL & SERIAL COMM KODLARI--------------------------------//

        string handshake = "handshake"; //piconun çalıştığını ve veri almaya hazır olduğunu doğrulamak için "handshake" string'i belirledik

        private void MainLoop()
        {
            while (true)
            {
                // Serial Communication Part
                try
                {
                    SerialSend(handshake); //konsola "handshake" komutu gönderir
                    Thread.Sleep(5);
                    SerialSend("V=" + ((int)(velocityOutput)).ToString());
                    Console.Write("VELO: " + ((int)(velocityOutput)).ToString() + " ");
                    Thread.Sleep(5);
                    SerialSend("w=" + ((int)(angleOutput)).ToString());
                    Console.Write(" ROT VELO: " + ((int)(angleOutput)).ToString() + " ");
                    Thread.Sleep(5);
                    Console.WriteLine(SerialRead()); //not need to write handshake status to console; so we write just values comes from pico we_1,enc1,e1 etc.

                }
                catch
                {
                    connectionLost = true; //if there is not connection between pico and pc
                }

                if  (connectionLost)
                {
                    try
                    {
                        Serial_Comm();
                        connectionLost = false;
                    }
                    catch
                    {

                    }
                    Thread.Sleep(1000); // Belirli bir süre bekleyin
                }


                Console.WriteLine("Distance with Hip: " + this.DistanceWithHip.ToString("0.00") + " Angle with Hip: " + this.AngleWithHip.ToString("0.00"));
                Thread.Sleep(50);
            }
        }


        // SERIAL COMMUNICATION DEFINITION
        
        public void Serial_Comm()
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
            _serialPort.WriteLine("handshake");
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

        public double DistanceWithHip { get; private set; }
        //Robotun V değişkenindeki PID kontrol kodları
        private PidController pidController;

        /// <summary>
        /// set a distance value for between robot and tracked person
        /// </summary>
        public double SetDistanceWithHip = 200;

        public void PidController()
        {
            //InitializeComponent();

            // PidController'ı oluştur ve ayarlarını yap
            double gainProportional = 0.1;
            double gainIntegral = 0;
            double gainDerivative = 0;
            double outputMax = 15; //robotun max V hızını buraya girelim
            double outputMin = -15;

            pidController = new PidController(gainProportional, gainIntegral, gainDerivative, outputMax, outputMin);

            // Hedef değeri ayarla
            pidController.SetPoint = SetDistanceWithHip;

            // PID kontrolcüsünün sonucunu her saniye güncellemek için bir zamanlayıcı başlat
            System.Windows.Threading.DispatcherTimer timer = new System.Windows.Threading.DispatcherTimer();
            timer.Interval = TimeSpan.FromSeconds(1);
            timer.Tick += Timer_Tick;
            timer.Start();
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            // distanceWithHip değerini PID kontrolcüsüne gönder
            pidController.ProcessVariable = DistanceWithHip;

            TimeSpan deltaTime = TimeSpan.FromSeconds(0.5); // Kontrol döngüsü aralığı "second"

            // Çıkış değerini hesapla
            velocityOutput = pidController.ControlVariable(deltaTime);

            // Sonucu ekrana yazdır
            //_serialPort.WriteLine($"V={output}");
            //Console.WriteLine($"V={velocityOutput}");
        }
        //Robotun V değişkenindeki PID kontrol kodları



        public double AngleWithHip { get; private set; }
        //Robotun w değişkenindeki PID kontrol kodları
        private PidController anglePidController;
        
        /// <summary>
        /// Set an angle value for the robot
        /// </summary>
        public double SetAngleWithHip = 0;

        public void AnglePidController()
        {
            // AnglePidController'ı oluştur ve ayarlarını yap
            double angleGainProportional = 1.0;
            double angleGainIntegral = 0;
            double angleGainDerivative = 0;
            double angleOutputMax = 15;
            double angleOutputMin = -15;

            anglePidController = new PidController(angleGainProportional, angleGainIntegral, angleGainDerivative, angleOutputMax, angleOutputMin);

            // Hedef açı değerini ayarla
            anglePidController.SetPoint = SetAngleWithHip;

            // PID kontrolcüsünün sonucunu her saniye güncellemek için bir zamanlayıcı başlat
            System.Windows.Threading.DispatcherTimer angleTimer = new System.Windows.Threading.DispatcherTimer();
            angleTimer.Interval = TimeSpan.FromSeconds(1);
            angleTimer.Tick += AngleTimer_Tick;
            angleTimer.Start();
        }

        private void AngleTimer_Tick(object sender, EventArgs e)
        {
            // angleWithHip değerini PID kontrolcüsüne gönder
            anglePidController.ProcessVariable = AngleWithHip;

            TimeSpan deltaTime = TimeSpan.FromSeconds(0.5); // Kontrol döngüsü aralığı "second"

            // Çıkış değerini hesapla
            angleOutput = anglePidController.ControlVariable(deltaTime);
            // Sonucu ekrana yazdır
            //_serialPort.WriteLine($"w={angleOutput}");
            //Console.WriteLine($"w={angleOutput}");
        }
        //Robotun w değişkenindeki PID kontrol kodları

        //------------------------------------ÖMER FARUK TARAFINDAN EKLEME: PID KONTROL & SERIAL COMM KODLARI--------------------------------//

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            //Task.Run(() => SerialComm.Serial_Comm());
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
                mainThread.Abort();
            }
        }





        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                //-----------------------------------------------HERE IS OUR CODES---------------------------------------------------//
                
                //bool skeletonDetected = false; // İskeletin varlığını takip etmek için bir bayrak (flag) kullanacağız
                if (skeletons.Length != 0)
                {
                    float hipX = 0;
                    float hipY = 0;
                    float hipZ = 0;
                    double distanceWithHip = 0;
                    double angleWithHip = 0;
                    //skeletonDetected = true; // İskelet tespit edildiğini işaretleyelim
                    ArrayList skeletonSequence = new ArrayList();
                    
                    
                    for (int i = 0; i < skeletons.Length; i++)
                    {
                        //Console.WriteLine("skeletons.Length: " + skeletons.Length.ToString());
                        if (skeletons[i].TrackingState == SkeletonTrackingState.Tracked)
                        {
                            skeletonSequence.Add(i);
                        }
                        

                        RenderClippedEdges(skeletons[i], dc);

                        if (skeletons[i].TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skeletons[i], dc);
                        }
                        else if (skeletons[i].TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skeletons[i].Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }

                    }

                    if (skeletonSequence.Count > 0)
                    {
                        if (!skeletonSequence.Contains(personIndex))
                            personIndex = (int)(skeletonSequence[0]);

                        //Console.WriteLine("skeletons.Length: " + skeletons.Length.ToString());
                        if (skeletons[personIndex].TrackingState == SkeletonTrackingState.Tracked)
                        {
                            //Console.WriteLine("SKELETON TRACKED: " + personIndex.ToString());
                            hipX = skeletons[personIndex].Joints[JointType.HipCenter].Position.X * 100;
                            hipY = skeletons[personIndex].Joints[JointType.HipCenter].Position.Y * 100;
                            hipZ = skeletons[personIndex].Joints[JointType.HipCenter].Position.Z * 100;
                            distanceWithHip = Math.Sqrt(hipX * hipX + hipZ * hipZ);
                            angleWithHip = Math.Atan2(hipX, hipZ) * 180 / Math.PI;
                            this.DistanceWithHip = distanceWithHip;
                            this.AngleWithHip = angleWithHip;
                            PidController();
                            AnglePidController();
                            if (skeletons[personIndex].Joints[JointType.HipCenter].TrackingState == JointTrackingState.Tracked)
                            {
                                this.statusBarText.Text = "HIP: X:" + hipX.ToString("0.00") + "     Y:" + hipY.ToString("0.00") + "  Z:" + hipZ.ToString("0.00") + "\n    D:" + distanceWithHip.ToString("0.00") + "      A:" + angleWithHip.ToString("0.00");
                                //Console.WriteLine("PERSON: " + personIndex.ToString() + " " + hipX.ToString() + hipY.ToString() + hipZ.ToString() + distanceWithHip.ToString() + angleWithHip.ToString());
                                
                            }
                            else if (skeletons[personIndex].Joints[JointType.HipCenter].TrackingState == JointTrackingState.NotTracked)
                            {
                                hipX = 0;
                                hipY = 0;
                                hipZ = 0;
                                distanceWithHip = 0;
                                angleWithHip = 0;
                                // İskelet olmadığını belirten bir mesaj gösterilebilir.
                                this.statusBarText.Text = "No skeleton detected. HIP: " + hipX.ToString("0.00") + " " + hipY.ToString("0.00") + " " + hipZ.ToString("0.00") + " " + distanceWithHip.ToString("0.00") + " " + angleWithHip.ToString("0.00");
                                //Console.WriteLine("PERSON: " + personIndex.ToString() + " " + hipX.ToString() + hipY.ToString() + hipZ.ToString() + distanceWithHip.ToString() + angleWithHip.ToString());
                            }

                        }
                    }
                    

                    

                    //foreach (Skeleton skel in skeletons)
                    //{
                    //    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    //    {
                    //        hipX = skel.Joints[JointType.HipCenter].Position.X * 100;
                    //        hipY = skel.Joints[JointType.HipCenter].Position.Y * 100;
                    //        hipZ = skel.Joints[JointType.HipCenter].Position.Z * 100;
                    //        distanceWithHip = Math.Sqrt(hipX * hipX + hipZ * hipZ);
                    //        angleWithHip = Math.Atan2(hipX, hipZ) * 180 / Math.PI;

                    //        if (skel.Joints[JointType.HipCenter].TrackingState == JointTrackingState.Tracked)
                    //        {
                    //            this.statusBarText.Text = "HIP: X:" + hipX.ToString("0.00") + "     Y:" + hipY.ToString("0.00") + "  Z:" + hipZ.ToString("0.00") + "\n    D:" + distanceWithHip.ToString("0.00") + "      A:" + angleWithHip.ToString("0.00");
                    //            Console.WriteLine(hipX.ToString() + hipY.ToString() + hipZ.ToString() + distanceWithHip.ToString() + angleWithHip.ToString());
                    //        }
                    //        else if (skel.Joints[JointType.HipCenter].TrackingState == JointTrackingState.NotTracked)
                    //        {
                    //            hipX = 0;
                    //            hipY = 0;
                    //            hipZ = 0;
                    //            distanceWithHip = 0;
                    //            angleWithHip = 0;
                    //            // İskelet olmadığını belirten bir mesaj gösterilebilir.
                    //            this.statusBarText.Text = "No skeleton detected. HIP: " + hipX.ToString("0.00") + " " + hipY.ToString("0.00") + " " + hipZ.ToString("0.00") + " " + distanceWithHip.ToString("0.00") + " " + angleWithHip.ToString("0.00");
                    //            Console.WriteLine(hipX.ToString() + hipY.ToString() + hipZ.ToString() + distanceWithHip.ToString() + angleWithHip.ToString());
                    //        }

                    //        // CONTROL ALGORTIHM HERE.



                    //        //if (skeletons.Length != 0)
                    //        //{
                    //        //float skelX = (skeletons[0].Position.X) * 100;
                    //        //float skelY = (skeletons[0].Position.Y) * 100;
                    //        //float skelZ = (skeletons[0].Position.Z) * 100;
                    //        //double distanceWithSkel = Math.Sqrt(skelX * skelX + skelZ * skelZ);
                    //        //double angleWithSkel = (Math.Atan2(skelX, skelZ)) * 180 / Math.PI;
                    //        //float hipX = (skeletons[0].Joints[JointType.HipCenter].Position.X) * 100;
                    //        //float hipY = (skeletons[0].Joints[JointType.HipCenter].Position.Y) * 100;
                    //        //float hipZ = (skeletons[0].Joints[JointType.HipCenter].Position.Z) * 100;
                    //        //double distanceWithHip = Math.Sqrt(hipX * hipX + hipZ * hipZ);
                    //        //double angleWithHip = (Math.Atan2(hipX, hipZ)) * 180 / Math.PI;
                    //        //if (skeletons[0].Joints[JointType.HipCenter].TrackingState == JointTrackingState.Tracked)
                    //        //{
                    //        //    this.statusBarText.Text = "HIP:" + " X:" + hipX.ToString("0.00")  + "    Y:" +  hipY.ToString("0.00") +  "   Z:\n" +  hipZ.ToString("0.00") + "    D:" + distanceWithHip.ToString("0.00") + "  A:" + angleWithHip.ToString("0.00");
                    //        //}
                    //        //else
                    //        //{
                    //        //    this.statusBarText.Text = "SKEL: " + skelX.ToString("0.00") + " " + skelY.ToString("0.00") + " " + skelZ.ToString("0.00") + " " + distanceWithSkel.ToString("0.00") + " " + angleWithSkel.ToString("0.00");
                    //        //}

                    //        // CONTROL ALGORTIHM HERE.
                    //    }

                    //    //else if (skel.TrackingState == SkeletonTrackingState.NotTracked)
                    //    //{
                    //    //    // İskelet yoksa değerleri sıfırla
                    //    //    float hipX = 0;
                    //    //    float hipY = 0;
                    //    //    float hipZ = 0;
                    //    //    double distanceWithHip = 0;
                    //    //    double angleWithHip = 0;

                    //    //    if (skel.Joints[JointType.HipCenter].TrackingState == JointTrackingState.NotTracked)
                    //    //    {
                    //    //        this.statusBarText.Text = "HIP: " + hipX.ToString("0.00") + " " + hipY.ToString("0.00") + " " + hipZ.ToString("0.00") + " " + distanceWithHip.ToString("0.00") + " " + angleWithHip.ToString("0.00");
                    //    //    }
                    //    //}







                    //    //this.statusBarText.Text = "SKEL: " + skelX.ToString("0.00") + " " + skelY.ToString("0.00") + " " + skelZ.ToString("0.00") + " " + distanceWithSkel.ToString("0.00") + " " + angleWithSkel.ToString("0.00");
                    //    //this.statusBarText.Text = this.statusBarText.Text + "HIP: " + hipX.ToString("0.00") + " " + hipY.ToString("0.00") + " " + hipZ.ToString("0.00") + " " + distanceWithHip.ToString("0.00") + " " + angleWithHip.ToString("0.00");

                    //    //foreach (Skeleton skel in skeletons)
                    //    //{
                    //    RenderClippedEdges(skel, dc);

                    //    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    //    {
                    //        this.DrawBonesAndJoints(skel, dc);
                    //    }
                    //    else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                    //    {
                    //        dc.DrawEllipse(
                    //        this.centerPointBrush,
                    //        null,
                    //        this.SkeletonPointToScreen(skel.Position),
                    //        BodyCenterThickness,
                    //        BodyCenterThickness);
                    //    }

                    //}


                }
                

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }
    }

}