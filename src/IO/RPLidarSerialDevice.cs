using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using RPLidar4Net.Api.Data;
using RPLidar4Net.Api.Helpers;
using RPLidar4Net.Core;
using Serilog;

namespace RPLidar4Net.IO
{
    /// <summary>
    /// Serial connection for Slamtec RPLidar
    /// Tested on RPLidar A1 http://www.slamtec.com/en/Lidar/A1
    /// https://download.slamtec.com/api/download/rplidar-protocol/2.1.1?lang=en
    /// </summary>
    public class RPLidarSerialDevice
    {
        /// <summary>
        /// Serial Port Connection
        /// </summary>
        private SerialPort _serialPort;

        private readonly int _readWriteTimeout;

        private CancellationTokenSource _cts = new CancellationTokenSource();

        /// <summary>
        /// Connection Status
        /// </summary>
        private bool _isConnected;

        /// <summary>
        /// Motor Status
        /// </summary>
        private bool _motorRunning;

        /// <summary>
        /// Indicates the type of support for motor control
        /// </summary>
        private MotorControlSupport _motorControlSupport;

        /// <summary>
        /// Scanning Status
        /// </summary>
        private bool _isScanning;

        /// <summary>
        /// Estimated motor speed
        /// </summary>
        private double _motorSpeed;

        public double MotorSpeed
        {
            get => _motorSpeed;
            set
            {
                _motorSpeed = value;
                RaiseMotorSpeedChanged();
            }
        }

        public bool IsConnected => _isConnected;

        public event EventHandler MotorSpeedChanged;

        private void RaiseMotorSpeedChanged()
        {
            MotorSpeedChanged?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Scanning Thread
        /// </summary>
        private Thread _scanThread = null;

        public event EventHandler<NewScanEventArgs> NewScan;

        private void RaiseNewScan(IEnumerable<Point> points)
        {
            if (NewScan != null)
            {
                NewScanEventArgs eventArgs = new NewScanEventArgs(points);
                NewScan.Invoke(this, eventArgs);
            }
        }

        /// <summary>
        /// Robo Peak Lidar 360 Scanner, serial connection
        /// </summary>
        /// <param name="portName"></param>
        /// <param name="baudRate"></param>
        /// <param name="timeout"></param>
        public RPLidarSerialDevice(string portName = "com4", int baudRate = 115200, int timeout = 1000)
        {
            // Create a new SerialPort
            _serialPort = new SerialPort();
            _serialPort.PortName = portName;
            _serialPort.BaudRate = baudRate;
            //Setup RPLidar specifics
            _serialPort.Parity = Parity.None;
            _serialPort.DataBits = 8;
            _serialPort.StopBits = StopBits.One;
            // Set the read/write timeouts
            _serialPort.ReadTimeout = timeout;
            _serialPort.WriteTimeout = timeout;
            _readWriteTimeout = timeout;
        }

        /// <summary>
        /// Connect to serial port
        /// </summary>
        public void Connect()
        {
            if (this._isConnected)
            {
                Console.WriteLine("Sorry, says here that you're already connected");
            }
            else
            {
                // Connect Serial
                _serialPort.Open();
                this._isConnected = true;
                Console.WriteLine("Connected to RPLidar on {0}", _serialPort.PortName);
                _motorControlSupport = CheckMotorControlSupport();
            }
        }
        /// <summary>
        /// Disconnect Serial connection to RPLIDAR
        /// </summary>
        public void Disconnect()
        {
            if (_serialPort != null)
            {
                //Stop scan
                if (_isScanning)
                {
                    StopScan();
                }

                _serialPort.Close();
                this._isConnected = false;
            }
        }
        /// <summary>
        /// Dispose Object
        /// </summary>
        public void Dispose()
        {
            if (_serialPort != null)
            {
                if (_isConnected)
                {
                    Disconnect();
                }

                _serialPort.Dispose();

                _serialPort = null;
            }
        }
        /// <summary>
        /// Send Request to RPLidar
        /// </summary>
        /// <param name="command"></param>
        public IDataResponse SendRequest(Command command, byte[] payload = null, bool includePayloadSize = true)
        {
            if (_isConnected)
            {
                bool hasResponse = CommandHelper.GetHasResponse(command);
                if (hasResponse) //Clear input buffer of any junk
                    _serialPort.DiscardInBuffer();

                _serialPort.SendRequest(command, payload, includePayloadSize);

                //We must sleep after executing some commands
                bool sleep = CommandHelper.GetMustSleep(command);
                if (sleep) {
                    Thread.Sleep(20);
                }

                if (!hasResponse)
                    return null;

                ResponseDescriptor responseDescriptor = ReadResponseDescriptor();

                // Scan Responses are handled in the Scanning thread
                if (responseDescriptor.DataType != DataType.Scan)
                {
                    IDataResponse response = ReadDataResponse(responseDescriptor.DataResponseLength, responseDescriptor.DataType);
                    return response;
                }
            }
            return null;
        }

        private IDataResponse ReadDataResponse(uint dataResponseLength, DataType dataType)
        {
            byte[] dataResponseBytes = Read(dataResponseLength, _readWriteTimeout);
            IDataResponse response = DataResponseHelper.ToDataResponse(dataType, dataResponseBytes);
            return response;
        }

        private ResponseDescriptor ReadResponseDescriptor()
        {
            byte[] bytes = Read(Constants.ResponseDescriptorLength, _readWriteTimeout);

            string hexString = ByteHelper.ToHexString(bytes);
            Log.Information("ReadResponseDescriptor -- bytes : {@hexString}", hexString);

            ResponseDescriptor responseDescriptor = ResponseDescriptorHelper.ToResponseDescriptor(bytes);

            Log.Information("ReadResponseDescriptor -- responseDescriptor : {@responseDescriptor}", responseDescriptor);

            return responseDescriptor;
        }

        /// <summary>
        /// Start RPLidar Motor
        /// </summary>
        public void StartMotor()
        {
            if (_isConnected)
            {
                _serialPort.DtrEnable = false;

                if (_motorControlSupport != MotorControlSupport.None)
                {
                    var desiredFreq = GetDesiredRotationFrequency();

                    switch (_motorControlSupport)
                    {
                        case MotorControlSupport.RPM: SendRequest(Command.MotorSpeedControl, BitConverter.GetBytes(desiredFreq.DesiredRotationFrequencyRPM), true); break;
                        case MotorControlSupport.PWM: SendRequest(Command.SetMotorPWM, BitConverter.GetBytes(desiredFreq.DesiredRotationFrequencyPWM), true); break;
                    }
                }

                _motorRunning = true;
            }
        }

        /// <summary>
        /// Stop RPLidar Motor
        /// </summary>
        public void StopMotor()
        {
            if (_isConnected)
            {
                switch (_motorControlSupport)
                {
                    case MotorControlSupport.RPM: SendRequest(Command.MotorSpeedControl, new byte[2] { 0x00, 0x00 }, true); break;
                    case MotorControlSupport.PWM: SendRequest(Command.SetMotorPWM, new byte[2] { 0x00, 0x00 }, true); break;
                }
                _serialPort.DtrEnable = true;
                _motorRunning = false;
            }
        }

        public void SetMotorSpeed(ushort speed)
        {
            switch (_motorControlSupport)
            {
                case MotorControlSupport.RPM: SendRequest(Command.MotorSpeedControl, BitConverter.GetBytes(speed), true); break;
                case MotorControlSupport.PWM: SendRequest(Command.SetMotorPWM, BitConverter.GetBytes(speed), true); break;
            }
        }

        /// <summary>
        /// Reset RPLidar
        /// </summary>
        public void Reset()
        {
            this.SendRequest(Command.Reset);
        }
        /// <summary>
        /// Get Device Information
        /// Serial No. etc.
        /// </summary>
        public InfoDataResponse GetInfo()
        {
            return (InfoDataResponse)this.SendRequest(Command.GetInfo);
        }
        /// <summary>
        /// Get Device Health Status
        /// </summary>
        public HealthDataResponse GetHealth()
        {
            return (HealthDataResponse)this.SendRequest(Command.GetHealth);
        }
        /// <summary>
        /// Get desired rotation frequency
        /// </summary>
        public LidarConfigDataResponse GetDesiredRotationFrequency()
        {
            return (LidarConfigDataResponse)this.SendRequest(Command.GetLidarConf, CommandHelper.GetLidarConfigPayload(LidarConfigType.DesiredRotationFrequency));
        }
        public LidarConfigDataResponse GetMinRotationFrequency()
        {
            return (LidarConfigDataResponse)this.SendRequest(Command.GetLidarConf, CommandHelper.GetLidarConfigPayload(LidarConfigType.MinRotationFrequency));
        }
        public LidarConfigDataResponse GetMaxRotationFrequency()
        {
            return (LidarConfigDataResponse)this.SendRequest(Command.GetLidarConf, CommandHelper.GetLidarConfigPayload(LidarConfigType.MaxRotationFrequency));
        }
        /// <summary>
        /// Get number of ScanModes via Lidar Conf
        /// </summary>
        public int GetScanModeCount()
        {
            var response = (LidarConfigDataResponse)this.SendRequest(Command.GetLidarConf, CommandHelper.GetLidarConfigPayload(LidarConfigType.ScanModeCount));
            return response.ScanModeCount;
        }
        /// <summary>
        /// Get typical ScanMode via Lidar Conf
        /// </summary>
        public byte GetTypicalScanMode()
        {
            var response = (LidarConfigDataResponse)this.SendRequest(Command.GetLidarConf, CommandHelper.GetLidarConfigPayload(LidarConfigType.ScaneModeTypical));
            return response.TypicalScanMode;
        }
        /// <summary>
        /// Get name of ScanMode via Lidar Conf
        /// </summary>
        public string GetScanModeName(byte scanMode)
        {
            var response = (LidarConfigDataResponse)this.SendRequest(Command.GetLidarConf, CommandHelper.GetLidarConfigPayload(LidarConfigType.ScanModeName, scanMode));
            return response.ScanModeName;
        }
        /// <summary>
        /// Get answer command type of ScanMode via Lidar Conf
        /// </summary>
        public byte GetScanModeAnswerType(byte scanMode)
        {
            var response = (LidarConfigDataResponse)this.SendRequest(Command.GetLidarConf, CommandHelper.GetLidarConfigPayload(LidarConfigType.ScanModeAnsType, scanMode));
            return response.AnswerType;
        }
        /// <summary>
        /// Force Start Scanning
        /// Use with care, returns results without motor rotation synchronization
        /// </summary>
        public void ForceScan()
        {
            //Not already scanning
            if (!_isScanning)
            {
                //Have to be connected
                if (_isConnected)
                {
                    _isScanning = true;
                    //Motor must be running
                    if (!_motorRunning)
                        this.StartMotor();
                    //Start Scan
                    this.SendRequest(Command.ForceScan);
                    //Start Scan read thread
                    _scanThread = new Thread(() => ScanThread(_cts.Token));
                    _scanThread.Start();
                }
            }
        }
        /// <summary>
        /// Start Scanning
        /// </summary>
        public void StartScan()
        {
            //Not already scanning
            if (!_isScanning)
            {
                //Have to be connected
                if (_isConnected)
                {
                    _isScanning = true;
                    //Motor must be running
                    if (!_motorRunning)
                        this.StartMotor();

                    //Clear input buffer of any junk
                    _serialPort.DiscardInBuffer();
                    _serialPort.SendRequest(Command.ExpressScan, CommandHelper.GetExpressScanPayload(0), true);
                    ResponseDescriptor responseDescriptor = ReadResponseDescriptor();
                    //Start Scan
                    switch (responseDescriptor.DataType)
                    {
                        case DataType.LegacyExpressScan: _scanThread = new Thread(() => LegacyExpressScanThread(_cts.Token)); break;
                        case DataType.DenseCapsule: _scanThread = new Thread(() => DenseScanThread(_cts.Token)); break;
                        default: _scanThread = new Thread(() => ScanThread(_cts.Token)); break;
                    }
                    //Start Scan read thread
                    _scanThread.Start();
                }
            }
        }
        /// <summary>
        /// Stop Scanning
        /// </summary>
        public void StopScan()
        {
            if (this._isScanning)
            {
                _cts.Cancel();

                //Avoid thread lock with main thread/gui
                Thread myThread = new Thread(delegate ()
                {
                    this._isScanning = false;
                    _scanThread.Join();
                    this.SendRequest(Command.Stop);
                });
                myThread.Start();
                myThread.Join();

                _cts = new CancellationTokenSource();
                return;
            }
        }
        
        /// <summary>
        /// Waits for Serial Data
        /// </summary>
        /// <param name="responseLength"></param>
        /// <param name="timeout">Timeout in milliseconds</param>
        /// <returns></returns>
        public byte[] Read(UInt32 responseLength, int timeout)
        {
            try
            {
                byte[] data = _serialPort.Read((int)responseLength, timeout);
                return data;
            }
            catch (Exception e)
            {
                Log.Error(e, "Read()");
                Console.WriteLine("Error in Read(): " + e.Message);
                //Set connection status
                this._isConnected = false;
                //Then go through the motions
                this.Disconnect();

                return new byte[0];
            }
        }
        
        private enum MotorControlSupport { None, PWM, RPM };
        private MotorControlSupport CheckMotorControlSupport()
        {
            if (_isConnected)
            {
                var deviceInfo = GetInfo();
                byte.TryParse(deviceInfo.ModelID, out var modelId);
                var majorId = modelId >> 4;
                if (majorId >= Constants.BUILTIN_MOTORCTL_MINUM_MAJOR_ID)
                    return MotorControlSupport.RPM;
                else if (majorId >= Constants.A2A3_LIDAR_MINUM_MAJOR_ID)
                {
                    try
                    {
                        var accBoardDataResponse = (AccBoardDataResponse)SendRequest(Command.GetAccBoardFlag, new byte[4] {0,0,0,0});
                        if (accBoardDataResponse.IsSupported)
                            return MotorControlSupport.PWM;
                    }
                    catch 
                    {
                        //special case for C1 that does actually have motorcontrol but does not correctly answer the AccBoardFlag request
                        return MotorControlSupport.RPM;
                    }
                }
            }

            return MotorControlSupport.None;
        }

        /// <summary>
        /// Thread used for scanning
        /// Populates a list of Measurements, and adds that list to 
        /// </summary>
        public void ScanThread(CancellationToken ct)
        {
            DateTime lastStartFlagDateTime = new DateTime();
            var fullScan = new List<Point>();
            _responseBytePos = 0;

            try
            {
                //Loop while we're scanning
                while (this._isScanning && !ct.IsCancellationRequested)
                {
                    var newPoints = ReadScanDataResponses();

                    foreach (var point in newPoints)
                    {
                        //Check for new 360 degree scan bit
                        if (point.StartFlag)
                        {
                            MotorSpeed = (1 / (DateTime.Now - lastStartFlagDateTime).TotalSeconds) * 60;
                            lastStartFlagDateTime = DateTime.Now;

                            RaiseNewScan(fullScan);

                            fullScan = new List<Point>();
                        }

                        if (point.IsValid)
                            fullScan.Add(point);
                    }
                }
            }
            catch
            {
                //Set connection status
                this._isConnected = false;
                //Then go through the motions
                this.Disconnect();
            }
        }

        const int _incomingBufferSize = 8192;
        byte[] _currentBytes = new byte[_incomingBufferSize];
        byte[] _responseBytes = new byte[Constants.ScanDataResponseLength];
        int _responseBytePos;
        private IEnumerable<Point> ReadScanDataResponses()
        {
            var bytesRead = _serialPort.Read(_currentBytes, 0, _incomingBufferSize);
            var currentBytePos = 0;

            while (_isScanning && currentBytePos < bytesRead)
            {
                var currentByte = _currentBytes[currentBytePos];
                currentBytePos += 1;

                switch (_responseBytePos)
                {
                    case 0://Byte 0, sync bit and its inverse
                        int tmp = (currentByte >> 1);
                        int result = (tmp ^ currentByte) & 0x1;
                        if (result == 1)
                        {
                            //pass
                        }
                        else
                        {
                            continue;
                        }
                        break;
                    case 1://Expect the highest bit to be 1
                        int tmp2 = currentByte & Constants.RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
                        if (tmp2 == 1)
                        {
                            //pass
                        }
                        else
                        {
                            _responseBytePos = 0;
                            continue;
                        }
                        break;
                }
                _responseBytes[_responseBytePos++] = currentByte;

                if (_responseBytePos == Constants.ScanDataResponseLength)
                {
                    _responseBytePos = 0;
                    yield return ScanDataResponseHelper.ToPoint(ScanDataResponseHelper.ToScanDataResponse(_responseBytes));
                }
            }
        }

        private void LegacyExpressScanThread(CancellationToken ct)
        {
            var rawBuf = new byte[Protocol.EXPRESS_CAPSULE_SIZE];
            var pending = new List<Point>(5500);
            bool hasPrev = false;
            bool needRead = true; // false while re-syncing byte-by-byte
            Protocol.LegacyExpressCapsule prev = default;

            // Re-use a scratch list to avoid per-capsule allocation inside the hot path.
            var measurements = new List<Protocol.Measurement>(32);

            while (!ct.IsCancellationRequested)
            {
                try
                {
                    if (needRead)
                    {
                        rawBuf = Read((uint)rawBuf.Length, _readWriteTimeout);
                        if (ct.IsCancellationRequested) break;
                    }
                    needRead = true; // default: fetch a fresh capsule next iteration

                    var curr = Protocol.ParseExpressCapsule(rawBuf);
                    if (!curr.Valid)
                    {
                        // Sync lost — shift 1 byte and fill the tail with one new byte, then
                        // retry parsing WITHOUT calling ReadExact (needRead stays false).
                        Array.Copy(rawBuf, 1, rawBuf, 0, rawBuf.Length - 1);
                        rawBuf[rawBuf.Length - 1] = (byte)_serialPort.ReadByte();
                        hasPrev = false; // reset capsule pair — need fresh aligned pair
                        needRead = false;
                        continue;
                    }

                    // Need two consecutive capsules to decode angles.
                    if (!hasPrev)
                    {
                        prev = curr;
                        hasPrev = true;
                        continue;
                    }

                    measurements.Clear();
                    Protocol.DecodeLegacyExpressCapsulePair(in prev, in curr, measurements);

                    // Primary: flush on revolution boundary.
                    // IsNewScan header bit is the canonical signal; also catch wrap-around
                    // geometrically (start angle drops by > 180°) for firmware that doesn't
                    // set the bit reliably.
                    bool angleWrap = curr.StartAngleDeg < prev.StartAngleDeg - 180f;
                    bool flushNow = (curr.IsNewScan || prev.IsNewScan || angleWrap) && pending.Count > 0;
                    // Fallback: publish if a full revolution's worth of points accumulates
                    // without triggering the primary condition (~32 meas/capsule × ~42 caps/rev).
                    bool flushFallback = pending.Count >= 1500;
                    if (flushNow || flushFallback)
                    {
                        RaiseNewScan(pending);
                        pending.Clear();
                    }

                    prev = curr;

                    foreach (var m in measurements)
                    {
                        pending.Add(new Point() { Angle = m.Angle, Distance = m.Distance, Quality = m.Quality, StartFlag = m.IsNewScan });
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine("Error in LegacyExpressScanThread(): " + e.Message);
                    break;
                }
            }
        }

        private void DenseScanThread(CancellationToken ct)
        {
            var rawBuf = new byte[Protocol.EXPRESS_CAPSULE_SIZE]; // 84 bytes, same size
            var pending = new List<Point>(5500);
            bool hasPrev = false;
            bool needRead = true;
            Protocol.DenseCapsule prev = default;

            var measurements = new List<Protocol.Measurement>(40);

            while (!ct.IsCancellationRequested)
            {
                try
                {
                    if (needRead)
                    {
                        rawBuf = Read((uint)rawBuf.Length, _readWriteTimeout);
                        if (ct.IsCancellationRequested) break;
                    }
                    needRead = true;

                    var curr = Protocol.ParseDenseCapsule(rawBuf);
                    if (!curr.Valid)
                    {
                        Array.Copy(rawBuf, 1, rawBuf, 0, rawBuf.Length - 1);
                        rawBuf[rawBuf.Length - 1] = (byte)_serialPort.ReadByte();
                        hasPrev = false;
                        needRead = false;
                        continue;
                    }

                    if (!hasPrev)
                    {
                        prev = curr;
                        hasPrev = true;
                        continue;
                    }

                    measurements.Clear();
                    Protocol.DecodeDenseCapsulePair(in prev, in curr, measurements);

                    // In dense capsule format the IsNewScan header bit signals an encoder
                    // reset (per Slamtec SDK), not a revolution boundary. Use angle wrap-around
                    // instead: when the new capsule's start angle is more than 180° behind the
                    // previous one the sensor has completed a revolution.
                    bool angleWrap = curr.StartAngleDeg < prev.StartAngleDeg - 180f;
                    bool flushNow = angleWrap && pending.Count > 0;
                    bool flushFallback = pending.Count >= 1500;
                    if (flushNow || flushFallback)
                    {
                        RaiseNewScan(pending);
                        pending.Clear();
                    }

                    prev = curr;

                    foreach (var m in measurements)
                    {
                        pending.Add(new Point() { Angle = m.Angle, Distance = m.Distance, Quality = m.Quality, StartFlag = m.IsNewScan });
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine("Error in LegacyExpressScanThread(): " + e.Message);
                    break;
                }
            }
        }
    }
}
