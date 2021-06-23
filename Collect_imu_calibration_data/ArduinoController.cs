using System;
using System.IO.Ports;
using System.Threading;
using System.Collections.Generic;
using System.Text;

namespace CommunicationProtocol
{
    public enum Parameters : byte
    {
        None = 0,
        Mode = 1,
    }

    public enum Modes : byte
    {
        RawData = 0,
        RawDataSimpleCalc = 1,
        QuaternionsWithotNorm = 2,
        Quaternions = 3,
        MedgwickFilter = 4,
    }

    public enum Commands : byte
    {
        None = 0,
        GetState = 1,
        SetParameter = 2,
        GetQuaternions = 3,
        GetRawData = 4,
    }

    public enum RetCodes : byte
    {
        OK = 0,
    }

    public enum AccelRange : byte
    {
        ACCEL_RANGE_2G,
        ACCEL_RANGE_4G,
        ACCEL_RANGE_8G,
        ACCEL_RANGE_16G
    };

    public enum GyroRange : byte
    {
        GYRO_RANGE_250DPS,
        GYRO_RANGE_500DPS,
        GYRO_RANGE_1000DPS,
        GYRO_RANGE_2000DPS
    };

    public class RawData
    {
        public Int16 ax;
        public Int16 ay;
        public Int16 az;
        public Int16 gx;
        public Int16 gy;
        public Int16 gz;
        public Int16 mx;
        public Int16 my;
        public Int16 mz;
        public UInt32 time;

        public string Print()
        {
            return string.Format("{0}, [{1}, {2}, {3}], [{4}, {5}, {6}], [{7}, {8}, {9}]",
                time, ax, ay, az, gx, gy, gz, mx, my, mz);
        }


    }

    public class CalibratedData
    {
        public float ax;
        public float ay;
        public float az;
        public float gx;
        public float gy;
        public float gz;
        public float mx;
        public float my;
        public float mz;
        public UInt32 time;
    }

    public class SensorQuaternion
    {
        public float w;
        public float x;
        public float y;
        public float z;
        public UInt32 time;

        public string Print()
        {
            return string.Format("{0}, [{1:0.0000}, {2:0.0000}, {3:0.0000}, {4:0.0000}]",
                time, w, x, y, z);
        }
    }


    public class ArduinoController
    {
        private SerialPort sp;
        private byte ImuTotal;
        private byte HashByte = 0xe5;

        public ArduinoController(byte imuTotal)
        {
            ImuTotal = imuTotal;
        }

        public bool IsOpen { get { return sp.IsOpen; } }

        public int Connect(string portName, int baudRate)
        {
            sp = new SerialPort(portName, baudRate) { ReadTimeout = 1 };
            sp.Open();

            if (IsOpen)
                return 1;
            else
                return -1;
        }

        public int CloseConnection()
        {
            if (IsOpen)
            {
                sp.Close();
                return 1;
            }
            else
            {
                return -1;
            }
        }

        public void Wait(int sec, string message = "Time")
        {
            for (int i = 0; i < sec; ++i)
            {
                Console.Write("\r{0}: {1} sec", message, i + 1);
                Thread.Sleep(1000);
            }
            Console.WriteLine();
        }

        public int SetParameterToArduino(Parameters param, byte value)
        {
            if (IsOpen)
            {
                byte commandTx = (byte)Commands.SetParameter; // set params
                byte check_sum_to = (byte)(commandTx ^ (byte)param ^ value ^ 0xe5);
                var paramsArduino = new byte[] { commandTx, (byte)param, value, check_sum_to };
                sp.Write(paramsArduino, 0, 4);

                while (true)
                {
                    int n = sp.BytesToRead;
                    if (n == 3)
                    {
                        byte commandRx = (byte)sp.ReadByte();
                        byte retCode = (byte)sp.ReadByte();
                        byte check_sum_from = (byte)sp.ReadByte();
                        byte check_sum_calc = (byte)(commandRx ^ retCode ^ 0xe5);
                        if (check_sum_from == check_sum_calc && retCode == (byte)RetCodes.OK)
                        {
                            return 1;
                        }
                    }
                }
            }
            return -1;
        }

        public int ReadData(Commands command, UInt16 packSize, out byte[] dataRx, out uint time, out ulong package)
        {
            dataRx = null;
            time = 0;
            package = 0;
            byte commandTx = (byte)command;
            byte check_sum_to = (byte)(commandTx ^ ImuTotal ^ HashByte);

            var dataTx = new byte[] { commandTx, ImuTotal, check_sum_to };
            sp.Write(dataTx, 0, dataTx.Length);

            while (true)
            {
                int n = sp.BytesToRead;
                if (n == packSize)
                {
                    ulong checkSumCalc = 0;
                    dataRx = new byte[n];
                    for (int i = 0; i < n; ++i)
                    {
                        dataRx[i] = (byte)sp.ReadByte();
                        if (i < n - 4)
                        {
                            checkSumCalc += dataRx[i];
                        }

                    }

                    // service data
                    time = BitConverter.ToUInt32(dataRx, n - 12);
                    package = BitConverter.ToUInt32(dataRx, n - 8);
                    ulong checkSumRx = BitConverter.ToUInt32(dataRx, n - 4);

                    if (checkSumCalc != checkSumRx)
                    {
                        return -1;
                    }

                    break;
                }
            }
            return 1;
        }

        public int ReadRawData(out Dictionary<int, RawData> data, out UInt32 _time, out ulong pachage)
        {
            UInt16 packSize = (UInt16)(ImuTotal * (9 * 2 + 2) + (3 * 4)); // 332;
            data = new Dictionary<int, RawData>();
            if (ReadData(Commands.GetRawData, packSize, out byte[] dataRx, out _time, out pachage) > 0)
            {
                for (int imu_id = 0; imu_id < ImuTotal; ++imu_id)
                {
                    int offset = imu_id * (10 * 2);
                    var rawData = new RawData()
                    {
                        ax = BitConverter.ToInt16(dataRx, offset),
                        ay = BitConverter.ToInt16(dataRx, offset + 2),
                        az = BitConverter.ToInt16(dataRx, offset + 4),
                        gx = BitConverter.ToInt16(dataRx, offset + 6),
                        gy = BitConverter.ToInt16(dataRx, offset + 8),
                        gz = BitConverter.ToInt16(dataRx, offset + 10),
                        mx = BitConverter.ToInt16(dataRx, offset + 12),
                        my = BitConverter.ToInt16(dataRx, offset + 14),
                        mz = BitConverter.ToInt16(dataRx, offset + 16),
                        time = _time - (UInt32)BitConverter.ToUInt16(dataRx, offset + 18)
                    };
                    data.Add(imu_id, rawData);
                }
            }
            else
            {
                return -1;
            }
            return 1;
        }

        public int ReadQuaternionData(out Dictionary<int, SensorQuaternion> data, out UInt32 _time, out ulong pachage)
        {
            UInt16 packSize = (UInt16)(ImuTotal * (4 * 4 + 2) + (3 * 4)); // 300;
            data = new Dictionary<int, SensorQuaternion>();
            if (ReadData(Commands.GetQuaternions, packSize, out byte[] dataRx, out _time, out pachage) > 0)
            {
                for (int imu_id = 0; imu_id < ImuTotal; ++imu_id)
                {
                    int offset = imu_id * (4 * 4 + 2);
                    var quat = new SensorQuaternion()
                    {
                        w = BitConverter.ToSingle(dataRx, offset),
                        x = BitConverter.ToSingle(dataRx, offset + 4),
                        y = BitConverter.ToSingle(dataRx, offset + 8),
                        z = BitConverter.ToSingle(dataRx, offset + 12),
                        time = _time - (UInt32)BitConverter.ToUInt16(dataRx, offset + 16)
                    };
                    data.Add(imu_id, quat);
                };
            }
            else
            {
                return -1;
            }
            return 1;
        }

    }

    public class MotionProcessor
    {
        private float _G = 9.807f;
        private float _d2r = 3.14159265359f / 180.0f;
        private AccelRange _accelRange;
        private GyroRange _gyroRange;
        private float _accelScale;
        private float _gyroScale;
        private float _magScaleX;
        private float _magScaleY;
        private float _magScaleZ;

        // Accelerometr coordinate to Unity coordinate transform
        private Int16[] aX = new Int16[3] { 0, 1, 0 };
        private Int16[] aY = new Int16[3] { 0, 0, 1 };
        private Int16[] aZ = new Int16[3] { -1, 0, 0 };

        // Gyroscope coordinate to Unity coordinate transform
        private Int16[] gX = new Int16[3] { 0, -1, 0 };
        private Int16[] gY = new Int16[3] { 0, 0, -1 };
        private Int16[] gZ = new Int16[3] { 1, 0, 0 };

        // Compass coordinate to Unity coortinate transform
        private Int16[] mX = new Int16[3] { 0, 1, 0 };
        private Int16[] mY = new Int16[3] { 0, 0, -1 };
        private Int16[] mZ = new Int16[3] { 1, 0, 0 };

        // accelerometr
        private float accelBiasX = 0.0f;
        private float accelBiasY = 0.0f;
        private float accelBiasZ = 0.0f;
        private float _axs = 1.0f;
        private float _ays = 1.0f;
        private float _azs = 1.0f;
        private float _axmax, _aymax, _azmax;
        private float _axmin, _aymin, _azmin;
        private double _axbD = 0;
        private double _aybD = 0;
        private double _azbD = 0;

        // gyroscope
        private float gyroBiasX = 0.0f;
        private float gyroBiasY = 0.0f;
        private float gyroBiasZ = 0.0f;
        private double _gxbD = 0;
        private double _gybD = 0;
        private double _gzbD = 0;

        // magnetometr
        private float magnetBiasX = 0.0f;
        private float magnetBiasY = 0.0f;
        private float magnetBiasZ = 0.0f;
        private float _mxs = 1.0f;
        private float _mys = 1.0f;
        private float _mzs = 1.0f;
        private float _mxmax, _mymax, _mzmax;
        private float _mxmin, _mymin, _mzmin;
        private double _mxbD = 0;
        private double _mybD = 0;
        private double _mzbD = 0;

        // calibration
        public uint numSamples = 5000;
        public uint accelCounter = 0;
        public uint gyroCounter = 0;
        public uint magnetCounter = 0;

        private float GetAccelRange()
        {
            float range = 0.0f;
            if (_accelRange == AccelRange.ACCEL_RANGE_2G)
            {
                range = 2.0f;
            }
            else if (_accelRange == AccelRange.ACCEL_RANGE_4G)
            {
                range = 4.0f;
            }
            else if (_accelRange == AccelRange.ACCEL_RANGE_8G)
            {
                range = 8.0f;
            }
            else if (_accelRange == AccelRange.ACCEL_RANGE_16G)
            {
                range = 16.0f;
            }
            return range;
        }

        private float GetGyroScale()
        {
            float range = 0.0f;
            if (_gyroRange == GyroRange.GYRO_RANGE_250DPS)
            {
                range = 131.0f / 250.0f;
            }
            else if (_gyroRange == GyroRange.GYRO_RANGE_500DPS)
            {
                range = 65.5f / 500.0f;
            }
            else if (_gyroRange == GyroRange.GYRO_RANGE_1000DPS)
            {
                range = 32.8f / 1000.0f;
            }
            else if (_gyroRange == GyroRange.GYRO_RANGE_2000DPS)
            {
                range = 16.4f / 2000.0f;
            }
            return range;
        }

        public MotionProcessor(AccelRange accelRange, GyroRange gyroRange)
        {
            _accelRange = accelRange;
            _accelScale = _G * GetAccelRange() / 32767.5f; // setting the accel scale to 16G

            _gyroRange = gyroRange;
            _gyroScale = GetGyroScale(); // setting the gyro scale to 2000DPS
            //_gyroScale = GetGyroRange() * _d2r;
            
            byte[] _buffer = new byte[3];
            _buffer[0] = 176;
            _buffer[1] = 178;
            _buffer[2] = 168;
            _magScaleX = ((((float)_buffer[0]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
            _magScaleY = ((((float)_buffer[1]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
            _magScaleZ = ((((float)_buffer[2]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
        }

        public void ProcessRawData(RawData rawData, out CalibratedData calibData)
        {
            calibData = new CalibratedData();
            calibData.ax = (((float)(aX[0] * rawData.ax + aX[1] * rawData.ay + aX[2] * rawData.az) * _accelScale) - accelBiasX) * _axs;
            calibData.ay = (((float)(aY[0] * rawData.ax + aY[1] * rawData.ay + aY[2] * rawData.az) * _accelScale) - accelBiasY) * _ays;
            calibData.az = (((float)(aZ[0] * rawData.ax + aZ[1] * rawData.ay + aZ[2] * rawData.az) * _accelScale) - accelBiasZ) * _azs;
            calibData.gx = ((float)(gX[0] * rawData.gx + gX[1] * rawData.gy + gX[2] * rawData.gz) * _gyroScale) - gyroBiasX;
            calibData.gy = ((float)(gY[0] * rawData.gx + gY[1] * rawData.gy + gY[2] * rawData.gz) * _gyroScale) - gyroBiasY;
            calibData.gz = ((float)(gZ[0] * rawData.gx + gZ[1] * rawData.gy + gZ[2] * rawData.gz) * _gyroScale) - gyroBiasZ;
            calibData.mx = (((float)(mX[0] * rawData.mx + mX[1] * rawData.my + mX[2] * rawData.mz) * _magScaleX) - magnetBiasX) * _mxs;
            calibData.my = (((float)(mY[0] * rawData.mx + mY[1] * rawData.my + mY[2] * rawData.mz) * _magScaleY) - magnetBiasY) * _mys;
            calibData.mz = (((float)(mZ[0] * rawData.mx + mZ[1] * rawData.my + mZ[2] * rawData.mz) * _magScaleZ) - magnetBiasZ) * _mzs;
            calibData.time = rawData.time;
        }

        public SensorQuaternion UpdateQuaternion(SensorQuaternion oldQuat, CalibratedData calibData)
        {
            float gx = calibData.gx * _d2r;
            float gy = calibData.gy * _d2r;
            float gz = calibData.gz * _d2r;

            float qx = oldQuat.x;
            float qy = oldQuat.y;
            float qz = oldQuat.z;
            float qw = oldQuat.w;

            float qDotW = 0.5f * (-qx * gx - qy * gy - qz * gz);
            float qDotX = 0.5f * (qw * gx + qy * gz - qz * gy);
            float qDotY = 0.5f * (qw * gy - qx * gz + qz * gx);
            float qDotZ = 0.5f * (qw * gz + qx * gy - qy * gx);

            uint dt = calibData.time - oldQuat.time;
            float deltat = dt / 1000.0f;
            // Integrate to yield quaternion
            qx += qDotX * deltat;
            qy += qDotY * deltat;
            qz += qDotZ * deltat;
            qw += qDotW * deltat;
            double norm = Math.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw); // normalise quaternion
            norm = 1.0f / norm;
            qx *= (float)norm;
            qy *= (float)norm;
            qz *= (float)norm;
            qw *= (float)norm;

            var newQuat = new SensorQuaternion() { x = qx, y = qy, z = qz, w = qw, time = calibData.time };
            return newQuat;
        }

        private bool CalibrateAccelerometr(float x, float y, float z)
        {
            ++accelCounter;
            if (accelCounter < numSamples)
            {
                _axbD += (x / _axs + accelBiasX) / ((double)numSamples);
                _aybD += (y / _ays + accelBiasY) / ((double)numSamples);
                _azbD += (z / _azs + accelBiasZ) / ((double)numSamples);
            }
            else if (accelCounter == numSamples)
            {
                // X
                if (_axbD > _axmax) _axmax = (float)_axbD;
                if (_axbD < _axmin) _axmin = (float)_axbD;

                accelBiasX = (_axmin + _axmax) / 2.0f;
                _axs = _G / ((Math.Abs(_axmin) + Math.Abs(_axmax)) / 2.0f);

                // Y
                if (_aybD > _aymax) _aymax = (float)_aybD;
                if (_aybD < _aymin) _aymin = (float)_aybD;

                accelBiasY = (_aymin + _aymax) / 2.0f;
                _ays = _G / ((Math.Abs(_aymin) + Math.Abs(_aymax)) / 2.0f);

                // Z
                if (_azbD > _azmax) _azmax = (float)_azbD;
                if (_azbD < _azmin) _azmin = (float)_azbD;

                accelBiasZ = (_azmin + _azmax) / 2.0f;
                _azs = _G / ((Math.Abs(_azmin) + Math.Abs(_azmax)) / 2.0f);                

                /*
                if (_axbD > 9.0f)
                {
                    _axmax = (float)_axbD;
                }
                if (_aybD > 9.0f)
                {
                    _aymax = (float)_aybD;
                }
                if (_azbD > 9.0f)
                {
                    _azmax = (float)_azbD;
                }
                if (_axbD < -9.0f)
                {
                    _axmin = (float)_axbD;
                }
                if (_aybD < -9.0f)
                {
                    _aymin = (float)_aybD;
                }
                if (_azbD < -9.0f)
                {
                    _azmin = (float)_azbD;
                }

                // find bias and scale factor
                if ((Math.Abs(_axmin) > 9.0f) && (Math.Abs(_axmax) > 9.0f))
                {
                    accelBiasX[imu_id] = (_axmin + _axmax) / 2.0f;
                    _axs = _G / ((Math.Abs(_axmin) + Math.Abs(_axmax)) / 2.0f);
                }
                if ((Math.Abs(_aymin) > 9.0f) && (Math.Abs(_aymax) > 9.0f))
                {
                    accelBiasY[imu_id] = (_aymin + _aymax) / 2.0f;
                    _ays = _G / ((Math.Abs(_aymin) + Math.Abs(_aymax)) / 2.0f);
                }
                if ((Math.Abs(_azmin) > 9.0f) && (Math.Abs(_azmax) > 9.0f))
                {
                    accelBiasZ[imu_id] = (_azmin + _azmax) / 2.0f;
                    _azs = _G / ((Math.Abs(_azmin) + Math.Abs(_azmax)) / 2.0f);
                }
                */

                _axbD = 0;
                _aybD = 0;
                _azbD = 0;

                return true;
            }
            else
            {
                return true;
            }
            return false;
        }

        private bool CalibrateGyroscope(float x, float y, float z)
        {
            ++gyroCounter;
            if (gyroCounter <= numSamples)
            {
                //_gxbD += (x + gyroBiasX) / ((double)numSamples);
                //_gybD += (y + gyroBiasY) / ((double)numSamples);
                //_gzbD += (z + gyroBiasZ) / ((double)numSamples);
                _gxbD += x + gyroBiasX;
                _gybD += y + gyroBiasY;
                _gzbD += z + gyroBiasZ;
            }
            if (gyroCounter == numSamples)
            {
                gyroBiasX = (float)(_gxbD / gyroCounter);
                gyroBiasY = (float)(_gybD / gyroCounter);
                gyroBiasZ = (float)(_gzbD / gyroCounter);
                _gxbD = 0;
                _gybD = 0;
                _gzbD = 0;
                return true;
            }
            else
            {
                return false;
            }
        }

        private bool CalibrateMagnitometr(float x, float y, float z)
        {
            ++magnetCounter;

            if (magnetCounter < numSamples)
            {
                _mxbD += (x / _mxs + magnetBiasX) / ((double)numSamples);
                _mybD += (y / _mys + magnetBiasY) / ((double)numSamples);
                _mzbD += (z / _mzs + magnetBiasZ) / ((double)numSamples);
            }
            else if (magnetCounter == numSamples)
            {
                // X
                if (_mxbD > _mxmax) _mxmax = (float)_mxbD;
                if (_mxbD < _mxmin) _mxmin = (float)_mxbD;

                magnetBiasX = (_mxmin + _mxmax) / 2.0f;

                // Y
                if (_mybD > _mymax) _mymax = (float)_mybD;
                if (_mybD < _mymin) _mymin = (float)_mybD;

                magnetBiasY = (_mymin + _mymax) / 2.0f;

                // Z
                if (_mzbD > _mzmax) _mzmax = (float)_mzbD;
                if (_mzbD < _mzmin) _mzmin = (float)_mzbD;

                magnetBiasZ = (_mzmin + _mzmax) / 2.0f;
                                
                _mxbD = 0;
                _mybD = 0;
                _mzbD = 0;

                return true;
            }
            else
            {
                return true;
            }
            return false;
        }

        private void PrintCounters()
        {
            Console.Write("\rAccelerometr: {0}; Gyroscope: {1}; Magnetometr: {2}", accelCounter, gyroCounter, magnetCounter);
        }

        public void PrintAccelCalibrationData()
        {
            Console.WriteLine("Accelerometr calibration data:");

            Console.WriteLine("axbias: {0:00.0000}, aybias: {1:00.0000}, azbias: {2:00.0000}", accelBiasX, accelBiasY, accelBiasZ);
            Console.WriteLine(" axmax: {0:00.0000},  aymax: {1:00.0000},  azmax: {2:00.0000}", _axmax, _aymax, _azmax);
            Console.WriteLine(" axmin: {0:00.0000},  aymin: {1:00.0000},  azmin: {2:00.0000}", _axmin, _aymin, _azmin);
            Console.WriteLine("   axs: {0:00.0000},    ays: {1:00.0000},    azs: {2:00.0000}", _axs, _ays, _azs);
        }

        public string PrintGyroCalibrationData(bool print)
        {
            string msg = "";
            Console.WriteLine("Gyroscope calibration data:");

            msg += string.Format("gyroBiasX: {0}, gyroBiasY: {1}, gyroBiasZ: {2}\n", gyroBiasX, gyroBiasY, gyroBiasZ);
            if (print)
            {
                Console.WriteLine(msg);
            }
            
            return msg;
        }

        public void PrintMagnetCalibrationData()
        {
            Console.WriteLine("Magnetometr calibration data:");
            Console.WriteLine("mxbias: {0:00.0000}, mybias: {1:00.0000}, mzbias: {2:00.0000}", magnetBiasX, magnetBiasY, magnetBiasZ);
            Console.WriteLine(" mxmax: {0:00.0000},  mymax: {1:00.0000},  mzmax: {2:00.0000}", _mxmax, _mymax, _mzmax);
            Console.WriteLine(" mxmin: {0:00.0000},  mymin: {1:00.0000},  mzmin: {2:00.0000}", _mxmin, _mymin, _mzmin);
            Console.WriteLine("   mxs: {0:00.0000},    mys: {1:00.0000},    mzs: {2:00.0000}", _mxs, _mys, _mzs);
        }

        public bool Calibrate(byte imu_id, RawData rawData, bool accel, bool gyro, bool magnet)
        {
            ProcessRawData(rawData, out CalibratedData calibData);

            bool accelCalibrated = false;
            bool gyroCalibrated = false;
            bool magnetCalibrated = true;
            if (accel)
            {
                accelCalibrated = CalibrateAccelerometr(calibData.ax, calibData.ay, calibData.az);
            }
            if (gyro)
            {
                gyroCalibrated = CalibrateGyroscope(calibData.gx, calibData.gy, calibData.gz);
            }
            if (magnet)
            {
                magnetCalibrated = CalibrateMagnitometr(calibData.mx, calibData.my, calibData.mz);
            }

            PrintCounters();

            if ((accelCalibrated == accel) && (gyroCalibrated == gyro) && (magnetCalibrated == magnet))
            {
                return true;
            }

            return false;
        }
    }
}
