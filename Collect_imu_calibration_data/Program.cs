using System;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Collections.Generic;

namespace CommunicationProtocol
{
    class Program
    {
        static byte imu_total = 3;
        static int init_time = 20;
        static int baud_rate = 500000;
        static string com_port = "COM3";

        static int data_size = 500;
        static string data_folder = @"D:\ROBOTS\__ПРОЕКТЫ_В_РАБОТЕ\data_gloves_v1\Raw_imu_data";
        static int timeout = 10;

        
        static void Collect_IMU_raw_data(ArduinoController controller, int side, string title)
        {
            var csv_files = new Dictionary<int, StringBuilder>();
            for (byte i = 0; i < imu_total; ++i)
            {
                StringBuilder csv_file = new StringBuilder();
                csv_file.AppendLine("imu,time,ax,ay,az,gx,gy,gz,mx,my,mz");
                csv_files.Add(i, csv_file);
            }

            int data_counter = 0;
            while (true)
            {
                if (controller.ReadRawData(out Dictionary<int, RawData> data, out UInt32 time, out ulong package) > 0)
                {
                    data_counter++;
                    Console.Write("\rReceived data: {0}", data_counter);

                    foreach (var csv_pair in csv_files)
                    {
                        var imu_id = csv_pair.Key;
                        var csv_file = csv_pair.Value;
                        AddRawDataInFile(imu_id, data[imu_id], csv_file);
                    }

                    if (data_counter >= data_size)
                    {
                        Console.WriteLine();
                        foreach (var csv_pair in csv_files)
                        {
                            var imu_id = csv_pair.Key;
                            var csv_file = csv_pair.Value;
                            string path = string.Format("{0}\\{1}-{2}-{3}.csv", data_folder, title, imu_id, side);
                            System.IO.File.WriteAllText(path, csv_file.ToString());
                        }
                        break;
                    }
                }
            }
        }

        static void Collect_Magnetometr_Calibration_Data(ArduinoController controller)
        {
            Collect_IMU_raw_data(controller, 0, "magnet");
        }

        static void Collect_Gyro_Accel_Calibration_Data(ArduinoController controller)
        {
            byte sides = 6;
            for (byte side_itr = 1; side_itr <= sides; ++side_itr)
            {
                Collect_IMU_raw_data(controller, side_itr, "gyro_accel");

                if (side_itr != sides)
                {
                    controller.Wait(timeout, string.Format("Переверните датчик на строну {0}/{1}", side_itr + 1, sides));
                    Console.Beep();
                }
            }
        }

        static void Collect_Gyro_Accel_Calibration_Data_v0(ArduinoController controller)
        {
            byte sides = 6;
            for (byte side_itr = 1; side_itr <= sides; ++side_itr)
            {
                var csv_files = new Dictionary<int, StringBuilder>();
                for (byte i = 0; i < imu_total; ++i)
                {
                    StringBuilder csv_file = new StringBuilder();
                    csv_file.AppendLine("imu,time,ax,ay,az,gx,gy,gz,mx,my,mz");
                    csv_files.Add(i, csv_file);
                }

                int data_counter = 0;
                while (true)
                {
                    if (controller.ReadRawData(out Dictionary<int, RawData> data, out UInt32 time, out ulong package) > 0)
                    {
                        data_counter++;
                        Console.Write("\rReceived data: {0}", data_counter);

                        foreach (var csv_pair in csv_files)
                        {
                            var imu_id = csv_pair.Key;
                            var csv_file = csv_pair.Value;
                            AddRawDataInFile(imu_id, data[imu_id], csv_file);
                        }

                        if (data_counter >= data_size)
                        {
                            Console.WriteLine();
                            foreach (var csv_pair in csv_files)
                            {
                                var imu_id = csv_pair.Key;
                                var csv_file = csv_pair.Value;
                                string path = string.Format("{0}\\gyro_accel-{1}-{2}.csv", data_folder, imu_id, side_itr);
                                System.IO.File.WriteAllText(path, csv_file.ToString());
                            }
                            break;
                        }
                    }
                }

                if (side_itr != sides)
                {
                    controller.Wait(timeout, string.Format("Переверните датчик на строну {0}/{1}", side_itr + 1, sides));
                    Console.Beep();
                }
            }
        }
        
        static void AddRawDataInFile(int imu, RawData row, StringBuilder file)
        {
            var newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}",
                                imu, row.time,
                                row.ax, row.ay, row.az,
                                row.gx, row.gy, row.gz,
                                row.mx, row.my, row.mz);
            file.AppendLine(newLine);
        }

        static void Collect_calibration_data()
        {

            var controller = new ArduinoController(imu_total);
            if (controller.Connect(com_port, baud_rate) < 0)
            {
                Console.WriteLine("No connection");
            }

            Console.WriteLine("Controller initialization, please wait {0} second...", init_time);
            controller.Wait(init_time);

            if (controller.SetParameterToArduino(Parameters.Mode, (byte)Modes.RawData) <= 0)
            {
                Console.WriteLine("Can't set arduino parameters. Calibration process is stoped.");
                return;
            }

            var motionProc = new MotionProcessor(AccelRange.ACCEL_RANGE_16G, GyroRange.GYRO_RANGE_2000DPS);

            Console.WriteLine("\nCollection gyroscope and accelerometr data:");
            Console.WriteLine("Attention: IMU sensor must be hard fixed on each of 6 side!!!");
            Collect_Gyro_Accel_Calibration_Data(controller);

            Console.WriteLine("\nCollection magnitometr data:");
            Console.WriteLine("Attention: IMU sensor must be randomly rotated!!!");
            Collect_Magnetometr_Calibration_Data(controller);
        }


        static void Main(string[] args)
        {
            Collect_calibration_data();
        }
    }
}
