using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SVGtoRoboArmCode
{
    using System.Globalization;
    using System.IO.Ports;
    using System.Threading;

    public class StepToAngelConverter
    {
        public const long AGearsTeath = 32 * 22 * 26 * 31;

        public const long BGearsTeath = 9 * 11 * 9 * 10;

        public const long StepsPerRotation = 64;

        public double ToAngel(long steps)
        {
            return (double)(steps) * 2 * BGearsTeath / (StepsPerRotation * AGearsTeath);
        }

        public double ToAngelGrad(long steps)
        {
            return ToAngel(steps) * 180;
        }

        public long ToSteps(double angel)
        {
            return (long)(angel * StepsPerRotation * AGearsTeath / (2 * BGearsTeath));
        }
    }

    public class ArmCoordinateSystemConverter
    {
        private readonly StepToAngelConverter angelConverter = new StepToAngelConverter();

        public long ATotalStep { get; private set; }
        public long BTotalStep { get; private set; }
        public long ATarget { get; private set; }
        public long BTarget { get; private set; }
        public double ALength { get; private set; }
        public double BLength { get; private set; }

        public double AAngel
        {
            get
            {
                return angelConverter.ToAngel(ATotalStep);
            }
        }

        public double BAngel
        {
            get
            {
                return angelConverter.ToAngel(BTotalStep);
            }
        }

        public double AXPosition
        {
            get
            {
                return ALength * Math.Cos(AAngel * Math.PI);
            }
        }

        public double AYPosition
        {
            get
            {
                return ALength * Math.Sin(AAngel * Math.PI);
            }
        }

        public double BXPosition
        {
            get
            {
                return AXPosition - BLength * Math.Cos((BAngel + AAngel) * Math.PI);
            }
        }

        public double BYPosition
        {
            get
            {
                return AYPosition - BLength * Math.Sin((BAngel + AAngel) * Math.PI);
            }
        }

        public ArmCoordinateSystemConverter(double aLength, double bLength)
        {
            ALength = aLength;
            BLength = bLength;
        }

        public void MoveToPosition(double x, double y)
        {
            double lQuad = x * x + y * y;
            double l = Math.Sqrt(lQuad);

            double bAngel = Math.Acos((ALength * ALength + BLength * BLength - lQuad) / (2 * ALength * BLength)) / Math.PI;
            double a1 = Math.Acos(-(BLength * Math.Cos(bAngel * Math.PI) - ALength) / l) / Math.PI;
            double a2 = Math.Acos(x / l) / Math.PI;
            double aAngel = 1 - (a1 + a2);

            long aValue = angelConverter.ToSteps(aAngel);
            long bValue = angelConverter.ToSteps(bAngel);

            ATarget = aValue - ATotalStep;
            ATotalStep = aValue;

            BTarget = bValue - BTotalStep;
            BTotalStep = bValue;
        }

        public void Reset()
        {
            ATarget = -ATotalStep;
            ATotalStep = 0;
            BTarget = -BTotalStep;
            BTotalStep = 0;
        }
    }

    public class Point
    {
        public double X { get; private set; }
        public double Y { get; private set; }

        public Point(double x, double y)
        {
            X = x;
            Y = y;
        }
    }

    public class ExtendedPoint : Point
    {
        public bool PenDown { get; private set; }

        public ExtendedPoint(double x, double y, bool down) : base(x, y)
        {
            PenDown = down;
        }
    }

    public class PathRunner
    {
        private char[] commands = new[] { 'M', 'm', 'L', 'l', 'Z', 'z', 'H', 'h', 'V', 'v', 'C', 'c', 'S', 's', 'Q', 'q', 'T', 't', 'A', 'a' };

        private const double StepSize = 1;

        public IEnumerable<string> ParseData(string data)
        {
            if (string.IsNullOrWhiteSpace(data))
                throw new ArgumentException($"{nameof(data)} parameter should not be empty.");

            var start = 0;
            for (int i = 0; i < data.Length; i++)
            {
                if (commands.Contains(data[i]))
                {
                    if (start < i) yield return data.Substring(start, i - start);

                    yield return data[i].ToString();
                    start = i + 1;
                    continue;
                }

                if (data[i] == ' ')
                {
                    if (start < i) yield return data.Substring(start, i - start);
                    start = i + 1;
                }

                if (data[i] == '-')
                {
                    if (start < i) yield return data.Substring(start, i - start);
                    start = i;
                }
            }
            if (start < data.Length) yield return data.Substring(start, data.Length - start);
        }

        public IEnumerable<ExtendedPoint> Run(string data, ExtendedPoint current = null)
        {
            if (current == null)
            {
                current = new ExtendedPoint(0, 0, false);
            }

            var cmds = ParseData(data).ToArray();

            for (var i = 0; i < cmds.Length; i++)
            {
                if (cmds[i] == "M")
                {
                    current = new ExtendedPoint(double.Parse(cmds[i + 1], CultureInfo.InvariantCulture),
                        double.Parse(cmds[i + 2], CultureInfo.InvariantCulture), false);
                    yield return current;
                    continue;
                }

                if (cmds[i] == "m")
                {
                    current = new ExtendedPoint(current.X + double.Parse(cmds[i + 1], CultureInfo.InvariantCulture),
                        current.Y + double.Parse(cmds[i + 2], CultureInfo.InvariantCulture), false);
                    yield return current;
                    continue;
                }

                if (cmds[i] == "L")
                {
                    var start = current;
                    var dest = new ExtendedPoint(
                        double.Parse(cmds[i + 1], CultureInfo.InvariantCulture),
                        double.Parse(cmds[i + 2], CultureInfo.InvariantCulture), true);

                    var xSize = dest.X - start.X;
                    var ySize = dest.Y - start.Y;

                    var l = (int)Math.Ceiling(Math.Sqrt(Math.Pow(xSize, 2) + Math.Pow(ySize, 2)) / StepSize);

                    xSize = xSize / l;
                    ySize = ySize / l;

                    for (int j = 1; j <= l; j++)
                    {
                        yield return new ExtendedPoint(start.X + xSize * j, start.Y + ySize * j, true);
                    }

                    current = dest;
                }
            }
        }
    }

    class Program
    {
        public const double ALength = 10;
        public const double BLength = 10;


        static void Main(string[] args)
        {
            ComTest();
        }

        static SerialPort port;

        private static void ComTest()
        {
            port = new SerialPort();
            var runner = new PathRunner();
            var converter = new ArmCoordinateSystemConverter(10, 10);
            var acon = new StepToAngelConverter();
            try
            {
                while (true)
                {
                    try
                    {
                        Console.WriteLine("Enter coordinates. Or 'exit' to quit.");
                        var cmd = Console.ReadLine();
                        if (string.CompareOrdinal(cmd, "exit") == 0)
                        {
                            if (port.IsOpen)
                            {
                                port.Close();
                            }
                            return;
                        }

                        if (string.CompareOrdinal(cmd, "ports") == 0)
                        {
                            var portNames = SerialPort.GetPortNames();
                            Console.WriteLine("Available serial ports:");
                            foreach (var portName in portNames)
                            {
                                Console.WriteLine(portName);
                            }
                            continue;
                        }

                        if (cmd.StartsWith("con "))
                        {
                            var parts = cmd.Split(new[] { " " }, StringSplitOptions.RemoveEmptyEntries);
                            if (parts.Length < 3)
                            {
                                Console.WriteLine($"Can't connect using '{cmd}'.");
                                continue;
                            }

                            if (port.IsOpen)
                            {
                                port.Close();
                            }
                            var baud = int.Parse(parts[2]);
                            Console.WriteLine($"Connecting {parts[1]} at {baud} baud.");

                            port.PortName = parts[1];
                            port.BaudRate = baud;
                            port.DataBits = 8;
                            port.Parity = Parity.None;
                            port.StopBits = StopBits.One;
                            port.DataReceived += OutData;
                            port.RtsEnable = true;
                            port.DtrEnable = true;
                            port.Open();
                            continue;
                        }

                        if (cmd.StartsWith("out "))
                        {
                            var txt = cmd.Substring(4);
                            Serial(txt);
                            continue;
                        }
                        
                        Console.WriteLine($"Running path: '{cmd}'.");
                        var pathData = runner.Run(cmd);
                        var penState = 0;
                        foreach(var point in pathData)
                        {
                            Console.WriteLine($"{(point.PenDown ? "LineTo" : "MoveTo")} ({point.X}; {point.Y}).");
                            converter.MoveToPosition(point.X, point.Y);
                            if (point.PenDown && penState!= 1)
                            {
                                Serial("c30d1s1agw-1agw100s");

                                Thread.Sleep(700);
                                Serial("u");
                                penState = 1;
                            }
                            else if(!point.PenDown && penState != 2)
                            {
                                Serial("c170d1s1agw-1agw100s");
                                Thread.Sleep(700);
                                Serial("u");
                                penState = 2;
                            }
                            Console.WriteLine($"Move to A angel {acon.ToAngelGrad(converter.ATotalStep)} B angel {acon.ToAngelGrad(converter.BTotalStep)}.");

                            Serial($"{converter.ATarget}b{converter.BTarget}agw");
                        }                        
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine(ex);
                        Console.WriteLine();
                    }
                }
            }
            finally
            {
                if (port != null)
                {
                    if (port.IsOpen)
                    {
                        port.Close();
                    }
                    port.Dispose();
                }
            }
        }

        private static void Serial(string v)
        {
            Console.WriteLine($"PORT: {v}");
            if (port.IsOpen)
              port.Write(v);
        }

        private static void OutData(object sender, SerialDataReceivedEventArgs e)
        {
            if (port != null)
            {
                var buffer = new byte[port.BytesToRead];
                var num = port.Read(buffer, 0, buffer.Length);
                var str = Encoding.ASCII.GetString(buffer);
                Console.Write(str);
            }
        }

        private static void PathRunnerTests()
        {
            var runner = new PathRunner();

            while (true)
            {
                Console.WriteLine("Enter coordinates. Or 'exit' to quit.");
                var cmd = Console.ReadLine();
                if (string.CompareOrdinal(cmd, "exit") == 0)
                {
                    return;
                }

                var result = runner.ParseData(cmd);
                foreach (var s in result)
                {
                    Console.WriteLine(s);
                }

                var points = runner.Run(cmd);
                foreach (var point in points)
                {
                    Console.WriteLine($"({point.X},{point.Y})");
                }
            }
        }

        static void ArmCoordinateSystemConverterTests(string[] args)
        {
            var converter = new ArmCoordinateSystemConverter(ALength, BLength);
            while (true)
            {
                Console.WriteLine("Enter coordinates. Or 'exit' to quit.");
                var cmd = Console.ReadLine();
                if (string.CompareOrdinal(cmd, "exit") == 0)
                {
                    return;
                }

                if (string.CompareOrdinal(cmd, "reset") == 0)
                {
                    Console.WriteLine("Reset.");
                    converter.Reset();
                }
                else
                {
                    var parts = cmd.Split(new[] { ";" }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length != 2)
                    {
                        Console.WriteLine("Enter data in format: x; y");
                        continue;
                    }

                    double x, y;
                    if (!double.TryParse(parts[0].Trim(), out x))
                    {
                        Console.WriteLine($"X : '{parts[0]}' is not a number.");
                        continue;
                    }

                    if (!double.TryParse(parts[1].Trim(), out y))
                    {
                        Console.WriteLine($"Y : '{parts[1]}' is not a number.");
                        continue;
                    }

                    converter.MoveToPosition(x, y);
                }

                Console.WriteLine($"Moved to Steps({converter.ATarget};{converter.BTarget}) Total ({converter.ATotalStep};{converter.BTotalStep}).");
                Console.WriteLine($"Moved to Angels({converter.AAngel};{converter.BAngel})");
                Console.WriteLine($"Moved to AJoint ({converter.AXPosition};{converter.AYPosition}) Pen ({converter.BXPosition};{converter.BYPosition}).");
            }
        }
    }
}