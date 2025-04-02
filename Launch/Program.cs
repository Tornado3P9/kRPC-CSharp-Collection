// dotnet add package KRPC.Client --version 0.5.4
// dotnet add package Google.Protobuf --version 3.30.1
// dotnet add package System.CommandLine --version 2.0.0-beta4.22272.1

using System;
using System.Collections.Generic;
using System.Net;
using System.Diagnostics;
using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Client.Services.UI;

class Program
{
    public static void Main(string[] args)
    {
        (int target, int compass, bool auto_throttle, bool ag5) = ParseArguments(args);
        Console.WriteLine($"Target: {target}, Compass: {compass}, auto_throttle: {auto_throttle}, ag5: {ag5}");

        // Connect to KRPC
        var conn = new Connection("Launch into orbit");
        var vessel = conn.SpaceCenter().ActiveVessel;

        float targetAltitude = target;
        float current_Altitude = 0;
        var uiElements = SetupUI(conn, auto_throttle);
        var panel = uiElements.Item1;
        var button = uiElements.Item2;
        var buttonClicked = uiElements.Item3;

        // Set up streams for telemetry
        var ut = conn.AddStream(() => conn.SpaceCenter().UT);
        var flight = vessel.Flight();
        var altitude = conn.AddStream(() => flight.MeanAltitude);
        var apoapsis = conn.AddStream(() => vessel.Orbit.ApoapsisAltitude);

        // Controller setup
        Controller i = new();
        i.SetOutputLimits(0.01, 1);
        double currentTWR = 0;  // placeholder
        double targetTWR = 1.6;
        double error = 0;       // placeholder
        float throttle = 1;
        
        // Pre-launch setup
        vessel.Control.SAS = false;
        vessel.Control.RCS = false;
        vessel.Control.Throttle = throttle;

        // Countdown...
        Console.Clear();
        Console.WriteLine("...3");
        Thread.Sleep(1000);
        Console.WriteLine("...2");
        Thread.Sleep(1000);
        Console.WriteLine("...1");
        Thread.Sleep(1000);
        Console.WriteLine("Launch!");

        // Activate the first stage
        vessel.Control.ActivateNextStage();
        vessel.AutoPilot.Engage();
        vessel.AutoPilot.TargetPitchAndHeading(90, vessel.Flight().Heading);  // (pitch, yaw)
        vessel.AutoPilot.TargetRoll = vessel.Flight().Roll;

        // Initialize Counter
        int counter = 0;

        // Roll Program
        while (true)
        {
            double verticalSpeed = vessel.Flight(vessel.Orbit.Body.ReferenceFrame).VerticalSpeed;
            if (verticalSpeed >= 60 || counter > 15)
            {
                break;
            }
            counter++;
            Thread.Sleep(1000);
        }
        Console.WriteLine("Roll");
        vessel.AutoPilot.TargetRoll = 0;
        vessel.AutoPilot.TargetPitchAndHeading(90, compass);

        // Unhide UI elements
        panel.Visible = true;
        button.Visible = true;
        
        // Main ascent loop
        Console.WriteLine("Gravity turn");
        Stopwatch stopwatch = new();  // deltaTime calculation
        stopwatch.Start();
        float pitch = 0;
        counter = 0;
        while (true)
        {
            // Gravity turn
            current_Altitude = (float)altitude.Get();
            pitch = (1.48272E-8f * current_Altitude * current_Altitude) - 0.00229755f * current_Altitude + 90f;
            pitch = Math.Max(pitch, 2);
            vessel.AutoPilot.TargetPitchAndHeading(pitch, compass);

            // Stop main loop and disable engines when target apoapsis is reached
            if (apoapsis.Get() > targetAltitude)
            {
                vessel.Control.Throttle = 0;
                Console.WriteLine("Target apoapsis reached");
                break;
            }

            // Handle auto staging once thrust is no longer generated
            counter++;
            if (vessel.Thrust == 0 && counter % 10 == 0)
            {
                counter = 0;
                Console.WriteLine("Thrust is zero, activating next stage.");
                vessel.Control.ActivateNextStage();
            }

            if (auto_throttle)
            {
                // Calculate deltaTime
                double deltaTime = stopwatch.Elapsed.TotalSeconds;
                stopwatch.Restart();

                // Calculate TWR error
                currentTWR = vessel.Thrust / (vessel.Mass * vessel.Orbit.Body.SurfaceGravity);
                error = currentTWR - targetTWR;

                // Update Controller
                throttle = (float)i.Update(error, deltaTime);
                vessel.Control.Throttle = throttle;
            }

            // Handle the throttle button being clicked
            if (buttonClicked.Get())
            {
                auto_throttle = !auto_throttle;
                button.Text.Content = auto_throttle ? "On" : "Off";
                button.Clicked = false;
            }

            // Sleep for a short duration to prevent excessive CPU usage
            Thread.Sleep(100);
        }

        // Stop Redundant Streams and Remove UI elements
        apoapsis.Remove();
        buttonClicked.Remove();
        button.Remove();
        panel.Remove();

        // Wait until out of atmosphere
        Console.WriteLine("Coasting out of atmosphere");
        while (altitude.Get() < 70050) {
            if (ag5 && altitude.Get() > 65000) {
                vessel.Control.ToggleActionGroup(5);
                Console.WriteLine("Action group 5 activated above 65 km");
                ag5 = false;
            }
            Thread.Sleep(100);
        }
        altitude.Remove();

        // Plan circularization burn (using vis-viva equation)
        Console.WriteLine("Planning circularization burn");
        double mu = vessel.Orbit.Body.GravitationalParameter;
        double r = vessel.Orbit.Apoapsis;
        double a1 = vessel.Orbit.SemiMajorAxis;
        double a2 = r;
        double v1 = Math.Sqrt (mu * ((2.0 / r) - (1.0 / a1)));
        double v2 = Math.Sqrt (mu * ((2.0 / r) - (1.0 / a2)));
        double deltaV = v2 - v1;
        var node = vessel.Control.AddNode(ut.Get() + vessel.Orbit.TimeToApoapsis, prograde: (float)deltaV);

        // Calculate burn time (using rocket equation)
        double g0 = vessel.Orbit.Body.SurfaceGravity;
        double F = vessel.AvailableThrust;
        double Isp = vessel.SpecificImpulse * g0;
        double m0 = vessel.Mass;
        double m1 = m0 / Math.Exp(deltaV / Isp);
        double flowRate = F / Isp;
        double burnTime = (m0 - m1) / flowRate;
        double half_burnTime = burnTime / 2.0;

        // Orientate ship
        Console.WriteLine("Orientating ship for circularization burn");
        vessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
        vessel.AutoPilot.TargetDirection = Tuple.Create(0.0, 1.0, 0.0);
        vessel.AutoPilot.Wait();

        // Wait until burn
        Console.WriteLine("Waiting until circularization burn");
        double burnUT = ut.Get() + vessel.Orbit.TimeToApoapsis - half_burnTime;
        ut.Remove();
        double leadTime = 20;
        conn.SpaceCenter().WarpTo(burnUT - leadTime);

        // Execute burn
        Console.WriteLine($"{leadTime} seconds... Ready to execute burn");
        var timeToApoapsis = conn.AddStream(() => vessel.Orbit.TimeToApoapsis);
        while (timeToApoapsis.Get() - half_burnTime > 0) {
            Thread.Sleep(100);
        }
        Console.WriteLine("Executing burn");
        vessel.Control.Throttle = 1;
        Thread.Sleep((int)(burnTime * 1000));
        vessel.Control.Throttle = 0;
        Console.WriteLine("Burn finished");

        // Finalize launch
        node.Remove();
        timeToApoapsis.Remove();
        vessel.AutoPilot.Disengage();
        vessel.Control.SAS = true;
        vessel.Control.SASMode = SASMode.StabilityAssist;
        Console.WriteLine("Waiting for steering to settle down");
        Thread.Sleep(3000);
        var apoapsisAltitude = vessel.Orbit.ApoapsisAltitude / 1000.0;
        var periapsisAltitude = vessel.Orbit.PeriapsisAltitude / 1000.0;
        var semiMajorAxis = vessel.Orbit.SemiMajorAxis / 1000.0;
        var eccentricity = vessel.Orbit.Eccentricity;
        var inclination = vessel.Orbit.Inclination;
        Console.WriteLine($"  Apoapsis: {apoapsisAltitude:F3} km, Periapsis: {periapsisAltitude:F3} km");
        Console.WriteLine($"  Semi-major axis: {semiMajorAxis:F3} km, Eccentricity: {eccentricity:F4}");
        Console.WriteLine($"  Inclination: {inclination:F2} degrees");
        Console.WriteLine("Launch complete");
        conn.Dispose();
    }

    private static (int, int, bool, bool) ParseArguments(string[] args)
    {
        int target = 90000;
        int compass = 90;
        bool auto_throttle = true;
        bool ag5 = false;

        foreach (var arg in args)
        {
            if (arg == "--help" || arg == "-h")
            {
                Console.WriteLine("Usage: Launch [options]");
                Console.WriteLine("Options:");
                Console.WriteLine("  -h, --help        Show help information");
                Console.WriteLine("  -V, --version     Show version information");
                Console.WriteLine($"  --target=<int>    target altitude (default: {target})");
                Console.WriteLine($"  --compass=<int>   horizontal compass direction in degrees (default: {compass})");
                Console.WriteLine($"  --auto_throttle=<bool>   Auto Throttle (default: {auto_throttle})");
                Console.WriteLine($"  --ag5=<bool>      A boolean flag for Action Group 5 (default: {ag5})");
                Environment.Exit(0);
            }
            if (arg == "--version" || arg == "-V")
            {
                Console.WriteLine("Launch v0.1.0");
                Environment.Exit(0);
            }
            else if (arg.StartsWith("--target="))
            {
                if (int.TryParse(arg.AsSpan(9), out int parsedTarget))
                {
                    target = parsedTarget;
                }
            }
            else if (arg.StartsWith("--compass="))
            {
                if (int.TryParse(arg.AsSpan(10), out int parsedCompass))
                {
                    compass = parsedCompass;
                }
            }
            else if (arg.StartsWith("--auto_throttle="))
            {
                if (bool.TryParse(arg.AsSpan(16), out bool parsedThrottle))
                {
                    auto_throttle = parsedThrottle;
                }
            }
            else if (arg.StartsWith("--ag5="))
            {
                if (bool.TryParse(arg.AsSpan(6), out bool parsedActionGroup))
                {
                    ag5 = parsedActionGroup;
                }
            }
        }

        return (target, compass, auto_throttle, ag5);
    }

    static Tuple<Panel, Button, Stream<bool>> SetupUI(Connection conn, bool autoThrottle)
    {
        var canvas = conn.UI().StockCanvas;

        // Get the size of the game window in pixels
        var screenSize = canvas.RectTransform.Size;

        // Add a panel to contain the UI elements
        var panel = canvas.AddPanel();

        // Position the panel on the left of the screen
        var rect = panel.RectTransform;
        rect.Size = Tuple.Create(200.0, 85.0);
        rect.Position = Tuple.Create(screenSize.Item1 / 4, screenSize.Item2 / 2.3);

        // Add a button to set the throttle to maximum
        var button = panel.AddButton(autoThrottle ? "On" : "Off");
        button.RectTransform.Position = Tuple.Create(0.0, -12.0);

        // Add some text displaying the total engine thrust
        var text = panel.AddText("Auto Throttle");
        text.RectTransform.Position = Tuple.Create(0.0, 12.0);
        text.Color = Tuple.Create(1.0, 1.0, 1.0);
        text.Size = 18;

        // Hide UI elements for the beginning
        panel.Visible = false;
        button.Visible = false;

        // Set up a stream to monitor the throttle button
        var buttonClicked = conn.AddStream(() => button.Clicked);

        return new Tuple<Panel, Button, Stream<bool>>(panel, button, buttonClicked);
    }
}

public class Controller
{
    private readonly double Ki;
    private double integral;
    private (double Min, double Max) OutputLimits;
    public Controller()
    {
        Ki = 1;
        integral = 1;
        OutputLimits = (0, 1);
    }

    public void SetOutputLimits(double Min, double Max)
    {
        OutputLimits = (Min, Max);
    }

    public double Update(double current, double deltaTime)
    {
        double Target = 0;
        double error = Target - current;
        double Output = Ki * integral;

        if ((Output <= OutputLimits.Max || error <= 0) && (Output >= OutputLimits.Min || error >= 0))
        {
            integral += error * deltaTime;
        }

        return Math.Max(OutputLimits.Min, Math.Min(OutputLimits.Max, Output));
    }
}
