// dotnet add package KRPC.Client --version 0.5.4
// dotnet add package Google.Protobuf --version 3.30.1

// using System;
// using System.Linq;
// using System.Threading;
using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;

class ManeuverExecution
{
    static void Main(string[] args)
    {
        string? circularizeAt = ParseArguments(args);
        
        try
        {
            ExecuteManeuverNode(circularizeAt);
        }
        catch (Exception e)
        {
            Console.WriteLine($"\nAn unexpected error occurred: {e.Message}");
        }
    }

    static void ExecuteManeuverNode(string? circularizeAt)
    {
        using var conn = new Connection(name: "Maneuver Execution");
        var spaceCenter = conn.SpaceCenter();
        var vessel = spaceCenter.ActiveVessel;
        vessel.Control.SAS = false;

        Console.Clear();

        if (circularizeAt != null){
            Console.WriteLine("Planning circularization burn");
            double mu = vessel.Orbit.Body.GravitationalParameter;
            double r = circularizeAt == "ap" ? vessel.Orbit.Apoapsis : vessel.Orbit.Periapsis;
            double a1 = vessel.Orbit.SemiMajorAxis;
            double a2 = r;
            double v1 = Math.Sqrt(mu * ((2.0 / r) - (1.0 / a1)));
            double v2 = Math.Sqrt(mu * ((2.0 / r) - (1.0 / a2)));
            double deltaV = v2 - v1;
            double timeToBurn = circularizeAt == "ap" ? vessel.Orbit.TimeToApoapsis : vessel.Orbit.TimeToPeriapsis;
            vessel.Control.AddNode(conn.SpaceCenter().UT + timeToBurn, prograde: (float)deltaV);
            Thread.Sleep(1000);
        }

        if (vessel.Control.Nodes.Count == 0)
        {
            Console.WriteLine("No maneuver node exists.");
            return;
        }

        var node = vessel.Control.Nodes[0];
        double startTime = CalculateStartTime(conn, node);

        spaceCenter.WarpTo(startTime - 30);
        vessel.AutoPilot.Engage();
        vessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
        vessel.AutoPilot.TargetDirection = node.BurnVector(node.ReferenceFrame);
        Console.WriteLine("Waiting until maneuver start...");

        CountdownToManeuver(conn, startTime);

        vessel.Control.Throttle = 1;
        var originalVector = node.BurnVector(node.ReferenceFrame);
        Console.WriteLine("Maneuver in progress...");

        while (!IsManeuverComplete(vessel, originalVector))
        {
            Thread.Sleep(100);
        }

        vessel.Control.Throttle = 0;
        vessel.AutoPilot.Disengage();
        node.Remove();
        vessel.Control.SAS = true;
        Thread.Sleep(1000);
        Console.WriteLine("Script exited.");
    }

    static double CalculateStartTime(Connection conn, Node node)
    {
        double burnTime = CalculateBurnTime(conn, node);
        return conn.SpaceCenter().UT + node.TimeTo - burnTime / 2;
    }

    static double CalculateBurnTime(Connection conn, Node node)
    {
        double dV = node.DeltaV;
        var vessel = conn.SpaceCenter().ActiveVessel;
        double g0 = vessel.Orbit.Body.SurfaceGravity;

        var activeEngines = vessel.Parts.Engines.Where(engine => engine.Active).ToList();
        double totalIsp = activeEngines.Sum(engine => engine.SpecificImpulse);
        double isp = activeEngines.Count > 0 ? totalIsp / activeEngines.Count : 0;

        if (isp == 0)
        {
            // throw new DivideByZeroException("No active engines with ISP found.");
            throw new InvalidOperationException("No active engines with ISP found.");
        }

        Console.WriteLine("Available Engines:");
        foreach (var engine in activeEngines)
        {
            Console.WriteLine($"  Engine: {engine.Part.Title}, ISP: {engine.SpecificImpulse}");
        }

        double mf = vessel.Mass / Math.Exp(dV / (isp * g0));
        double fuelFlow = vessel.AvailableThrust / (isp * g0);
        double burnTime = (vessel.Mass - mf) / fuelFlow;

        Console.WriteLine($"Maneuver duration: {burnTime:F2}s.");
        return burnTime;
    }

    static void CountdownToManeuver(Connection conn, double startTime)
    {
        for (int i = 5; i > 0; i--)
        {
            while (conn.SpaceCenter().UT < startTime - i)
            {
                Thread.Sleep(100);
            }
            Console.WriteLine($"...{i}");
        }
    }

    static bool IsManeuverComplete(Vessel vessel, Tuple<double, double, double> originalVector, double threshold = 0.2)
    {
        var node = vessel.Control.Nodes.FirstOrDefault();
        if (node == null)
        {
            return true; // No maneuver node, consider complete
        }

        var burnVector = node.BurnVector(node.ReferenceFrame);
        if (VectorAngle(originalVector, burnVector) > 90)
        {
            return true;
        }

        return node.RemainingDeltaV < threshold;
    }

    static double VectorAngle(Tuple<double, double, double> v1, Tuple<double, double, double> v2)
    {
        double dotProduct = v1.Item1 * v2.Item1 + v1.Item2 * v2.Item2 + v1.Item3 * v2.Item3;
        double magV1 = Math.Sqrt(v1.Item1 * v1.Item1 + v1.Item2 * v1.Item2 + v1.Item3 * v1.Item3);
        double magV2 = Math.Sqrt(v2.Item1 * v2.Item1 + v2.Item2 * v2.Item2 + v2.Item3 * v2.Item3);

        try
        {
            return Math.Acos(dotProduct / (magV1 * magV2)) * (180f / Math.PI);
        }
        catch (Exception)
        {
            return 0; // Handle edge cases where acos input is out of range
        }
    }

    private static string? ParseArguments(string[] args)
    {
        string? circularizeAt = null;

        foreach (var arg in args)
        {
            if (arg == "--help" || arg == "-h")
            {
                Console.WriteLine("Usage: Maneuver_Node [options]");
                Console.WriteLine("Options:");
                Console.WriteLine("  -h, --help        Show help information");
                Console.WriteLine("  -V, --version     Show version information");
                Console.WriteLine("  --circularize_at=<ap|pe> Choose either 'ap' or 'pe'.");
                Environment.Exit(0);
            }
            if (arg == "--version" || arg == "-V")
            {
                Console.WriteLine("Maneuver_Node v0.1.0");
                Environment.Exit(0);
            }
            else if (arg.StartsWith("--circularize_at="))
            {
                string value = arg[17..];
                if (value == "ap" || value == "pe")
                {
                    circularizeAt = value;
                }
            }
        }

        return circularizeAt;
    }
}
