// dotnet add package KRPC.Client --version 0.5.4
// dotnet add package Google.Protobuf --version 3.30.1

using System;
using System.Threading;
using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;

class Program
{
    public static void Main()
    {
        Console.Clear();
        using var conn = new Connection(name: "Kerbin reentry maneuver");
        var spaceCenter = conn.SpaceCenter();
        var vessel = spaceCenter.ActiveVessel;

        Console.WriteLine("Started reentry maneuver");
        vessel.Control.SAS = true;
        Thread.Sleep(2000);
        vessel.Control.SASMode = SASMode.Retrograde;

        Console.WriteLine("Waiting until altitude < 75Km");
        while (vessel.Flight().MeanAltitude >= 75000)
        {
            Thread.Sleep(1000);
        }

        // Do reentry staging
        // vessel.Control.ActivateNextStage();
        Console.WriteLine("AG10 active: last staging before reentry");
        vessel.Control.ToggleActionGroup(0);

        // Arm parachutes
        DoParachute(vessel);

        // Unlock all
        vessel.Control.SAS = false;
        vessel.Control.RCS = false;

        Console.WriteLine("Script exited.");
    }

    static void DoParachute(Vessel vessel)
    {
        while (vessel.Flight().SurfaceAltitude > 5000)
        {
            Thread.Sleep(1000);
        }
        Console.WriteLine("Parachutes");
        foreach (var parachute in vessel.Parts.Parachutes)
        {
            parachute.Arm();
        }
    }
}
