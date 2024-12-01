package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous(name="AutoNetZone", group="Auto")

public class AutoNetZone extends LinearOpMode
{
    //Variables


    //Pedro Pathing
    private boolean forward = true;
    private Follower follower;

    //Paths
    private Path forwards;
    private Path backwards;

    public void runOpMode() throws InterruptedException
    {

    }
}