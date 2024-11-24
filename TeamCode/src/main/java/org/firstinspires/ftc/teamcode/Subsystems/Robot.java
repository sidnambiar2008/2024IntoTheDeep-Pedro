package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Robot {

    //Variables
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public Gamepad gp1, gp2;

    public IntakeArm intakeArm;
    public static VoltageSensor voltageSensor;
    public OuttakeArm outtakeArm;
    public Drivebase drivebase;

    public List<Subsystem> subsystems;

    //Constructors
    public Robot(HardwareMap map, Telemetry t, Gamepad gp1, Gamepad gp2){
        hardwareMap = map;
        telemetry = t;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        drivebase = new Drivebase(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap);
        intakeArm = new IntakeArm(hardwareMap);
        subsystems = new ArrayList<>(Arrays.asList( outtakeArm, intakeArm, drivebase));
        this.gp1 = gp1;
        this.gp2 = gp2;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    //Methods
    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }


    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }
}
