package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@TeleOp(name = "Voltage", group = "TeleOp")
public class VoltageTest extends LinearOpMode {

    public AnalogInput leftArmAnalog;
    public AnalogInput rightArmAnalog;
    public CRServo rightServo;


    public void runOpMode(){
        leftArmAnalog = hardwareMap.get(AnalogInput.class, "left_analog");
        rightArmAnalog = hardwareMap.get(AnalogInput.class, "right_analog");
        rightServo = hardwareMap.get(CRServo.class, "right_arm");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()){
            rightServo.setPower(0);
            telemetry.addData("leftVoltage", leftArmAnalog.getVoltage());
            telemetry.addData("rightVoltage", rightArmAnalog.getVoltage());
            telemetry.update();
        }
    }
}
