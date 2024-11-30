package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shoddy.ShoddyRobotClass;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.Toggles;

@Config
@TeleOp(name="ServoTest", group="Test")
public class ServoTest extends LinearOpMode {

    Toggles toggles = new Toggles(this);
    //Robot robot;
    ShoddyRobotClass robot;

    private ElapsedTime runtime = new ElapsedTime();
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    double incrementValue = 0.05;
    boolean incrementLarge = true;
    public static double positionLeft = .85, positionRight = .15;

    @Override
    public void runOpMode(){
        //robot = new Robot(this);
        robot = new ShoddyRobotClass(this);

        robot.servoSetUp();
        robot.analogSetUp();
        robot.motorSetUp();
        robot.topVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bottomVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (gamepad1.x){
                if (currentGamepad1.x && previousGamepad1.x) {
                     incrementLarge = !incrementLarge;
                     if (incrementLarge){
                         incrementValue = 0.05;
                     } else {
                         incrementValue = 0.01;
                     }
                }
            }

            if (gamepad1.right_bumper) {
                if (currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
//                    robot.leftLinear.setPosition(robot.leftLinear.getPosition() + incrementValue);
//                    robot.rightLinear.setPosition(robot.rightLinear.getPosition() - incrementValue);
                    positionLeft = robot.leftLinear.getPosition() + incrementValue;
                    positionRight = robot.rightLinear.getPosition() - incrementValue;
                }
            }

            if (gamepad1.left_bumper) {
                if (currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
//                    robot.leftLinear.setPosition(robot.leftLinear.getPosition() - incrementValue);
//                    robot.rightLinear.setPosition(robot.rightLinear.getPosition() + incrementValue);
                    positionLeft = robot.leftLinear.getPosition() - incrementValue;
                    positionRight = robot.rightLinear.getPosition() + incrementValue;
                }
            }

            robot.leftLinear.setPosition(positionLeft);
            robot.rightLinear.setPosition(positionRight);

            robot.bottomVertical.setPower(gamepad1.left_stick_y);
            robot.topVertical.setPower((gamepad1.left_stick_y)*-1);
            //bottom vertical positive goes up, top vertical negative goes up

            telemetry.addData("Increment Value:", incrementValue);
            telemetry.addData("Left Slide Position", robot.leftLinear.getPosition());
            //telemetry.addData("Right Slide Voltage", robot.leftArmAnalog.getVoltage());
            telemetry.addData("Right Slide Position", robot.rightLinear.getPosition());
            //telemetry.addData("Right Slide Position", robot.rightArmAnalog.getVoltage());
            telemetry.update();
        }
    }
}
