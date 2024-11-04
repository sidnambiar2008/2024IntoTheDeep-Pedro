package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.RobotClass;

@TeleOp(name="TeleTest", group="Linear OpMode")
public class TeleOpTest extends LinearOpMode {
    RobotClass robot = new RobotClass(this);
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo leftSlideServo = null;
    private Servo rightSlideServo = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private Servo intakeServo = null;
    @Override
    public void runOpMode() {

        {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "leftback_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            Servo leftSlideServo;
            Servo rightSlideServo;
            Servo leftArmServo;
            Servo rightArmServo;
            Servo intakeServo;

            leftSlideServo = hardwareMap.get(Servo.class, "left_slide");
            rightSlideServo = hardwareMap.get(Servo.class, "right_slide");
            leftArmServo = hardwareMap.get(Servo.class, "left_arm");
            rightArmServo = hardwareMap.get(Servo.class, "right_arm");
            intakeServo = hardwareMap.get(Servo.class, "intake");
        }// Motor and Servo Init

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }



            // Send calculated powers
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }

    }
    //Methods
    
}
