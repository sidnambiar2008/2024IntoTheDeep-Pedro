package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.RobotClass;
import org.firstinspires.ftc.teamcode.Utility.Positions;

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
    private CRServo leftArmServo = null;
    private CRServo rightArmServo = null;
    private CRServo intakeServo = null;
    private AnalogInput left_armPosition = null;
    private AnalogInput right_armPosition = null;


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

            leftSlideServo = hardwareMap.get(Servo.class, "left_slide");
            rightSlideServo = hardwareMap.get(Servo.class, "right_slide");
            leftArmServo = hardwareMap.get(CRServo.class, "left_arm");
            rightArmServo = hardwareMap.get(CRServo.class, "right_arm");
            intakeServo = hardwareMap.get(CRServo.class, "intake");

            left_armPosition = hardwareMap.get(AnalogInput.class, "left_arm_encoder");
            right_armPosition = hardwareMap.get(AnalogInput.class, "right_arm_encoder");
        }// Motor and Servo Init

        leftSlideServo.setPosition(0.15);
        rightSlideServo.setPosition(0.85);
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

            //leftArmServo.setPower(gamepad1.left_stick_y);
            //rightArmServo.setPower(gamepad1.left_stick_y*-1);

            if (gamepad1.right_bumper){
                leftSlideServo.setPosition(0);
                rightSlideServo.setPosition(1);
            }

            if(gamepad1.left_bumper){
                leftSlideServo.setPosition(0.15);
                rightSlideServo.setPosition(0.85);
            }

            if (gamepad1.a){
                intakeServo.setPower(-0.2);
            }

            if (gamepad1.b){
                intakeServo.setPower(0);
            }

            telemetry.addData("Left Arm Power", leftArmServo.getPower());
            telemetry.addData("Right Arm Power", rightArmServo.getPower());
            telemetry.addData("Left Arm Position", left_armPosition.getVoltage());
            telemetry.addData("Right Arm Position", right_armPosition.getVoltage());
            telemetry.addData("Left Slide Position", leftSlideServo.getPosition());
            telemetry.addData("Right Slide Position", rightSlideServo.getPosition());
            telemetry.update();


            // Send calculated powers

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

        }

    }
    //Methods

}
