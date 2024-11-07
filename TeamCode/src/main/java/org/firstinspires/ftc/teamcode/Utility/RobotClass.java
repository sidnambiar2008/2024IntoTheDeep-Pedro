package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotClass {
    private LinearOpMode myOpMode = null;

    //Wheel Motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;

    //Other Motors

    //Servos
    public Servo leftSlideServo;
    public Servo rightSlideServo;
    public Servo leftArmServo;
    public Servo rightArmServo;
    public Servo intakeServo;

    //Other Variables


    //Drive Robot Variables
    public static double speedVar = 1;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotClass(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Methods
    public void wheelSetUp(){

        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftback_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightback_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    public void servoSetUp(){

        leftSlideServo = myOpMode.hardwareMap.get(Servo.class, "left_slide");
        rightSlideServo = myOpMode.hardwareMap.get(Servo.class, "right_slide");
        leftArmServo = myOpMode.hardwareMap.get(Servo.class, "left_arm");
        rightArmServo = myOpMode.hardwareMap.get(Servo.class, "right_arm");
        intakeServo = myOpMode.hardwareMap.get(Servo.class, "intake");
    }

    public void driveRobot(double lf, double rf, double lb, double rb){
        // Send calculated power to wheels
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

}
