package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class RobotClass {
    private LinearOpMode myOpMode = null;

    //Wheel Motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;

    //Other Motors

    public DcMotor rightVerticalSlide;
    public DcMotor leftVerticalSlide;

    //Servos
    public Servo rightSlideServo;
    public Servo leftSlideServo;
    public CRServo rightArmServo;
    public CRServo leftArmServo;
    public CRServo intakeServo;

    //Other Variables


    //Drive Robot Variables
    public static double speedVar = 1;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotClass(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Methods
    public void wheelSetUpA(){

        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftback_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightback_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public void wheelSetUpB(){

        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftback_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightback_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public void servoSetUpA(){

        rightSlideServo = myOpMode.hardwareMap.get(Servo.class, "right_slide");
        leftSlideServo = myOpMode.hardwareMap.get(Servo.class, "left_slide");
        rightArmServo = myOpMode.hardwareMap.get(CRServo.class, "right_arm");
        leftArmServo = myOpMode.hardwareMap.get(CRServo.class, "left_arm");
        intakeServo = myOpMode.hardwareMap.get(CRServo.class, "intake");
    }
    public void motorSetUpA(){
        rightVerticalSlide = myOpMode.hardwareMap.get(DcMotor.class, "right_vertical");
        leftVerticalSlide = myOpMode.hardwareMap.get(DcMotor.class, "left_vertical");
    }

    public void servoSetUpB()
    {

    }

    public void motorSetUpB()
    {

    }

    public void driveRobot(double lf, double rf, double lb, double rb){
        // Send calculated power to wheels
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

}
