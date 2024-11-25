package org.firstinspires.ftc.teamcode.Shoddy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp(name="ShoddyTeleOp", group="TeleOp")
public class ShoddyTeleOp extends LinearOpMode {

    ShoddyRobotClass r = new ShoddyRobotClass(this);
    ShoddyToggles t = new ShoddyToggles(this);
    ShoddyPositions po = new ShoddyPositions();
    private ElapsedTime runtime = new ElapsedTime();

    //First PID
    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0.0002;
    public static double f = -0.15;
    private final double ticks_in_degree = 144.0 / 180.0;
    public static int target;
    public static double offset = -25;
    double armPos;
    double pid, targetArmAngle, ff, currentArmAngle, intakeArmPower;

    //Second PID
    private PIDController controller2;
    public static double p2 = 0.02, i2 = 0, d2 = 0.0002;
    public static double f2 = 0;
    private final double ticks_in_degree2 = 144.0 / 180.0;
    public static int target2;
    int armPos2;
    double pid2, targetArmAngle2, ff2, currentArmAngle2, outtakeArmPower;

    //ENUMS
    public enum RightStickY{
        OUTTAKE_VERTICAL,
        FOREBAR_ARMS,
        SWIVEL,
        WRIST,
        CLAW
    };
    RightStickY rightStickY = RightStickY.FOREBAR_ARMS;

    @Override
    public void runOpMode() {
        r.wheelSetUp();
        r.servoSetUp();
        r.motorSetUp();
        r.analogSetUp();
        r.topVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        r.rightSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        r.rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        r.topVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.bottomVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            t.copyGamepad();

            //Robot Drive (Left Stick and Right Stick X)
            {
                double max;
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));
                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                leftFrontPower *= po.speed;
                rightFrontPower *= po.speed;
                leftBackPower *= po.speed;
                leftFrontPower *= po.speed;

                // Send calculated power to wheels
                r.driveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            //Right Stick Y)
            {
                switch (rightStickY){
                    case FOREBAR_ARMS:
                        r.leftArm.setPower(gamepad1.right_stick_y);
                        r.rightArm.setPower(gamepad1.right_stick_y);
                        break;
                    case OUTTAKE_VERTICAL:
                        r.bottomVertical.setPower(gamepad1.right_stick_y);
                        r.topVertical.setPower(gamepad1.right_stick_y);
                        break;
                    case SWIVEL:
                        r.leftSwivel.setPower(gamepad1.right_stick_y);
                        r.rightSwivel.setPower(gamepad1.right_stick_y);
                        break;
                    case WRIST:
                        //r.wrist.setPosition((gamepad1.right_stick_y+1)/2);
                        break;
                    case CLAW:
                        //r.claw.setPosition((gamepad1.right_stick_y+1)/2);
                        break;
                }
            }

            //Claw Toggles (Bumpers) Right Bumper Open/Closed, Left Bumper wrist
            {
                t.toggle("right_bumper");
                if (t.rBumpToggle) {
                    r.claw.setPosition(po.clawOpen);
                } else {
                    r.claw.setPosition(po.clawClosed);
                }

                t.toggle("left_bumper");
                if (t.lBumpToggle) {
                    r.wrist.setPosition(po.wristPerp);
                } else {
                    r.wrist.setPosition(po.wristPar);
                }
            }

            //Intake Spin in / out (Triggers)
            {
                //Right trigger spin in, left trigger spin out
                if (gamepad1.right_trigger > 0) {
                    r.intake.setPower(-1 * (gamepad1.right_trigger) / 2);
                } else if (gamepad1.left_trigger > 0){
                    r.intake.setPower(1 * (gamepad1.left_trigger) / 2);
                }
            }

            //Linear Slides (A)
            {
                t.toggle("a");
                if (t.aToggle){
                        r.rightLinear.setPosition(po.rightSlideOut);
                        r.leftLinear.setPosition(po.leftSlideOut);
                    } else {
                        r.rightLinear.setPosition(po.rightSlideIn);
                        r.leftLinear.setPosition(po.leftSlideIn);
                    }
            }

            // (Y)
            {}

            //Switch RightStick Y (B)
            {
                if (gamepad1.b && !t.previousGamepad1.b){
                    switch (rightStickY){
                        case FOREBAR_ARMS:
                            rightStickY = RightStickY.OUTTAKE_VERTICAL;
                            break;
                        case OUTTAKE_VERTICAL:
                            rightStickY = RightStickY.SWIVEL;
                            break;
                        case SWIVEL:
                            rightStickY = RightStickY.WRIST;
                            break;
                        case WRIST:
                            rightStickY = RightStickY.CLAW;
                            break;
                        case CLAW:
                            rightStickY = RightStickY.FOREBAR_ARMS;
                            break;
                    }
                }
            }

            //Slow Mode Toggle (X)
            {
                t.toggle("x");
                if (t.slowModeToggle){
                    po.speed *= po.slowMultiplier;
                } else {
                    po.speed = po.maxSpeed;
                }
            }


            //Telemetry
            telemetry.addData("rightStickY", rightStickY);
            telemetry.addData("clawPos", r.claw.getPosition());
            telemetry.addData("wristPos", r.wrist.getPosition());
            telemetry.addData("lForebarVolt", r.leftArmAnalog.getVoltage());
            telemetry.addData("rForebarVolt", r.rightArmAnalog.getVoltage());
            telemetry.addData("botVerticalPos", r.bottomVertical.getCurrentPosition());
            telemetry.addData("topVerticalPos", r.topVertical.getCurrentPosition());
            telemetry.addData("topVerticalPos", r.topVertical.getCurrentPosition());
            telemetry.addData("leftLinear", r.leftLinear.getPosition());
            telemetry.addData("rightLinear", r.rightLinear.getPosition());
            telemetry.update();

        }
    }
    // Methods
//    private void SetInttakePIDTarget(int target) {
//        controller.setPID(p, i, d);
//        pid = controller.calculate(armPos, target);
//        targetArmAngle = Math.toRadians((target - offset) / ticks_in_degree);
//        ff = Math.cos(targetArmAngle) * f;
//        currentArmAngle = Math.toRadians((armPos - offset) / ticks_in_degree);
//
//        intakeArmPower = pid + ff;
//
//        //intake.setpower
//    }

    private void SetOuttakePIDTarget(int target2) {
        controller2.setPID(p2, i2, d2);
        armPos2 = r.topVertical.getCurrentPosition();
        pid2 = controller2.calculate(armPos2, target2);
        targetArmAngle2 = Math.toRadians((target2) / ticks_in_degree2);
        ff2 = Math.cos(targetArmAngle2) * f2;
        currentArmAngle2 = Math.toRadians((armPos2) / ticks_in_degree2);

        outtakeArmPower = pid2; // + ff2;

        r.topVertical.setPower(outtakeArmPower);
        r.bottomVertical.setPower(outtakeArmPower);
    }
}

