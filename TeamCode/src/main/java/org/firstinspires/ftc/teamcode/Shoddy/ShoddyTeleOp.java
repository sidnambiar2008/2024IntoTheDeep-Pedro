package org.firstinspires.ftc.teamcode.Shoddy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Config
@TeleOp(name="ShoddyTeleOp", group="TeleOp")
public class ShoddyTeleOp extends LinearOpMode {

    //public DcMotor verticalEnc;

    ShoddyRobotClass r;
    ShoddyToggles t;
    ShoddyPositions po;
    private ElapsedTime runtime;


    // variables for left and right intake slide positions
    public static double positionLeft = .85, positionRight = .15;

    public boolean usePIDFvertical = false;
    public boolean usePIDFswivel = false;
    public boolean usePIDFV4B = false;

    //First PID for V4B
    private PIDController controller;
    public static double p = 0.005, i = 0.01, d = 0.00004;
    public static double f = 0.06;
    private final double ticks_in_degree = 144.0 / 180.0;
    public static int V4BTarget;
    double armPos;
    double pid, targetArmAngle, ff, currentArmAngle, V4BPower;

    //Second PID for Vertical Slides
    private PIDController controller2;
    public static double p2 = 0.006, i2 = 0.001, d2 = 0;
    public static double f2 = 0;
    private final double ticks_in_degree2 = 144.0 / 180.0;
    public static int vertSlidesTarget;
    double armPos2;
    double pid2, targetArmAngle2, ff2, currentArmAngle2, verticalSlidesPower;

    //Third PID for Swivel
    private PIDController controller3;
    public static double p3 = 0.005, i3 = 0.02, d3 = 0.0002;
    public static double f3 = 0.035;
    private final double ticks_in_degree3 = 144.0 / 180.0;
    public static int swivelTarget;
    double armPos3;
    double pid3, targetArmAngle3, ff3, currentArmAngle3, swivelPower;

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
        r = new ShoddyRobotClass(this);
        t = new ShoddyToggles(this);
        po = new ShoddyPositions();
        runtime = new ElapsedTime();

        r.wheelSetUp();
        r.servoSetUp();
        r.motorSetUp();
        r.analogSetUp();

        //r.topVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //r.bottomVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        controller3 = new PIDController(p3, i3, d3);

        //Servo Startups
        //Yada yada

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            t.copyGamepad();

            //Robot Drive (Left Stick and Right Stick X)
            {
                double max;
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
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

            // Intake Power (Right Stick Y)
            {
                r.intake.setPower(gamepad1.right_stick_y);

//                //Right trigger spin in, left trigger spin out
//                if (gamepad1.right_trigger > 0) {
//                    r.intake.setPower(-1 * (gamepad1.right_trigger) / 2);
//                } else if (gamepad1.left_trigger > 0){
//                    r.intake.setPower(1 * (gamepad1.left_trigger) / 2);
//                }

            }

            //Claw Toggles (Bumpers) Right Bumper Open/Closed, Left Bumper wrist
            {

                if (t.toggle("right_bumper")) {
                    if (t.rBumpToggle) {
                        po.CLAW_CLOSED_BOOL = false;
                        r.claw.setPosition(po.CLAW_OPEN);
                    } else {
                        po.CLAW_CLOSED_BOOL = true;
                        r.claw.setPosition(po.CLAW_CLOSED);
                    }
                }

                if (t.toggle("left_bumper")) {
                    if (t.lBumpToggle) {
                        po.CLAW_DOWN_BOOL = false;
                        r.wrist.setPosition(po.WRIST_PERP);
                    } else {
                        r.wrist.setPosition(po.WRIST_PAR);
                    }
                }
            }

            // Vertical Adjust (Triggers)
            {

            }

            //Auto Intake 100% (A)
            {
                if (t.toggle("a")) {
                    if (t.aToggle) {
                        po.INTAKE_IN_BOOL = false;

                        positionRight = po.RIGHT_SLIDE_OUT_100;
                        positionLeft = po.LEFT_SLIDE_OUT_100;

                        usePIDFV4B = true;
                        V4BTarget = po.V4B_INTAKE_POS;

                    } else {
                        po.INTAKE_IN_BOOL = true;

                        usePIDFV4B = true;
                        V4BTarget = po.V4B_REST_POS;

                        positionRight = po.RIGHT_SLIDE_IN;
                        positionLeft = po.LEFT_SLIDE_IN;
                    }
                }
            }

            // (Y)
            {
                if (t.toggle("y")) {
                    if (t.yToggle) {
                        po.INTAKE_IN_BOOL = false;

                        positionRight = po.RIGHT_SLIDE_OUT_50;
                        positionLeft = po.LEFT_SLIDE_OUT_50;

                        usePIDFV4B = true;
                        V4BTarget = po.V4B_INTAKE_POS;

                    } else {
                        po.INTAKE_IN_BOOL = true;

                        usePIDFV4B = true;
                        V4BTarget = po.V4B_REST_POS;

                        positionRight = po.RIGHT_SLIDE_IN;
                        positionLeft = po.LEFT_SLIDE_IN;

                    }
                }
            }

            //Auto Transfer (B)
            {
                if (t.toggle("b")) {
                    if (t.bToggle) {

                        positionRight = po.RIGHT_SLIDE_IN;
                        positionLeft = po.LEFT_SLIDE_IN;

                        usePIDFV4B = true;
                        V4BTarget = po.V4B_TRANSFER_POS;

                        usePIDFswivel = true;
                        swivelTarget = po.SWIVEL_DOWN;

                        r.claw.setPosition(po.CLAW_OPEN);

                        usePIDFvertical = true;
                        vertSlidesTarget = po.VERTICAL_DOWN;

                        po.INTAKE_IN_BOOL = true;
                        po.CLAW_DOWN_BOOL = true;
                        po.CLAW_CLOSED_BOOL = false;

                    } else {

                        r.claw.setPosition(po.CLAW_CLOSED);

                        usePIDFvertical = true;
                        vertSlidesTarget = po.VERTICAL_UP;

                        usePIDFswivel = true;
                        swivelTarget = po.SWIVEL_UP;
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

            //Set powers
            // Set servo positions after all logic
            r.rightLinear.setPosition(positionRight);
            r.leftLinear.setPosition(positionLeft);

            if (usePIDFswivel){
                setSwivelPIDF(swivelTarget);
            }

            if (usePIDFvertical){
                setVerticalSlidesPIDF(vertSlidesTarget);
            }

            if (usePIDFV4B){
                setV4BPIDF(V4BTarget);
            }

            //Telemetry
            telemetry.addData("runtime", runtime.milliseconds());
            telemetry.addData("V4B Pos", r.rightV4BEnc.getCurrentPosition());
            telemetry.addData("V4B Target", V4BTarget);
            telemetry.addData("Swivel Pos", r.rightSwivelEnc.getCurrentPosition());
            telemetry.addData("Swivel Target", swivelTarget);
            telemetry.addData("Vert Pos", r.topVertical.getCurrentPosition());
            telemetry.addData("Vert Target", vertSlidesTarget);
            telemetry.update();

        }
    }
    // Methods
    private void setV4BPIDF(int target) {
        controller.setPID(p, i, d);
        armPos = r.rightV4BEnc.getCurrentPosition();
        pid = controller.calculate(armPos, target);
        targetArmAngle = target;
        ff = (Math.sin(Math.toRadians(targetArmAngle))) * f;
        currentArmAngle = Math.toRadians((armPos) / ticks_in_degree);

        V4BPower = pid + ff;

        r.leftArm.setPower(V4BPower);
        r.rightArm.setPower(V4BPower);
    }

    private void setVerticalSlidesPIDF(int target2) {
        controller2.setPID(p2, i2, d2);
        armPos2 = r.topVertical.getCurrentPosition();
        pid2 = controller2.calculate(armPos2, target2);
        targetArmAngle2 = Math.toRadians((target2) / ticks_in_degree2);
        ff2 = targetArmAngle2 * f2;
        currentArmAngle2 = Math.toRadians((armPos2) / ticks_in_degree2);

        verticalSlidesPower = pid2 + ff2;

        r.topVertical.setPower(verticalSlidesPower);
        r.bottomVertical.setPower(verticalSlidesPower);
    }

    private void setSwivelPIDF(int target3) {
        controller3.setPID(p3, i3, d3);
        armPos3 = r.rightSwivelEnc.getCurrentPosition();
        pid3 = controller3.calculate(armPos3, target3);
        targetArmAngle3 = target3;
        ff3 = (Math.cos(Math.toRadians(targetArmAngle3))) * f3;
        currentArmAngle3 = Math.toRadians((armPos3) / ticks_in_degree3);

        swivelPower = pid3 + ff3;

        r.leftSwivel.setPower(swivelPower);
        r.rightSwivel.setPower(swivelPower);
    }

    private void setLinearPower(double pos){
        r.rightLinear.setPosition(pos);
        r.leftLinear.setPosition(pos);
    }
}

