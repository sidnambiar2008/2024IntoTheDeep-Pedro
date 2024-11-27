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

    AbsoluteAnalogEncoder rightV4BEnc;
    AbsoluteAnalogEncoder leftV4BEnc;
    AbsoluteAnalogEncoder rightSwivelEnc;
    AbsoluteAnalogEncoder leftSwivelEnc;

    //First PID for V4B
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    private final double ticks_in_degree = 144.0 / 180.0;
    public static int V4BTarget;
    double armPos;
    double pid, targetArmAngle, ff, currentArmAngle, V4BPower;

    //Second PID for Vertical Slides
    private PIDController controller2;
    public static double p2 = 0, i2 = 0, d2 = 0;
    public static double f2 = 0;
    private final double ticks_in_degree2 = 144.0 / 180.0;
    public static int vertSlidesTarget;
    double armPos2;
    double pid2, targetArmAngle2, ff2, currentArmAngle2, verticalSlidesPower;

    //Third PID for Swivel
    private PIDController controller3;
    public static double p3 = 0, i3 = 0, d3 = 0;
    public static double f3 = 0;
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

        rightV4BEnc = new AbsoluteAnalogEncoder(r.rightArmAnalog);
        leftV4BEnc = new AbsoluteAnalogEncoder(r.leftArmAnalog);
        rightSwivelEnc = new AbsoluteAnalogEncoder(r.rightSwivelAnalog);
        leftSwivelEnc = new AbsoluteAnalogEncoder(r.leftSwivelAnalog);

        //verticalEnc = hardwareMap.get(DcMotor.class, "empty_motor");

        r.wheelSetUp();
        r.servoSetUp();
        r.motorSetUp();
        r.analogSetUp();
        r.topVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        r.rightSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        r.rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        r.topVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.bottomVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        controller3 = new PIDController(p3, i3, d3);


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
                    po.CLAW_CLOSED_BOOL = false;
                    r.claw.setPosition(po.CLAW_OPEN);
                } else {
                    po.CLAW_CLOSED_BOOL = true;
                    r.claw.setPosition(po.CLAW_CLOSED);
                }

                t.toggle("left_bumper");
                if (t.lBumpToggle) {
                    po.CLAW_DOWN_BOOL = false;
                    r.wrist.setPosition(po.WRIST_PERP);
                } else {
                    r.wrist.setPosition(po.WRIST_PAR);
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

            //Auto Intake 100% (A)
            {
                t.toggle("a");
                if (t.aToggle){
                    po.INTAKE_IN_BOOL = false;
                    r.rightLinear.setPosition(po.RIGHT_SLIDE_OUT_100);
                    r.leftLinear.setPosition(po.LEFT_SLIDE_OUT_100);
                    setV4BPIDF(po.V4B_INTAKE_POS);
                } else {
                    po.INTAKE_IN_BOOL = true;
                    r.rightLinear.setPosition(po.RIGHT_SLIDE_IN);
                    r.leftLinear.setPosition(po.LEFT_SLIDE_IN);
                    setV4BPIDF(po.V4B_REST_POS);
                    }
            }

            // (Y)
            {
                t.toggle("y");
                if (t.yToggle){
                    po.INTAKE_IN_BOOL = false;
                    r.rightLinear.setPosition(po.RIGHT_SLIDE_OUT_50);
                    r.leftLinear.setPosition(po.LEFT_SLIDE_OUT_50);
                    setV4BPIDF(po.V4B_INTAKE_POS);
                } else {
                    po.INTAKE_IN_BOOL = true;
                    r.rightLinear.setPosition(po.RIGHT_SLIDE_IN);
                    r.leftLinear.setPosition(po.LEFT_SLIDE_IN);
                    setV4BPIDF(po.V4B_REST_POS);
                }
            }

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
            telemetry.addData("lForebarPower", r.leftArm.getPower());
            telemetry.addData("rForebarPower", r.rightArm.getPower());
            //telemetry.addData("botVerticalPos", r.bottomVertical.getCurrentPosition());
            //telemetry.addData("topVerticalPos", r.topVertical.getCurrentPosition());
            //telemetry.addData("topVerticalPos", r.topVertical.getCurrentPosition());
            telemetry.addData("leftLinear", r.leftLinear.getPosition());
            telemetry.addData("rightLinear", r.rightLinear.getPosition());
            telemetry.update();

        }
    }
    // Methods
    private void setV4BPIDF(int target) {
        controller.setPID(p, i, d);
        armPos = rightV4BEnc.getCurrentPosition();
        pid = controller.calculate(armPos, target);
        targetArmAngle = Math.toRadians((target) / ticks_in_degree);
        ff = Math.cos(targetArmAngle) * f;
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
        ff2 = Math.cos(targetArmAngle2) * f2;
        currentArmAngle2 = Math.toRadians((armPos2) / ticks_in_degree2);

        verticalSlidesPower = pid2 + ff2;

        r.topVertical.setPower(verticalSlidesPower);
        r.bottomVertical.setPower(verticalSlidesPower);
    }

    private void setSwivelPIDF(int target3) {
        controller3.setPID(p3, i3, d3);
        armPos3 = rightSwivelEnc.getCurrentPosition();
        pid3 = controller3.calculate(armPos3, target3);
        targetArmAngle3 = Math.toRadians((target3) / ticks_in_degree3);
        ff3 = Math.cos(targetArmAngle3) * f3;
        currentArmAngle3 = Math.toRadians((armPos3) / ticks_in_degree3);

        swivelPower = pid3 + ff3;

        r.leftSwivel.setPower(swivelPower);
        r.rightSwivel.setPower(swivelPower);
    }
}

