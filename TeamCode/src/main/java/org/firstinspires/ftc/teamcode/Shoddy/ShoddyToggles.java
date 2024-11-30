package org.firstinspires.ftc.teamcode.Shoddy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ShoddyToggles {
    private LinearOpMode myOpMode = null;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public boolean lBumpToggle = false;
    public boolean rBumpToggle = false;
    public boolean slowModeToggle = false;
    public boolean aToggle = false;
    public boolean yToggle = false;


    public ShoddyToggles(LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void copyGamepad(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(myOpMode.gamepad1);
    }

    public boolean toggle(String button) {
        if (button.equals("right_bumper")) {
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                rBumpToggle = !rBumpToggle;
                return true;
            }
        } else if (button.equals("left_bumper")) {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                lBumpToggle = !lBumpToggle;
                return true;
            }
        } else if (button.equals("x")) {
            if (currentGamepad1.x && !previousGamepad1.x) {
                slowModeToggle = !slowModeToggle;
                return true;
            }
        } else if (button.equals("a")) {
            if (currentGamepad1.a && !previousGamepad1.a) {
                aToggle = !aToggle;
                return true;
            }
        } else if (button.equals("y")) {
            if (currentGamepad1.y && !previousGamepad1.y) {
                yToggle = !yToggle;
                return true;
            }
        }
        return false;
    }
    public boolean toggle(boolean bool){
        return !bool;
    }
}
