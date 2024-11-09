package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Toggles {
    private LinearOpMode myOpMode = null;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public boolean lBumpToggle = false;
    public boolean rBumpToggle = false;
    public boolean slowModeToggle;


    public Toggles(LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void copyGamepad(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(myOpMode.gamepad1);
    }

    public void toggle(String button){
        if (button.equals("right_bumper")){
            if(currentGamepad1.right_bumper && previousGamepad1.right_bumper){
                rBumpToggle = !rBumpToggle;
            }
        } else if (button.equals("left_bumper")){
            if(currentGamepad1.left_bumper && previousGamepad1.left_bumper){
                lBumpToggle = !lBumpToggle;
            }
        } else if (button.equals("x")) {
            if (currentGamepad1.x && previousGamepad1.x) {
                slowModeToggle = !slowModeToggle;
            }
        }
    }
}
