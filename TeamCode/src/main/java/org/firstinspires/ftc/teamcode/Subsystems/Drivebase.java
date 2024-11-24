package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Drivebase implements Subsystem {

    public Follower drive;
    public static final double CENTER_DIST = 8.25;
    public boolean driveDisable = false;

    //Constructor
    public Drivebase(HardwareMap map, Pose pos) {
        drive = new Follower(map);
        drive.setPose(pos);
    }
    public Drivebase(HardwareMap map) {
        this(map, new Pose(0, 0, 0));
    }

    //Methods
    public void setPose(Pose pose) {
        drive.setPose(pose);
    }

    public void setPower(Pose powers) {
        drive.setTeleOpMovementVectors(powers.getX(), powers.getY(), powers.getHeading());
    }

    public Follower getDrive() {
        return drive;
    }

    //Interface Methods
    @Override
    public void toInit(){
        //startupcode
    }

    @Override
    public void update(){
    }

}
