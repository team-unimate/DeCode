package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.lib.SubSystemGroup;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class Drivetrain extends SubSystemGroup {
    Field field = new Field();
    LimelightLocalizer limelightLocalizer;
    STATES state = STATES.TELEOP_DRIVING;

    DTFollower follower;
    private boolean AtShootingZone = false;
    public Drivetrain(){
        field.updateRobotPose(new Pose(0,0,0));
        field.addObstacle(new Field.Triangle(
                0, 143, 143, 143, 143/2, 143/2
        ));

        limelightLocalizer = new LimelightLocalizer();
        follower = new DTFollower();
        Subsystem[] subsystems = {
                follower,
                limelightLocalizer
        };
        scheduleSubsystems(subsystems);
    }
    public Follower getFollower(){
        return follower.getFollower();
    }
    @Override
    public void GroupPeriodic(Telemetry telemetry) {
        Pose currentPose = getFollower().getPose();
        if (currentPose!=null) {
        field.updateRobotPose(getFollower().getPose());

        AtShootingZone = field.checkCollision();

            }


    }
    public enum TELEOP_STATES {
        SHOOTING,
        NORMAL
    }
    TELEOP_STATES teleopState = TELEOP_STATES.NORMAL;
    public void StartTeleOpDrive(
    ){
        state = STATES.TELEOP_DRIVING;
        follower.getFollower().startTeleopDrive();
    }
    public enum PARKS{
        RED_PARK,
        BLUE_PARK
    }

    public enum STATES{
        AUTO_DRIVING,
        TELEOP_DRIVING
    }

    public void AutoPark(PARKS park){
        state = STATES.AUTO_DRIVING;
        Pose parkPose = new Pose(28.3+9, 23.6+9, 0);
        switch (park){
            case RED_PARK:
                Path parkRedPath = new Path(new BezierLine(follower.getFollower().getPose(),parkPose));
                parkRedPath.setLinearHeadingInterpolation(getFollower().getPose().getHeading(), 0);
                follower.getFollower().followPath(
                        parkRedPath
                );
                break;
            case BLUE_PARK:
                Path parkBluePath = new Path(new BezierLine(follower.getFollower().getPose(),parkPose.mirror()));
                parkBluePath.setLinearHeadingInterpolation(getFollower().getPose().getHeading(), 0);
                follower.getFollower().followPath(parkBluePath);
                break;
        }
    }

    public void SetTeleOpState(TELEOP_STATES state){
        teleopState = state;
    }
    public void DriveTeleOp(Gamepad gamepad){
        double multiplier = 1;
        if (teleopState==TELEOP_STATES.NORMAL){
            multiplier=1;
        } else {
            multiplier = 0.6;
        }
        follower.getFollower().setTeleOpDrive(-gamepad.left_stick_y,
                -gamepad.left_stick_x*multiplier,
                -gamepad.right_stick_x*multiplier,
                false);
    }
    public void SetPose(Pose estimatedPose){
        follower.getFollower().setStartingPose(estimatedPose);
        field.updateRobotPose(estimatedPose);
    }
    public boolean isAtShootingZone() {
        return AtShootingZone;
    }
}
