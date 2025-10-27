package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarSidePaths {
    public PathChain StartToFirstRow;
    public PathChain IntakeFirstRow;
    public PathChain FirstRowToShoot;
    public PathChain ShootToSecondRow;
    public PathChain IntakeSecondRow;
    public PathChain SecondRowToShoot;
    public PathChain ShootToThirdRow;
    public PathChain IntakeThirdRow;
    public PathChain ThirdRowToShoot;


    Follower follower;
    private Pose p_Start = new Pose(64.000, 9.000, Math.toRadians(-90));
    private Pose p_FirstRowEntry = new Pose(43.928, 35.870, Math.toRadians(180));
    private Pose p_FirstRowIntake = new Pose(11.957, 36.130, Math.toRadians(180));
    private Pose p_SecondRowEntry = new Pose(45.747, 59.783, Math.toRadians(180));
    private Pose p_SecondRowIntake = new Pose(11.177, 58.224, Math.toRadians(180));
    private Pose p_ThirdRowEntry = new Pose(43.668, 84.217, Math.toRadians(180));
    private Pose p_ThirdRowIntake = new Pose(17.675, 84.217, Math.toRadians(180));
    private Pose p_Shoot = new Pose(63.682, 12.477, Math.toRadians(90));

    public FarSidePaths(Follower follower, SIDES side) {
        if (side == SIDES.RED) {
            p_Start = p_Start.mirror();
            p_FirstRowEntry = p_FirstRowEntry.mirror();
            p_FirstRowIntake = p_FirstRowIntake.mirror();
            p_SecondRowEntry = p_SecondRowEntry.mirror();
            p_SecondRowIntake = p_SecondRowIntake.mirror();
            p_ThirdRowEntry = p_ThirdRowEntry.mirror();
            p_ThirdRowIntake = p_ThirdRowIntake.mirror();
            p_Shoot = p_Shoot.mirror();
        }
        this.follower = follower;
        StartToFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(p_Start, p_FirstRowEntry))
                .setLinearHeadingInterpolation(p_Start.getHeading(), p_FirstRowEntry.getHeading())
                .build();

        IntakeFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(p_FirstRowEntry, p_FirstRowIntake))
                .setTangentHeadingInterpolation()
                .build();

        FirstRowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(p_FirstRowIntake, p_Shoot))
                .setLinearHeadingInterpolation(p_FirstRowIntake.getHeading(), p_Shoot.getHeading())
                .build();

        ShootToSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(p_Shoot, p_SecondRowEntry))
                .setLinearHeadingInterpolation(p_Shoot.getHeading(), p_SecondRowEntry.getHeading())
                .build();

        IntakeSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(p_SecondRowEntry, p_SecondRowIntake))
                .setTangentHeadingInterpolation()
                .build();

        SecondRowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(p_SecondRowIntake, p_Shoot))
                .setLinearHeadingInterpolation(p_SecondRowIntake.getHeading(), p_Shoot.getHeading())
                .build();

        ShootToThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(p_Shoot, p_ThirdRowEntry))
                .setLinearHeadingInterpolation(p_Shoot.getHeading(), p_ThirdRowEntry.getHeading())
                .build();

        IntakeThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(p_ThirdRowEntry, p_ThirdRowIntake))
                .setTangentHeadingInterpolation()
                .build();

        ThirdRowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(p_ThirdRowIntake, p_Shoot))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }





}