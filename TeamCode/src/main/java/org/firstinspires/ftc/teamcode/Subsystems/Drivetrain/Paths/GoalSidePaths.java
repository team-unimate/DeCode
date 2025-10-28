package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Paths.SIDES;

public class GoalSidePaths {
    public PathChain StartToSpin;
    public PathChain SpinToFirstRow;
    public PathChain IntakeFirstRow;
    public PathChain FirstRowToShoot;
    public PathChain ShootToSecondRow;
    public PathChain IntakeSecondRow;
    public PathChain SecondRowToShoot;
    public PathChain ShootToThirdRow;
    public PathChain IntakeThirdRow;
    public PathChain ThirdRowToShoot;
    public PathChain ShootToLaunch;
    public PathChain PushRobot;
    public PathChain Finish;



    public Pose p_Start = new Pose(35.090, 132.823, Math.toRadians(-90));
    public Pose p_Spin = new Pose(61.603, 86.296, Math.toRadians(-90));
    public Pose p_FirstRowEntry = new Pose(50, 80, Math.toRadians(180-5));
    public Pose p_FirstRowIntake = new Pose(20, 80, Math.toRadians(180-5));
    public Pose p_SecondRowEntry = new Pose(50, 57, Math.toRadians(180));
    public Pose p_SecondRowIntake = new Pose(17, 57, Math.toRadians(180));
    public Pose p_ThirdRowEntry = new Pose(50, 36.130, Math.toRadians(180));
    public Pose p_ThirdRowIntake = new Pose(17, 36.130, Math.toRadians(180));
    public Pose p_Launch = new Pose(64.722, 9.000, Math.toRadians(180));
    public Pose p_Push = new Pose(33.271, 9.877, Math.toRadians(180));
    public Pose p_End = new Pose(61.343, 53.285, Math.toRadians(180));

    public Follower follower;

    public GoalSidePaths(Follower follower, SIDES side) {
        this.follower = follower;

        if (side == SIDES.RED) {
            p_Start = p_Start.mirror();
            p_Spin = p_Spin.mirror();
            p_FirstRowEntry = p_FirstRowEntry.mirror();
            p_FirstRowIntake = p_FirstRowIntake.mirror();
            p_SecondRowEntry = p_SecondRowEntry.mirror();
            p_SecondRowIntake = p_SecondRowIntake.mirror();
            p_ThirdRowEntry = p_ThirdRowEntry.mirror();
            p_ThirdRowIntake = p_ThirdRowIntake.mirror();
            p_Launch = p_Launch.mirror();
            p_Push = p_Push.mirror();
            p_End = p_End.mirror();
        }

        StartToSpin = follower.pathBuilder()
                .addPath(new BezierLine(p_Start, p_Spin))
                .setLinearHeadingInterpolation(p_Start.getHeading(), p_Spin.getHeading())
                .setBrakingStrength(4)
                .build();

        SpinToFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(p_Spin, p_FirstRowEntry))
                .setLinearHeadingInterpolation(p_Spin.getHeading(), p_FirstRowEntry.getHeading())
                .build();

        IntakeFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(p_FirstRowEntry, p_FirstRowIntake))
                .setLinearHeadingInterpolation(p_FirstRowEntry.getHeading(), p_FirstRowIntake.getHeading())
                .build();

        FirstRowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(p_FirstRowIntake, p_Spin))
                .setLinearHeadingInterpolation(p_FirstRowIntake.getHeading(), p_Spin.getHeading())
                .build();

        ShootToSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(p_Spin, p_SecondRowEntry))
                .setLinearHeadingInterpolation(p_Spin.getHeading(), p_SecondRowEntry.getHeading())
                .build();

        IntakeSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(p_SecondRowEntry, p_SecondRowIntake))
                .setLinearHeadingInterpolation(p_SecondRowEntry.getHeading(), p_SecondRowIntake.getHeading())
                .build();

        SecondRowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(p_SecondRowIntake, p_Spin))
                .setLinearHeadingInterpolation(p_SecondRowIntake.getHeading(), p_Spin.getHeading())
                .build();

        ShootToThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(p_Spin, p_ThirdRowEntry))
                .setLinearHeadingInterpolation(p_Spin.getHeading(), p_ThirdRowEntry.getHeading())
                .build();

        IntakeThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(p_ThirdRowEntry, p_ThirdRowIntake))
                .setLinearHeadingInterpolation(p_ThirdRowEntry.getHeading(), p_ThirdRowIntake.getHeading())
                .build();

        ThirdRowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(p_ThirdRowIntake, p_Spin))
                .setLinearHeadingInterpolation(p_ThirdRowIntake.getHeading(), p_Spin.getHeading())
                .build();

        ShootToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(p_Spin, p_Launch))
                .setLinearHeadingInterpolation(p_Spin.getHeading(), p_Launch.getHeading())
                .build();

        PushRobot = follower.pathBuilder()
                .addPath(new BezierLine(p_Launch, p_Push))
                .setLinearHeadingInterpolation(p_Launch.getHeading(), p_Push.getHeading())
                .build();

        Finish = follower.pathBuilder()
                .addPath(new BezierLine(p_Push, p_End))
                .setTangentHeadingInterpolation()
                .build();
    }

}