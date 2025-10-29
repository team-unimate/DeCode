package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.IntakeDexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.IntakeDexer;

import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.Sensoring.ArtifactSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
public class Teleop1 extends LinearOpMode {


    @Override

    public void runOpMode(){

        MainRobot mainRobot = new MainRobot();
        mainRobot.Init(hardwareMap, telemetry);
        Pose startingPose = new Pose(17.5/2,17.5/2);


        waitForStart();

        mainRobot.SetGoal(TurretSubsystem.GOALS.BLUE_GOAL);
        mainRobot.intakeDexer.Reset();
        mainRobot.outtake.Reset();
        mainRobot.drivetrain.StartTeleOpDrive();
        mainRobot.drivetrain.SetPose(startingPose);
        mainRobot.SetPattern(Patterns.PGP);
        mainRobot.setShootingMode(IntakeDexer.SHOOTING_MODES.PATTERN);

        while (!isStopRequested()){
            if (gamepad1.bWasPressed()){
                mainRobot.SetGoal(TurretSubsystem.GOALS.MOTIF);
            } else if (gamepad1.yWasPressed()){
                mainRobot.SetGoal(TurretSubsystem.GOALS.BLUE_GOAL);
            }
            mainRobot.drivetrain.DriveTeleOp(gamepad1);
            if (gamepad1.aWasPressed()){
                mainRobot.ForceShoot();
            }
            if (gamepad1.leftBumperWasPressed()){
                mainRobot.intakeDexer.IndexBallFromSlot(1);

            } else if (gamepad1.dpadUpWasPressed()){
                mainRobot.intakeDexer.IndexBallFromSlot(2);

            } else if (gamepad1.rightBumperWasPressed()){
                mainRobot.intakeDexer.IndexBallFromSlot(3);

            }

            mainRobot.Periodic(telemetry);

        }
    }
}
