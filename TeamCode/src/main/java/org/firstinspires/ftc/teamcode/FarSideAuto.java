package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Paths.FarSidePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Paths.SIDES;

@Autonomous(name = "Far Side Auto", group = "Main")
public class FarSideAuto extends OpMode {
    private MainRobot mainRobot = new MainRobot();
    private FarSidePaths farSidePaths;
    private Follower follower;

    private enum AutoState {
        START_TO_FIRST_ROW,
        INTAKE_FIRST_ROW,
        FIRST_ROW_TO_SHOOT,
        SHOOT_TO_SECOND_ROW,
        INTAKE_SECOND_ROW,
        SECOND_ROW_TO_SHOOT,
        SHOOT_TO_THIRD_ROW,
        INTAKE_THIRD_ROW,
        THIRD_ROW_TO_SHOOT,
        DONE
    }

    private AutoState currentState = AutoState.START_TO_FIRST_ROW;

    @Override
    public void init() {
        mainRobot.Init(hardwareMap, telemetry);
        follower = mainRobot.drivetrain.getFollower();

        // Cria os caminhos para o lado BLUE
        farSidePaths = new FarSidePaths(follower, SIDES.BLUE);

        // Define a pose inicial
        follower.setPose(new Pose(64.000, 9.000, Math.toRadians(-90)));

        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(farSidePaths.StartToFirstRow);
    }

    @Override
    public void loop() {
        follower.update();
        mainRobot.Periodic(telemetry);

        switch (currentState) {

            case START_TO_FIRST_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.IntakeFirstRow);
                    currentState = AutoState.INTAKE_FIRST_ROW;
                }
                break;

            case INTAKE_FIRST_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.FirstRowToShoot);
                    currentState = AutoState.FIRST_ROW_TO_SHOOT;
                }
                break;

            case FIRST_ROW_TO_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.ShootToSecondRow);
                    currentState = AutoState.SHOOT_TO_SECOND_ROW;
                }
                break;

            case SHOOT_TO_SECOND_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.IntakeSecondRow);
                    currentState = AutoState.INTAKE_SECOND_ROW;
                }
                break;

            case INTAKE_SECOND_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.SecondRowToShoot);
                    currentState = AutoState.SECOND_ROW_TO_SHOOT;
                }
                break;

            case SECOND_ROW_TO_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.ShootToThirdRow);
                    currentState = AutoState.SHOOT_TO_THIRD_ROW;
                }
                break;

            case SHOOT_TO_THIRD_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.IntakeThirdRow);
                    currentState = AutoState.INTAKE_THIRD_ROW;
                }
                break;

            case INTAKE_THIRD_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(farSidePaths.ThirdRowToShoot);
                    currentState = AutoState.THIRD_ROW_TO_SHOOT;
                }
                break;

            case THIRD_ROW_TO_SHOOT:
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                telemetry.addLine("✅ Auto Complete!");
                break;
        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }
}
