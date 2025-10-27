package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Patterns;
import org.firstinspires.ftc.teamcode.core.controller.PIDFController;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

import java.util.List;

public class TurretSubsystem extends Subsystem {

    private static final int TICKS_PER_REV = 8192;
    private static final double GEAR_RATIO = 170.0 / 50.0;
    private static final double TOTAL_TICKS_PER_REV = TICKS_PER_REV * GEAR_RATIO;

    private CRServo servoT1;
    private CRServo servoT2;
    private DcMotor encoder;
    private Telemetry telemetry;
    private IMU imu;
    private Limelight3A limelight;

    private int setPoint = 0;
    private double turretOffsetInches = 3;
    private double distanceToGoal = 0;

    private Patterns pattern = null;
    private Pose poseBot = new Pose(0, 0, 0);

    private final PIDFController pidf = new PIDFController(
            TurretConstants.kP,
            TurretConstants.kI,
            TurretConstants.kD,
            TurretConstants.kF
    );

    private final Pose poseRedGoal = new Pose(155, 155, 0);
    private final Pose poseBlueGoal = new Pose(0, 135, 0);
    private final Pose poseMotif = new Pose(70, 140, 0);

    public enum GOALS {
        RED_GOAL,
        BLUE_GOAL,
        MOTIF
    }

    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, TurretConstants.HMIMU);
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);

        servoT1 = hardwareMap.get(CRServo.class, TurretConstants.HMServoTurret1);
        servoT2 = hardwareMap.get(CRServo.class, TurretConstants.HMServoTurret2);
        encoder = hardwareMap.get(DcMotor.class, TurretConstants.HMEncoder);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Patterns getPattern() {
        return pattern;
    }

    public void AimTo(GOALS goal, Pose poseBot) {
        this.poseBot = poseBot;

        double tx = 0;
        Pose target;

        switch (goal) {
            case RED_GOAL:
                tx = getTagError(24);
                target = poseRedGoal;
                break;

            case BLUE_GOAL:
                tx = getTagError(20);
                target = poseBlueGoal;
                break;

            case MOTIF:
                tx = getTagError(21);
                tx = getTagError(22);
                tx = getTagError(23);
                target = poseMotif;
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + goal);
        }

        Pose turretPose = getTurretPose(poseBot);
        double deltaX = target.getX() - turretPose.getX();
        double deltaY = target.getY() - turretPose.getY();

        distanceToGoal = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        SetAngleFC(angle + tx);
    }

    private Pose getTurretPose(Pose botPose) {
        double heading = botPose.getHeading();
        double turretX = botPose.getX() + turretOffsetInches * Math.cos(heading);
        double turretY = botPose.getY() + turretOffsetInches * Math.sin(heading);
        return new Pose(turretX, turretY, botPose.getHeading());
    }

    public double GetDistanceToGoal() {
        return distanceToGoal;
    }

    public void Reset() {
        imu.resetYaw();
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getAngle() {
        double ticks = encoder.getCurrentPosition();
        double degrees = (ticks / TOTAL_TICKS_PER_REV) * 360.0;
        return AngleUnit.normalizeDegrees(degrees);
    }

    private double getSafeTurretTarget(double currentAngle, double targetAngle) {
        currentAngle = AngleUnit.normalizeDegrees(currentAngle);
        targetAngle = AngleUnit.normalizeDegrees(targetAngle);

        double delta = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

        if (Math.abs(delta) > 90) {
            delta += (delta > 0) ? -360 : 360;
        }

        double safeTarget = AngleUnit.normalizeDegrees(currentAngle + delta);
        return Math.max(-185, Math.min(185, safeTarget));
    }

    public void SetSetPoint(int sp) {
        setPoint = sp;
    }

    public void SetAngle(double desiredDegrees) {
        double currentAngle = getAngle();
        double safeAngle = getSafeTurretTarget(currentAngle, desiredDegrees);
        int targetTicks = (int) ((safeAngle / 360.0) * TOTAL_TICKS_PER_REV);
        SetSetPoint(targetTicks);
    }

    public double getTagError(int tagID) {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        double txResult = 0.0;

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId()==21){
                    pattern = Patterns.GPP;
                } else if (fr.getFiducialId()==22){
                    pattern = Patterns.PGP;
                } else if (fr.getFiducialId()==23){
                    pattern = Patterns.PPG;
                }

            if (fr.getFiducialId() == tagID) {
                txResult = fr.getTargetXDegrees();
            }
        }

        return txResult;
    }

    public void SetAngleFC(double desiredDegrees) {
        double robotHeading = Math.toDegrees(poseBot.getHeading());
        double turretTargetRel = AngleUnit.normalizeDegrees(desiredDegrees - robotHeading + 180);
        SetAngle(-turretTargetRel);
    }

    @Override
    public void Periodic() {
        double currentPos = encoder.getCurrentPosition();
        pidf.setSetpoint(setPoint);
        double output = -pidf.calculate(currentPos);

        servoT1.setPower(output);
        servoT2.setPower(output);
    }

    public boolean stabilized() {
        pidf.setTolerance(TurretConstants.stabilizedTolerance);
        return pidf.atSetpoint(encoder.getCurrentPosition());
    }
}
