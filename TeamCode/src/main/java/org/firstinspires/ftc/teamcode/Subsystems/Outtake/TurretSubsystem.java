package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.controller.PIDFController;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class TurretSubsystem extends Subsystem {
    private CRServo servoT1;
    private CRServo servoT2;
    private DcMotor encoder;
    private Telemetry telemetry;
    private IMU imu;
    private int setPoint = 0;

    private static final int TICKS_PER_REV = 8192;
    private static final double GEAR_RATIO = 170.0 / 50.0;
    private static final double TOTAL_TICKS_PER_REV = TICKS_PER_REV * GEAR_RATIO;

    // 🔸 Distância da turret até o centro do robô (em polegadas)
    private double turretOffsetInches = 3; // ajuste conforme seu robô

    Pose poseBot = new Pose(0,0,0);
    PIDFController pidf = new PIDFController(
            TurretConstants.kP,
            TurretConstants.kI,
            TurretConstants.kD,
            TurretConstants.kF
    );

    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        imu = hardwareMap.get(IMU.class, TurretConstants.HMIMU);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        servoT1 = hardwareMap.get(CRServo.class, TurretConstants.HMServoTurret1);
        servoT2 = hardwareMap.get(CRServo.class, TurretConstants.HMServoTurret2);
        encoder = hardwareMap.get(DcMotor.class, TurretConstants.HMEncoder);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public enum GOALS {
        RED_GOAL,
        BLUE_GOAL,
        MOTIF
    }

    Pose poseRedGoal = new Pose(155, 155, 0);
    Pose poseBlueGoal = new Pose(0, 135, 0);
    double distanceToGoal = 0;

    public void AimTo(GOALS goal, Pose poseBot) {
        this.poseBot = poseBot;
        Pose target;
        switch (goal) {
            case RED_GOAL:
                target = poseRedGoal;
                break;
            case BLUE_GOAL:
                target = poseBlueGoal;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + goal);
        }

        // 🔸 Calcula a pose real da turret (não do centro do robô)
        Pose turretPose = getTurretPose(poseBot);

        double deltax = target.getX() - turretPose.getX();
        double deltay = target.getY() - turretPose.getY();

        distanceToGoal = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));
        double angle = Math.toDegrees(Math.atan2(deltay, deltax));

    }

    // 🔸 Retorna a pose da turret baseada na pose do robô e no deslocamento em polegadas
    private Pose getTurretPose(Pose botPose) {
        double heading = botPose.getHeading(); // radianos
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

        double delta = targetAngle - currentAngle;
        delta = AngleUnit.normalizeDegrees(delta);

        if (Math.abs(delta) > 90) {
            if (delta > 0) delta -= 360;
            else delta += 360;
        }

        double safeTarget = AngleUnit.normalizeDegrees(currentAngle + delta);
        safeTarget = Math.max(-185, Math.min(185, safeTarget));

        return safeTarget;
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

    // angulo 0 = angulo virado pra frente na arena
    public void SetAngleFC(double desiredDegrees) {
        double robotHeading = Math.toDegrees(poseBot.getHeading());
        double turretTargetRel = desiredDegrees - robotHeading + 180;
        turretTargetRel = AngleUnit.normalizeDegrees(turretTargetRel);
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
