package org.firstinspires.ftc.teamcode.core.Navigation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Vector2d {
    public double angleRadians;
    public double magnitude;
    public Pose2D p1;
    public Pose2D p2;
    public double deltaX;
    public double deltaY;
    public Vector2d(double angle, double mag){
        angleRadians = angle;
        magnitude = mag;

        this.p1 = new Pose2D(DistanceUnit.CM, 0,0, AngleUnit.RADIANS, 0);
        this.p2 = new Pose2D(DistanceUnit.CM, mag*Math.cos(angle),mag*Math.sin(angle), AngleUnit.RADIANS, 0);
        deltaX = this.p2.getX(DistanceUnit.CM)-this.p1.getX(DistanceUnit.CM);
        deltaY = this.p2.getY(DistanceUnit.CM)-this.p1.getY(DistanceUnit.CM);
    }
    public Vector2d(Pose2D p1, Pose2D p2){
        this.p1 = p1;
        this.p2 = p2;
        this.angleRadians = Math.atan2(p2.getX(DistanceUnit.CM)- p1.getX(DistanceUnit.CM), p2.getY(DistanceUnit.CM)- p1.getY(DistanceUnit.CM));
        this.magnitude =
                Math.sqrt(Math.pow(p2.getX(DistanceUnit.CM)-p1.getX(DistanceUnit.CM), 2)+ Math.pow(p2.getY(DistanceUnit.CM)-p1.getY(DistanceUnit.CM), 2));
    }
    public Vector2d MultipliedVector(double v){
        return new Vector2d(angleRadians, magnitude*v);
    }
}
