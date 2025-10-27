package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class Field {

    private Square robotSquare;
    private final List<Triangle> obstacles = new ArrayList<>();

    public Field() {
        robotSquare = new Square(2, 2); // 18x18 inches
    }

    public void addObstacle(Triangle triangle) {
        obstacles.add(triangle);
    }

    public void updateRobotPose(Pose pose) {
        // se o campo tem 144x144 inches
        double adjustedX = 144 - pose.getX();
        double adjustedY = pose.getY();
        robotSquare.updatePose(new Pose(adjustedX, adjustedY, pose.getHeading()));
    }

    public boolean checkCollision() {
        for (Triangle triangle : obstacles) {
            if (PolygonCollision.isColliding(robotSquare.getVertices(), triangle.getVertices())) {
                return true;
            }
        }
        return false;
    }

    // ---- Inner Classes ----

    public static class Square {
        private double width, height;
        private Pose pose;
        private double[][] vertices = new double[4][2]; // { {x,y}, ... }

        public Square(double width, double height) {
            this.width = width;
            this.height = height;
        }

        public void updatePose(Pose pose) {
            this.pose = pose;
            updateVertices();
        }

        private void updateVertices() {
            double hw = width / 2.0;
            double hh = height / 2.0;
            double heading = Math.toRadians(pose.getHeading());

            double[][] localPoints = {
                    {-hw, -hh},
                    { hw, -hh},
                    { hw,  hh},
                    {-hw,  hh}
            };

            for (int i = 0; i < 4; i++) {
                double lx = localPoints[i][0];
                double ly = localPoints[i][1];
                double rx = lx * Math.cos(heading) - ly * Math.sin(heading);
                double ry = lx * Math.sin(heading) + ly * Math.cos(heading);
                vertices[i][0] = pose.getX() + rx;
                vertices[i][1] = pose.getY() + ry;
            }
        }

        public double[][] getVertices() {
            return vertices;
        }
    }

    public static class Triangle {
        private final double[][] vertices = new double[3][2];

        public Triangle(double x1, double y1, double x2, double y2, double x3, double y3) {
            vertices[0][0] = x1; vertices[0][1] = y1;
            vertices[1][0] = x2; vertices[1][1] = y2;
            vertices[2][0] = x3; vertices[2][1] = y3;
        }

        public double[][] getVertices() {
            return vertices;
        }
    }

    // 🔥 Algoritmo de colisão por Separating Axis Theorem (SAT) simples
    public static class PolygonCollision {

        public static boolean isColliding(double[][] polyA, double[][] polyB) {
            return !hasSeparatingAxis(polyA, polyB) && !hasSeparatingAxis(polyB, polyA);
        }

        private static boolean hasSeparatingAxis(double[][] polyA, double[][] polyB) {
            int countA = polyA.length;
            for (int i = 0; i < countA; i++) {
                int next = (i + 1) % countA;
                double edgeX = polyA[next][0] - polyA[i][0];
                double edgeY = polyA[next][1] - polyA[i][1];

                double axisX = -edgeY;
                double axisY = edgeX;

                double[] projectionA = projectPolygon(axisX, axisY, polyA);
                double[] projectionB = projectPolygon(axisX, axisY, polyB);

                if (projectionA[1] < projectionB[0] || projectionB[1] < projectionA[0]) {
                    return true;
                }
            }
            return false;
        }

        private static double[] projectPolygon(double axisX, double axisY, double[][] poly) {
            double min = Double.MAX_VALUE, max = -Double.MIN_VALUE;
            for (double[] point : poly) {
                double proj = (point[0] * axisX + point[1] * axisY) /
                        Math.sqrt(axisX * axisX + axisY * axisY);
                min = Math.min(min, proj);
                max = Math.max(max, proj);
            }
            return new double[]{min, max};
        }
    }
}
