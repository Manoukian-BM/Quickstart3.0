package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.math.Pose;

public class PoseKalmanFilter {
    private double x, y, heading;

    private double pX = 1.0;
    private double pY = 1.0;
    private double pHeading = 1.0;

    private final double qModelPos = 0.04;
    private final double qModelHeading = 0.008;

    private final double rVisionPos = 0.15;
    private final double rVisionHeading = 0.04;

    public PoseKalmanFilter(Pose initialPose) {
        this.x = initialPose.x();
        this.y = initialPose.y();

        // Pose já fornece o heading em radianos
        this.heading = normalizeAngle(initialPose.heading());
    }

    public void predict(
            double deltaX,
            double deltaY,
            double deltaHeading) {

        this.x += deltaX;
        this.y += deltaY;

        // deltaHeading também precisa estar em radianos
        this.heading = normalizeAngle(this.heading + deltaHeading);

        this.pX += qModelPos;
        this.pY += qModelPos;
        this.pHeading += qModelHeading;
    }

    public void updateWithVision(Pose visionPose) {
        double kX = pX / (pX + rVisionPos);
        this.x += kX * (visionPose.x() - this.x);
        pX = (1.0 - kX) * pX;

        double kY = pY / (pY + rVisionPos);
        this.y += kY * (visionPose.y() - this.y);
        pY = (1.0 - kY) * pY;

        double headingError =
                normalizeAngle(visionPose.heading() - this.heading);

        double kHeading =
                pHeading / (pHeading + rVisionHeading);

        this.heading = normalizeAngle(
                this.heading + kHeading * headingError
        );

        pHeading = (1.0 - kHeading) * pHeading;
    }

    public Pose getEstimatedPose() {
        // Pose recebe heading em radianos
        return new Pose(x, y, heading);
    }

    private double normalizeAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }

        while (radians <= -Math.PI) {
            radians += 2.0 * Math.PI;
        }

        return radians;
    }
}
