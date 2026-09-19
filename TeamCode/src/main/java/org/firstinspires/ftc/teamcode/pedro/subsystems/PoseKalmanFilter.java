package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.math.Pose;

public class PoseKalmanFilter {
    private double x, y, heading;

    // Covariâncias de erro de estimativa (Incerteza acumulada)
    private double pX = 1.0, pY = 1.0, pHeading = 1.0;

    // Ruídos do Processo (por ciclo da Odometria)
    private final double qModelPos = 0.04;
    private final double qModelHeading = 0.008;

    // Ruídos de Medição da Limelight (Confiabilidade visual)
    private final double rVisionPos = 0.15;
    private final double rVisionHeading = 0.04;

    // construtor que inicializa o filtro de Kalman com a pose inicial do robô
    public PoseKalmanFilter(Pose initialPose) {
        this.x = initialPose.x();
        this.y = initialPose.y();
        this.heading = Math.toDegrees(initialPose.heading());
    }

    public void predict(double deltaX, double deltaY, double deltaHeading) {
        this.x += deltaX;
        this.y += deltaY;
        this.heading = Math.toDegrees(normalizeAngle(Math.toRadians(this.heading + deltaHeading)));

        pX += qModelPos;
        pY += qModelPos;
        pHeading += qModelHeading;
    }

    // executado estritamente quando a Limelight valida uma AprilTag na HIVE
    public void updateWithVision(Pose visionPose) {
        // Correção de X
        double kX = pX / (pX + rVisionPos);
        this.x = this.x + kX * (visionPose.x() - this.x);
        pX = (1 - kX) * pX;

        // Correção de Y
        double kY = pY / (pY + rVisionPos);
        this.y = this.y + kY * (visionPose.y() - this.y);
        pY = (1 - kY) * pY;

        // Correção do Ângulo
        double visionHeadingDeg = Math.toDegrees(visionPose.heading());
        double headingError = visionHeadingDeg - this.heading;

        if (headingError > 180) headingError -= 360;
        else if (headingError < -180) headingError += 360;

        double kHeading = pHeading / (pHeading + rVisionHeading);
        this.heading = this.heading + kHeading * headingError;
        this.heading = Math.toDegrees(normalizeAngle(Math.toRadians(this.heading)));
        pHeading = (1 - kHeading) * pHeading;
    }

    public Pose getEstimatedPose() {
        return new Pose(x, y, Math.toRadians(heading));
    }

    private double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
