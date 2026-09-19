package org.firstinspires.ftc.teamcode.pedro.subsystems;

public class turretKalmanFilter {
    private double estimatedError = 0; // O ângulo limpo para a HIVE
    private double pAngle = 1.0;       // Incerteza do filtro

    private final double qProcess = 0.1;   // O quanto o movimento do robô suja a leitura por loop
    private final double rVision = 0.05;   // Ruído de leitura da Limelight (oscilação óptica)


    // Passo de Predição: Desconta o quanto o robô girou fisicamente usando os dados do PP

    public void predict(double robotDeltaHeadingDeg) {
        // Se o robô girou para a direita, o alvo foi para a esquerda no campo de visão da turret
        this.estimatedError -= robotDeltaHeadingDeg;
        pAngle += qProcess; // Aumenta a incerteza do filtro
    }

    // Passo de Atualização: Roda quando a Limelight consegue ver a HIVE (independente de onde ela se moveu)

    public void update(double limelightTx) {
        double kGain = pAngle / (pAngle + rVision);
        this.estimatedError = this.estimatedError + kGain * (limelightTx - this.estimatedError);
        pAngle = (1 - kGain) * pAngle;
    }

    public double getCleanAngleError() {
        return estimatedError;
    }
}
