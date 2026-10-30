package org.firstinspires.ftc.teamcode.pedro.subsystems;

/** PID em RPM com feedforward; ganhos iniciais exigem calibracao no mecanismo real. */
public final class ShooterRpmController {
    public static final double KP = 0.00035;
    public static final double KI = 0.00012;
    public static final double KD = 0.000008;
    public static final double KV = 0.00030; // potencia/RPM a 12 V, valor inicial
    private static final double I_ZONE_RPM = 300;
    private static final double MAX_I_POWER = 0.12;
    private static final double DERIVATIVE_FILTER_SECONDS = 0.04;
    private double integralPower, previousRpm, filteredDerivative;
    private long previousTime;
    private boolean initialized;

    public double update(double target, double measuredRpm, double voltage, long now) {
        if (!Double.isFinite(target) || target <= 0 || !Double.isFinite(measuredRpm)) {
            reset();
            return 0;
        }
        double dt = initialized ? (now - previousTime) / 1e9 : 0;
        // Nao integrar nem diferenciar atraves de uma pausa no loop.
        if (dt <= 0 || dt > 0.1) {
            integralPower = 0;
            filteredDerivative = 0;
            dt = 0;
        } else {
            double derivative = (measuredRpm - previousRpm) / dt;
            double alpha = dt / (DERIVATIVE_FILTER_SECONDS + dt);
            filteredDerivative += alpha * (derivative - filteredDerivative);
        }
        initialized = true;
        previousTime = now;
        previousRpm = measuredRpm;
        double error = target - measuredRpm;
        if (Math.abs(error) > I_ZONE_RPM) integralPower = 0;
        double candidateI = clamp(integralPower + KI * error * dt, -MAX_I_POWER, MAX_I_POWER);
        double battery = Double.isFinite(voltage) && voltage >= 9 && voltage <= 15 ? voltage : 12;
        double compensation = 12 / battery;
        // Derivada da medicao evita um pico quando o alvo muda pela odometria.
        double base = KV * target + KP * error - KD * filteredDerivative;
        double candidateOutput = (base + candidateI) * compensation;
        if (Math.abs(error) <= I_ZONE_RPM
                && ((candidateOutput >= 0 && candidateOutput <= 1)
                    || (candidateOutput > 1 && error < 0)
                    || (candidateOutput < 0 && error > 0))) integralPower = candidateI;
        return clamp((base + integralPower) * compensation, 0, 1);
    }

    public void reset() {
        integralPower = 0;
        filteredDerivative = 0;
        initialized = false;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}