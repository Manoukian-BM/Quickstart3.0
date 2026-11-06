package org.firstinspires.ftc.teamcode.pedro.subsystems;

/** Geometria em polegadas, heading anti-horario em radianos; sem dependencias de hardware. */
public final class TurretGeometry {
    public static final double FORWARD_OFFSET_MM = -90.0;
    public static final double LEFT_OFFSET_MM = 0.0;
    private static final double AUDIENCE_LEFT_REDUCTION_DEGREES = 12.0;
    private static final double LATERAL_TRANSITION_IN = 6.0;
    /** Aplica a calibracao lateral da abertura selecionada, mesmo junto a linha central. */
    public static double fieldCorrectionRadians(double robotX, boolean audienceOpening) {
        if (!Double.isFinite(robotX)) return Double.NaN;
        if (!audienceOpening) return Math.toRadians(-2.5);
        double base = Math.toRadians(2.5);
        double factor=Math.max(0.0,Math.min(1.0,
                (BlueHiveField.BLUE_X_IN-robotX)/LATERAL_TRANSITION_IN));
        return base-Math.toRadians(AUDIENCE_LEFT_REDUCTION_DEGREES)*factor;
    }
    private TurretGeometry() { }
    public static final class Aim {
        public final double turretX, turretY, distance, globalAngle, relativeAngle;
        public final boolean valid;
        private Aim(double x, double y, double distance, double global, double relative, boolean valid) {
            turretX=x; turretY=y; this.distance=distance;
            globalAngle=global; relativeAngle=relative; this.valid=valid;
        }
    }
    public static Aim calculate(double x, double y, double heading, double targetX, double targetY) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)
                || !Double.isFinite(targetX) || !Double.isFinite(targetY))
            return new Aim(Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN,false);
        double forward=FORWARD_OFFSET_MM/25.4, left=LEFT_OFFSET_MM/25.4;
        double tx=x+forward*Math.cos(heading)-left*Math.sin(heading);
        double ty=y+forward*Math.sin(heading)+left*Math.cos(heading);
        double dx=targetX-tx, dy=targetY-ty, distance=Math.hypot(dx,dy);
        double global=Math.atan2(dy,dx);
        return new Aim(tx,ty,distance,global,normalize(global-heading),distance>1e-6);
    }
    public static double normalize(double angle) {
        return Math.atan2(Math.sin(angle),Math.cos(angle));
    }
    /** Mantem o mapeamento preexistente; nao representa calibracao mecanica medida. */
    public static double servoCommand(double relative, double center, double direction,
                                      double rangeDegrees, double trim, double min, double max) {
        double raw=center+direction*Math.toDegrees(relative)/rangeDegrees+trim;
        return Math.max(min,Math.min(max,raw));
    }
}
