package org.firstinspires.ftc.teamcode.pedro.subsystems;

/** Um pedido do piloto produz no maximo um ajuste; a odometria nunca e reescrita. */
public final class BlueHiveFineAdjustment {
    private static final long TIMEOUT_NS=2_500_000_000L;
    private static final long SETTLE_NS=250_000_000L;
    private static final double MAX_ADJUSTMENT_DEG=8.0;
    private final double[] offsets={0,0};
    private final boolean[] confirmed = new boolean[2];
    private int cell;
    private boolean active;
    private long started, settled, firstSample, lastSample;
    private double x,y,heading,servo,lastTimestamp,mean;
    private int samples;
    private int sampleTagId = -1;
    private String status="Segure LB do P2 para mirar. R1 solicita ajuste fino";

    public void request(int cell, long now, double x, double y, double heading, double servo) {
        if (cell<0 || cell>1 || !finite(x,y,heading,servo)) { cancel("Pose invalida"); return; }
        this.cell=cell; this.started=now; this.active=true; this.lastTimestamp=Double.NaN;
        resetSamples(now,x,y,heading,servo);
        status="Aguardando robo e torreta parados";
    }

    /** Chamado todo loop, inclusive sem imagem, para limitar tempo e detectar movimento. */
    public void track(long now, int cell, double x, double y, double heading, double servo) {
        if (!active) return;
        if (now-started>=TIMEOUT_NS) { cancel("Tempo esgotado; mantida odometria"); return; }
        if (cell!=this.cell || !finite(x,y,heading,servo)) { cancel("Alvo ou pose mudou; mantida odometria"); return; }
        if (Math.hypot(x-this.x,y-this.y)>0.25
                || Math.abs(TurretGeometry.normalize(heading-this.heading))>Math.toRadians(0.5)
                || Math.abs(servo-this.servo)*360>0.25) {
            resetSamples(now,x,y,heading,servo);
            status="Aguardando robo e torreta parados";
        }
    }

    /** Retorna true ao confirmar a correcao com imagens suficientes. */
    public boolean observe(long now, double timestamp, int tagId, double errorDegrees) {
        if (!active || now-settled<SETTLE_NS) return false;
        if (!Double.isFinite(timestamp) || timestamp<=0 || timestamp==lastTimestamp) return false;
        if (Double.isFinite(lastTimestamp) && timestamp<lastTimestamp) {
            lastTimestamp=timestamp; samples=0; return false;
        }
        lastTimestamp=timestamp;
        if (tagId != sampleTagId || now-lastSample>150_000_000L) samples=0;
        sampleTagId = tagId;
        lastSample=now;
        if (!Double.isFinite(errorDegrees)) { samples=0; status="Aguardando as duas tags do meio"; return false; }
        double candidate=offsets[cell]-errorDegrees;
        // Mapeamento atual da torreta: posicao=0.5-angulo/360.
        double nextServo=servo+errorDegrees/360.0;
        if (nextServo<0 || nextServo>1) {
            samples=0; status="Ajuste ultrapassa limite da torreta; reposicione o robo"; return false;
        }
        if (Math.abs(errorDegrees)>MAX_ADJUSTMENT_DEG || Math.abs(candidate)>MAX_ADJUSTMENT_DEG) {
            samples=0; status="Erro maior que 8 graus; confira pose ou ajuste manual"; return false;
        }
        if (samples==0 || Math.abs(candidate-mean)>0.5) {
            samples=1; mean=candidate; firstSample=now;
        } else { samples++; mean+=(candidate-mean)/samples; }
        status="Confirmando tags centrais " + tagId + " e " + (tagId + 1);
        if (samples>=4 && now-firstSample>=80_000_000L) {
            offsets[cell]=mean;
            confirmed[cell] = true;
            active=false;
            status="Ajuste visual aplicado";
            return true;
        }
        return false;
    }

    public void clear(int cell) {
        offsets[cell] = 0;
        confirmed[cell] = false;
        cancel("Ajuste zerado; odometria");
    }

    public void clearAll() {
        offsets[0] = offsets[1] = 0;
        confirmed[0] = confirmed[1] = false;
        cancel("Pose redefinida; ajustes visuais zerados");
    }
    public void cancel(String reason) { active=false; samples=0; status=reason; }
    public boolean isActive() { return active; }
    public boolean hasCorrection(int cell) { return confirmed[cell]; }
    public double degrees(int cell) { return offsets[cell]; }
    public String status() { return status; }
    private void resetSamples(long now,double x,double y,double heading,double servo) {
        this.x=x; this.y=y; this.heading=heading; this.servo=servo;
        settled=now; samples=0;
        sampleTagId = -1;
    }
    private static boolean finite(double a,double b,double c,double d) {
        return Double.isFinite(a)&&Double.isFinite(b)&&Double.isFinite(c)&&Double.isFinite(d);
    }
}