package org.firstinspires.ftc.teamcode.pedro.subsystems;

/** Centro horizontal do conjunto azul completo, sem misturar quadros ou aberturas. */
public final class BlueHiveTagCenter {
    // Centros oficiais da faixa, em polegadas; a inversao da ordem nao altera X=0.
    private static final double[] TAG_X = {-6.5, -2.75, 2.75, 6.5};
    private BlueHiveTagCenter() { }

    public static final class Detection {
        public final int id;
        public final double tx, ty;
        public Detection(int id, double tx, double ty) {
            this.id=id; this.tx=tx; this.ty=ty;
        }
    }

    /** Retorna NaN se faltar uma tag, houver repeticao ou geometria inconsistente. */
    public static double horizontalDegrees(Detection[] detections, int firstId) {
        if (detections==null || (firstId!=38 && firstId!=42)) return Double.NaN;
        double[] u=new double[4];
        boolean[] seen=new boolean[4];
        int count=0;
        for (Detection d : detections) {
            if (d==null || d.id<firstId || d.id>firstId+3) continue;
            int i=d.id-firstId;
            if (seen[i] || !Double.isFinite(d.tx) || !Double.isFinite(d.ty)
                    || Math.abs(d.tx)>=80 || Math.abs(d.ty)>=80) return Double.NaN;
            seen[i]=true; count++;
            u[i]=Math.tan(Math.toRadians(d.tx));
        }
        if (count!=4) return Double.NaN;
        double sign=Math.signum(u[3]-u[0]);
        if (Math.abs(Math.toDegrees(Math.atan(u[3]))-Math.toDegrees(Math.atan(u[0])))<1)
            return Double.NaN;
        for (int i=1;i<4;i++) if ((u[i]-u[i-1])*sign<=0) return Double.NaN;

        // Projecao de uma reta: u(X)=(a*X+b)/(c*X+1).
        // Ajustar o centro fisico evita a media de imagem deslocada pela perspectiva.
        double[][] m=new double[3][4];
        for (int i=0;i<4;i++) {
            double[] row={TAG_X[i],1,-u[i]*TAG_X[i]};
            for (int j=0;j<3;j++) {
                for (int k=0;k<3;k++) m[j][k]+=row[j]*row[k];
                m[j][3]+=row[j]*u[i];
            }
        }
        for (int col=0;col<3;col++) {
            int pivot=col;
            for (int r=col+1;r<3;r++) if (Math.abs(m[r][col])>Math.abs(m[pivot][col])) pivot=r;
            double[] tmp=m[col]; m[col]=m[pivot]; m[pivot]=tmp;
            if (Math.abs(m[col][col])<1e-10) return Double.NaN;
            double div=m[col][col];
            for (int k=col;k<4;k++) m[col][k]/=div;
            for (int r=0;r<3;r++) if (r!=col) {
                double factor=m[r][col];
                for (int k=col;k<4;k++) m[r][k]-=factor*m[col][k];
            }
        }
        double a=m[0][3], b=m[1][3], c=m[2][3];
        for (int i=0;i<4;i++) {
            double denominator=1+c*TAG_X[i];
            if (denominator<0.25) return Double.NaN;
            double fitted=Math.toDegrees(Math.atan((a*TAG_X[i]+b)/denominator));
            if (Math.abs(fitted-Math.toDegrees(Math.atan(u[i])))>0.3) return Double.NaN;
        }
        double center=Math.toDegrees(Math.atan(b));
        double lo=Math.min(Math.toDegrees(Math.atan(u[0])),Math.toDegrees(Math.atan(u[3])));
        double hi=Math.max(Math.toDegrees(Math.atan(u[0])),Math.toDegrees(Math.atan(u[3])));
        return Double.isFinite(center) && center>lo && center<hi ? center : Double.NaN;
    }
}