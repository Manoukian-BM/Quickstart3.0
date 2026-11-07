package org.firstinspires.ftc.teamcode.pedro.subsystems;

/** Medidas reais de 30/10/2026 no referencial de pose ja usado pelo TeleOpBlue. */
public final class BlueHiveField {
    private BlueHiveField() { }

    // Robo encostado na lateral esquerda e no fundo, intake voltado para +Y.
    // A pose de reset e do centro do chassi, nao do contato com a parede.
    public static final double RESET_X_IN = 9.1;
    public static final double RESET_Y_IN = 130.4;
    public static final double RESET_HEADING_RADIANS = Math.toRadians(90.0);
    private static final double CHASSIS_HALF_WIDTH_CM = 22.5;
    private static final double CHASSIS_HALF_LENGTH_CM = 22.5;

    // Centros das ABERTURAS das bolas; nao o pivo de inclinacao da colmeia.
    public static final double OPENINGS_FROM_LEFT_CM = 149.0;
    public static final double OPENINGS_FROM_RIGHT_CM = 211.0;
    public static final double FIELD_WIDTH_CM = OPENINGS_FROM_LEFT_CM + OPENINGS_FROM_RIGHT_CM;
    public static final double AUDIENCE_FROM_BACK_CM = 130.0;       // 38-41: parede amarela
    public static final double SCORING_FROM_OPPOSITE_CM = 130.0;   // 42-45: parede oposta
    public static final double FIELD_LENGTH_CM = 360.0;             // Entre faces internas

    public static final double LEFT_WALL_X_IN = RESET_X_IN - CHASSIS_HALF_WIDTH_CM / 2.54;
    public static final double RIGHT_WALL_X_IN = LEFT_WALL_X_IN + FIELD_WIDTH_CM / 2.54;
    public static final double BACK_WALL_Y_IN = RESET_Y_IN + CHASSIS_HALF_LENGTH_CM / 2.54;
    // A origem antiga nao e necessariamente a parede: manter o mesmo referencial.
    public static final double OPPOSITE_WALL_Y_IN = BACK_WALL_Y_IN - FIELD_LENGTH_CM / 2.54;
    public static final double CENTER_Y_IN = (BACK_WALL_Y_IN + OPPOSITE_WALL_Y_IN) / 2.0;

    public static final double BLUE_X_IN = LEFT_WALL_X_IN + OPENINGS_FROM_LEFT_CM / 2.54;
    public static final double AUDIENCE_Y_IN = BACK_WALL_Y_IN - AUDIENCE_FROM_BACK_CM / 2.54;
    public static final double SCORING_Y_IN = OPPOSITE_WALL_Y_IN + SCORING_FROM_OPPOSITE_CM / 2.54;
}