package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("roda0");
        c.frontRightName.set("roda1");
        c.backLeftName.set("roda2");
        c.backRightName.set("roda3");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.8178636445773868);
        c.yPodOffset.set(-2.862377617302842);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.37694041349054375);
                Controller secondaryTranslationalForward = Controller.proportional(0.13926941075140814);
                Controller primaryTranslationalLateral = Controller.proportional(0.649761904619428);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2400696617990275);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016352662044872138));
                c.brake.set(Controller.proportionalFeedforward(0.013899762738141317));

                c.headingFeedback.set(Controller.proportional(6.754992156898448));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06809251219856517, 0.003928548100050323));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11454046197126017, 0.09218402353385206));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014229074903394855, 0.0011513886973784488));

                c.maxAchievableForwardVelocity.set(62.725856332639395);
                c.maxAchievableStrafeVelocity.set(44.33180956652571);
                c.naturalForwardDeceleration.set(29.544716194239136);
                c.naturalStrafeDeceleration.set(71.52600040656567);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h , localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}