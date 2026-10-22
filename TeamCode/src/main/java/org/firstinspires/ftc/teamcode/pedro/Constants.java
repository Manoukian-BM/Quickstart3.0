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
        c.xPodOffset.set(0.53186345288134);
        c.yPodOffset.set(-0.24842183420977257);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3068732447746335);
                Controller secondaryTranslationalForward = Controller.proportional(0.11338146414010876);
                Controller primaryTranslationalLateral = Controller.proportional(0.4814171940142222);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17787078948391682);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014493156014284029));
                c.brake.set(Controller.proportionalFeedforward(0.012319182612141424));

                c.headingFeedback.set(Controller.proportional(4.565107550745594));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06186257612710757, 0.002714922802971673));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09611082104065073, 0.07863995998662344));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015637363019944486, 0.0012405083934607772));

                c.maxAchievableForwardVelocity.set(68.11343064497149);
                c.maxAchievableStrafeVelocity.set(53.131956107667655);
                c.naturalForwardDeceleration.set(34.085794386513136);
                c.naturalStrafeDeceleration.set(72.41131469096291);
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