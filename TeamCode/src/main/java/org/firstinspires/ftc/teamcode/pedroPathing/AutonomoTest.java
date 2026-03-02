package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

@Autonomous
@Configurable
public class AutonomoTest extends OpMode {

    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // CAMERA
    DcMotor cameraMotor;
    Limelight3A limelight;
    final int TARGET_ID = 20;
    final double kP = 0.02;
    final double kD = 0.004;
    final double MAX_POWER = 0.6;

    double ultimoErro = 0;
    double lastTx = 0;
    long lastSeenTime = 0;
    boolean scanRight = true;

    // MECANISMOS
    DcMotor intake, intake2, launcher;
    DistanceSensor sensor;
    VoltageSensor batteryVoltage;

    ElapsedTime paradaTimer = new ElapsedTime();

    boolean atirando = false;
    boolean liberarArtefato = false;

    double DISTANCIA_DETECCAO = 5.0;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.500, 124.500, Math.toRadians(143)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        sensor = hardwareMap.get(DistanceSensor.class, "distance");
        cameraMotor = hardwareMap.get(DcMotor.class, "cameraMotor");

        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        // launcher sempre ligado
        launcher.setPower(compensar(1.0));
    }

    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void loop() {
        follower.update();
        updateCamera();
        autonomousPathUpdate();
        updateMecanismos();
    }

    // ================= CAMERA =================

    public void updateCamera() {

        double rotCamera = 0;
        double txDegrees = 0;
        boolean viuTag = false;
        long now = System.currentTimeMillis();

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        if (fr.getFiducialId() == TARGET_ID) {
                            txDegrees = fr.getTargetXDegrees();
                            viuTag = true;
                            lastTx = txDegrees;
                            lastSeenTime = now;
                            break;
                        }
                    }
                }
            }
        }

        double giroCamera;

        if (viuTag) {
            double erro = txDegrees;
            double derivada = erro - ultimoErro;
            ultimoErro = erro;
            giroCamera = (erro * kP) + (derivada * kD);
            if (Math.abs(erro) < 0.4) giroCamera = 0;
        } else {
            long tempoPerdido = now - lastSeenTime;
            if (tempoPerdido < 700) giroCamera = (lastTx * kP * 0.5);
            else if (tempoPerdido < 1800) giroCamera = 0;
            else {
                double scanPower = 0.12;
                giroCamera = scanRight ? scanPower : -scanPower;
                if ((now / 1000) % 3 == 0) scanRight = !scanRight;
            }
        }

        giroCamera = Math.max(-MAX_POWER, Math.min(MAX_POWER, giroCamera));
        cameraMotor.setPower(compensar(giroCamera));
    }

    // ================= MECANISMOS =================

    public void iniciarDisparo() {
        atirando = true;
        liberarArtefato = true;
    }

    public void pararDisparo() {
        atirando = false;
        liberarArtefato = false;
    }

    public void updateMecanismos() {

        double distancia = sensor.getDistance(
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);

        boolean temObjeto = distancia < DISTANCIA_DETECCAO;

        intake.setPower(compensar(atirando ? 0.7 : 0.6));

        if (temObjeto && !liberarArtefato)
            intake2.setPower(0);
        else
            intake2.setPower(compensar(atirando ? 0.7 : 0.6));
    }

    // ================= AUTONOMO =================

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    iniciarDisparo();
                    paradaTimer.reset();
                    pathState = 2;
                }
                break;
            case 2:
                if (paradaTimer.seconds() >= 3) {
                    pararDisparo();
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    iniciarDisparo();
                    paradaTimer.reset();
                    pathState = 6;
                }
                break;
            case 6:
                if (paradaTimer.seconds() >= 3) {
                    pararDisparo();
                    follower.followPath(paths.Path5);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    iniciarDisparo();
                    paradaTimer.reset();
                    pathState = 12;
                }
                break;
            case 12:
                if (paradaTimer.seconds() >= 3) {
                    pararDisparo();
                    follower.followPath(paths.Path10);
                    pathState = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11);
                    pathState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path12);
                    pathState = 15;
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    iniciarDisparo();
                    paradaTimer.reset();
                    pathState = 16;
                }
                break;
            case 16:
                if (paradaTimer.seconds() >= 3) {
                    pararDisparo();
                    pathState = 17;
                }
                break;

            case 17:
                pararDisparo();
                break;
        }
    }


    // ================= PATHS =================

    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5, Path6,
                Path7, Path8, Path9, Path10, Path11, Path12;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.5, 124.5), new Pose(51.5, 91)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143)).build();

            Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(51.5, 91), new Pose(46, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(46, 84), new Pose(22.6, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(22.6, 84), new Pose(51.5, 91)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143)).build();

            Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(51.5, 91), new Pose(60, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180)).build();

            Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 60), new Pose(22.7, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path7 = follower.pathBuilder().addPath(new BezierCurve(new Pose(22.7, 60), new Pose(30, 60), new Pose(30, 70)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();

            Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(30, 70), new Pose(18, 70)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90)).build();

            Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(15, 70), new Pose(51.4, 91)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(143)).build();

            Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(51.4, 91), new Pose(60, 37.5)))
                    .setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(180)).build();

            Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 37.5), new Pose(22.7, 35.5)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(22.7, 35.5), new Pose(51.5, 91)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144)).build();
        }
    }
}