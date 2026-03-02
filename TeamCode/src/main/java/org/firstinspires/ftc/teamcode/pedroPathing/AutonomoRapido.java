package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Autonomous
@Configurable
public class AutonomoRapido extends OpMode {

    public Follower follower;
    private int pathState = 0;
    private Paths paths;

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

    DcMotor intake, intake2, launcher;
    DistanceSensor sensor;
    VoltageSensor batteryVoltage;

    ElapsedTime paradaTimer = new ElapsedTime();

    boolean atirando = false;
    boolean liberarArtefato = false;

    double TEMPO_DISPARO = 2.0;
    double TEMPO_ESPERA = 1.0;
    double VELOCIDADE_NORMAL = 1.0;
    double VELOCIDADE_COLETA = 0.75;
    boolean fazendoRecuo = false;
    ElapsedTime recuoTimer = new ElapsedTime();
    double SPEED_INTAKE_SLOW = 0.3;
    double SPEED_RECUO = -0.3;
    double TIMER_RECUO = 0.25;
    double distanciaTagCm = -1;
    double potenciaLauncherCalculada = 0.9;




    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(VELOCIDADE_NORMAL);
        follower.setStartingPose(new Pose(32.1,134.5,Math.toRadians(180)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class,"intake");
        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        launcher = hardwareMap.get(DcMotor.class,"launcher");
        sensor = hardwareMap.get(DistanceSensor.class,"distance");
        cameraMotor = hardwareMap.get(DcMotor.class,"cameraMotor");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try{
            limelight = hardwareMap.get(Limelight3A.class,"limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        }catch(Exception e){ limelight = null; }

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        launcher.setPower(compensar(0.95));
    }

    private double compensar(double power){
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1,Math.min(1,power*fator));
    }

    @Override
    public void loop(){
        follower.update();
        controlarVelocidade();
        updateCamera();
        autonomousPathUpdate();
        updateMecanismos();
        telemetryPose();
    }

    public void iniciarDisparo(){
        atirando=true;
        liberarArtefato=true;
    }

    public void pararDisparo(){
        atirando=false;
        liberarArtefato=false;
    }

    public void controlarVelocidade(){

        switch(pathState){

            // trechos onde coleta artefatos
            case 3:
            case 4:
            case 7:
            case 8:
            case 9:
            case 13:
                follower.setMaxPower(VELOCIDADE_COLETA);
                break;

            default:
                follower.setMaxPower(VELOCIDADE_NORMAL);
                break;
        }
    }

    public void updateMecanismos(){

        if(fazendoRecuo){
            intake2.setPower(compensar(SPEED_RECUO));
            intake.setPower(0);

            if(recuoTimer.seconds() >= TIMER_RECUO){
                fazendoRecuo = false;
                iniciarDisparo();
            }
            return;
        }

        if(atirando){
            intake2.setPower(compensar(0.95));
            intake.setPower(compensar(0.95));
        }else{
            intake.setPower(compensar(1.0));
            intake2.setPower(0);
            switch (pathState){
                case 3:
                case 4:
                case 7:
                case 8:
                case 13:
                    intake2.setPower(compensar(SPEED_INTAKE_SLOW));
                    intake.setPower(0);
                    break;

                default:
                    intake.setPower(compensar(0.72));
                    intake2.setPower(0);
                    break;
            }
        }
    }

    public void telemetryPose(){

        Pose pose = follower.getPose();

        telemetry.addLine("===== ODOMETRIA =====");
        telemetry.addData("X (cm)", "%.2f", pose.getX());
        telemetry.addData("Y (cm)", "%.2f", pose.getY());
        telemetry.addData("Heading (graus)", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Estado Auto", pathState);
        telemetry.update();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

// ===== PATH 1 =====
            case 1:
                if (!follower.isBusy()) {
                    fazendoRecuo = true;
                    recuoTimer.reset();
                    paradaTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (paradaTimer.seconds() >= TEMPO_DISPARO) {
                    pararDisparo();
                    paradaTimer.reset();
                    pathState = 20;
                }
                break;

            case 20:
                if (paradaTimer.seconds() >= TEMPO_ESPERA) {
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;


// ===== IR PARA PATH 4 =====
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


// ===== PATH 4 =====
            case 5:
                if (!follower.isBusy()) {
                    fazendoRecuo = true;
                    recuoTimer.reset();
                    paradaTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (paradaTimer.seconds() >= TEMPO_DISPARO) {
                    pararDisparo();
                    paradaTimer.reset();
                    pathState = 21;
                }
                break;

            case 21:
                if (paradaTimer.seconds() >= TEMPO_ESPERA) {
                    follower.followPath(paths.Path5);
                    pathState = 7;
                }
                break;


// ===== IR PARA PATH 7 =====
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


// ===== PATH 7 =====
            case 9:
                if (!follower.isBusy()) {
                    fazendoRecuo = true;
                    recuoTimer.reset();
                    paradaTimer.reset();
                    pathState = 30;
                }
                break;

            case 30:
                if (paradaTimer.seconds() >= TEMPO_DISPARO) {
                    pararDisparo();
                    paradaTimer.reset();
                    pathState = 31;
                }
                break;

            case 31:
                if (paradaTimer.seconds() >= TEMPO_ESPERA) {
                    follower.followPath(paths.Path8);
                    pathState = 24;
                }
                break;


// ===== IR PARA PATH 10 =====
            case 24:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10);
                    pathState = 11;
                }
                break;


// ===== PATH 10 =====
            case 11:
                if (!follower.isBusy()) {
                    fazendoRecuo = true;
                    recuoTimer.reset();
                    paradaTimer.reset();
                    pathState = 12;
                }
                break;

            case 12:
                if (paradaTimer.seconds() >= TEMPO_DISPARO) {
                    pararDisparo();
                    pathState = 99;   // FINALIZA AQUI
                }
                break;

            case 99:
                pararDisparo();
                break;
        }
    }

    public void updateCamera() {

        double txDegrees = 0;
        boolean viuTag = false;
        long now = System.currentTimeMillis();

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                // Correção: usar getBotpose() e depois getPosition() para extrair o z
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    // acessar o objeto de posição interno
                    org.firstinspires.ftc.robotcore.external.navigation.Position pos = botpose.getPosition();
                    if (pos != null) {
                        double zMetros = pos.z;
                        distanciaTagCm = zMetros * 100.0;

                        potenciaLauncherCalculada = 0.55 + (distanciaTagCm * 0.0035);
                        if (potenciaLauncherCalculada < 0.60) potenciaLauncherCalculada = 0.60;
                        if (potenciaLauncherCalculada > 0.95) potenciaLauncherCalculada = 0.95;
                    }
                }

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

        giroCamera = 1.35 * Math.max(-MAX_POWER, Math.min(MAX_POWER, giroCamera));
        cameraMotor.setPower(compensar(giroCamera));
    }

    public static class Paths{

        public PathChain Path1,Path2,Path3,Path4,Path5,Path6,Path7,Path8,Path9,Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.100, 134.500),

                                    new Pose(51.500, 91.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(51.500, 91.000),

                                    new Pose(46.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.000, 87.000),

                                    new Pose(21.500, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.500, 87.000),

                                    new Pose(51.500, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(51.500, 87.000),

                                    new Pose(48.000, 65.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 65.000),

                                    new Pose(21.500, 65.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.500, 65.000),

                                    new Pose(51.500, 91.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(51.500, 91.000),

                                    new Pose(46.000, 40.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.000, 40.000),

                                    new Pose(21.500, 40.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.500, 40.000),

                                    new Pose(61.000, 102.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }
}