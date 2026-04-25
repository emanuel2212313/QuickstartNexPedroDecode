package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class TesteDrive extends OpMode {

    DcMotor fe, fd, td, te;
    DcMotor intake, intake2, launcher, cameraMotor;

    Follower follower;
    DistanceSensor distanceSensor;

    Limelight3A limelight;
    final int TARGET_ID = 20;

    // CAMERA PID
    final double kP = 0.02;
    final double kD = 0.004;
    final double MAX_POWER = 0.6;

    double ultimoErro = 0;
    double lastTx = 0;
    long lastSeenTime = 0;
    boolean scanRight = true;

    boolean tagAlinhada = false;

    ElapsedTime intakeTimer = new ElapsedTime();

    boolean launcherAtivo = false;
    boolean lbAnterior = false;

    boolean intakeAutomatico = false;
    boolean intakeManual = false;
    boolean rtAnterior = false;

    boolean intake2Travado = false;

    double AzulFE, AzulTD, VermelhoTE, VermelhoFD;
    double SpinMode = 1;
    double launcherPowerMultiplier = 1.0;

    VoltageSensor batteryVoltage;

    boolean fieldCentric = true;
    boolean toggleAnterior = false;

    @Override
    public void init() {

        fd = hardwareMap.get(DcMotor.class, "FD");
        fe = hardwareMap.get(DcMotor.class, "FE");
        te = hardwareMap.get(DcMotor.class, "TE");
        td = hardwareMap.get(DcMotor.class, "TD");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        cameraMotor = hardwareMap.get(DcMotor.class, "cameraMotor");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        // 🔥 CORREÇÃO INTAKES
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        follower = Constants.createFollower(hardwareMap);
    }

    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void loop() {

        follower.update();

        // RESET HEADING
        if (gamepad1.options) {
            follower.setPose(new Pose(0, 0, 0));
        }

        // TOGGLE FIELD
        if (gamepad1.share && !toggleAnterior) {
            fieldCentric = !fieldCentric;
        }
        toggleAnterior = gamepad1.share;

        // DRIVE
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotRobo = gamepad1.right_stick_x;

        double heading = follower.getPose().getHeading();

        double rotX = x;
        double rotY = y;

        if (fieldCentric) {
            rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        }

        double FAzul = rotY + rotX;
        double FVermelho = rotY - rotX;

        AzulFE = FAzul + rotRobo;
        AzulTD = FAzul - rotRobo;
        VermelhoTE = FVermelho + rotRobo;
        VermelhoFD = FVermelho - rotRobo;

        fd.setPower(compensar(VermelhoFD));
        te.setPower(compensar(VermelhoTE));
        fe.setPower(compensar(AzulFE));
        td.setPower(compensar(AzulTD));

        // ================= CAMERA =================
        double rotCamera = -rotRobo;
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

            giroCamera = (erro * kP) + (derivada * kD) + (rotCamera * 0.35);

            if (Math.abs(erro) < 0.3)
                giroCamera *= 0.4;

            tagAlinhada = Math.abs(erro) < 0.3;

        } else {
            tagAlinhada = false;

            long tempoPerdido = now - lastSeenTime;

            if (tempoPerdido < 700) {
                giroCamera = (lastTx * kP * 0.5) + (rotCamera * 0.35);
            } else if (tempoPerdido < 1800) {
                giroCamera = rotCamera * 0.35;
                ultimoErro = 0;
            } else {
                double scanPower = 0.12;
                giroCamera = scanRight ? scanPower : -scanPower;

                if ((now / 1000) % 3 == 0)
                    scanRight = !scanRight;
            }
        }

        giroCamera = Math.max(-MAX_POWER, Math.min(MAX_POWER, giroCamera));

        int pos = cameraMotor.getCurrentPosition();
        int CAMERA_MIN = -634;
        int CAMERA_MAX = 634;

        if (giroCamera > 0 && pos >= CAMERA_MAX) giroCamera = 0;
        if (giroCamera < 0 && pos <= CAMERA_MIN) giroCamera = 0;

        cameraMotor.setPower(compensar(giroCamera));

        // SENSOR
        double distancia = distanceSensor.getDistance(DistanceUnit.CM);

        if (!launcherAtivo && distancia < 11)
            intake2Travado = true;
        else if (launcherAtivo)
            intake2Travado = false;

        // INTAKE MANUAL
        boolean rtAtual = gamepad2.right_trigger > 0.8;
        if (!launcherAtivo && rtAtual && !rtAnterior)
            intakeManual = !intakeManual;
        rtAnterior = rtAtual;

        // LAUNCHER
        boolean lbAtual = gamepad2.left_bumper;
        if (lbAtual && !lbAnterior) {
            launcherAtivo = !launcherAtivo;
            intakeTimer.reset();
            if (!launcherAtivo) intakeAutomatico = false;
        }
        lbAnterior = lbAtual;

        double launcherPower = 0;

        if (tagAlinhada && !launcherAtivo)
            launcherPower = 0.5;

        if (launcherAtivo)
            launcherPower = launcherPowerMultiplier;

        launcher.setPower(compensar(launcherPower));

        // INTAKE AUTO
        if (launcherAtivo && !intakeAutomatico && intakeTimer.seconds() >= 1.5)
            intakeAutomatico = true;

        double intakePower;
        if (launcherAtivo) intakePower = intakeAutomatico ? 0.7 : 0;
        else intakePower = intakeManual ? 1.0 : 0;

        intake.setPower(compensar(intakePower));

        if (!intake2Travado)
            intake2.setPower(compensar(intakePower));
        else
            intake2.setPower(0);

        telemetry.addData("Modo", fieldCentric ? "Field" : "Robot");
        telemetry.addData("Heading", Math.toDegrees(heading));
        telemetry.addData("Launcher", launcherAtivo);
        telemetry.addData("Distancia", distancia);
        telemetry.addData("Tag", tagAlinhada);
        telemetry.update();
    }
}