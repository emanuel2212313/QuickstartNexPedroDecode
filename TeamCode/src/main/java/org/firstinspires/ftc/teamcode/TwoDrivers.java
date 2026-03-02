package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TwoDrivers extends OpMode {

    // ===== DRIVETRAIN =====
    DcMotor fe, fd, td, te;

    // ===== MECANISMOS =====
    DcMotor intake;
    DcMotor intake2;
    DcMotor launcher;
    DcMotor cameraMotor;

    DistanceSensor distanceSensor;

    // ===== ESTADOS =====
    boolean launcherAtivo = false;
    boolean lbAnterior = false;

    boolean intakeAutomatico = false;
    boolean intakeManual = false;
    boolean rtAnterior = false;

    boolean intake2Travado = false;

    // ===== TIMER =====
    ElapsedTime intakeTimer = new ElapsedTime();

    // ===== DRIVE =====
    double AzulFE, AzulTD, VermelhoTE, VermelhoFD;
    double SpinMode = 1;

    // ===== VOLTAGEM =====
    VoltageSensor batteryVoltage;

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

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void loop() {

        // =================================================
        // 🅰️ DRIVER A — MOVIMENTO
        // =================================================
        double rotRobo = gamepad1.right_stick_x;
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        double FAzul = y + x;
        double FVermelho = y - x;

        if (gamepad1.right_stick_button) {
            SpinMode = (SpinMode == 0) ? 1 : 0;
        }

        if (SpinMode == 0) {
            AzulFE = FAzul + (rotRobo * 1.8);
            AzulTD = FAzul - (rotRobo * 1.8);
            VermelhoTE = FVermelho + (rotRobo * 1.8);
            VermelhoFD = FVermelho - (rotRobo * 1.8);
        } else {
            AzulFE = FAzul + rotRobo;
            AzulTD = FAzul - rotRobo;
            VermelhoTE = FVermelho + rotRobo;
            VermelhoFD = FVermelho - rotRobo;
        }

        fd.setPower(compensar(VermelhoFD));
        te.setPower(compensar(VermelhoTE));
        fe.setPower(compensar(AzulFE));
        td.setPower(compensar(AzulTD));


        // =================================================
        // 🅰️ DRIVER A — INTAKE MANUAL (RT)
        // =================================================
        boolean rtAtual = gamepad1.right_trigger > 0.8;
        if (!launcherAtivo && rtAtual && !rtAnterior) {
            intakeManual = !intakeManual;
        }
        rtAnterior = rtAtual;


        // =================================================
        // 🅱️ DRIVER B — SENSOR PIXEL
        // =================================================
        double distancia = distanceSensor.getDistance(DistanceUnit.CM);

        if (!launcherAtivo) {
            if (distancia < 8) intake2Travado = true;
        } else {
            intake2Travado = false;
        }


        // =================================================
        // 🅱️ DRIVER B — LAUNCHER (LB)
        // =================================================
        boolean lbAtual = gamepad2.left_bumper;

        if (lbAtual && !lbAnterior) {
            launcherAtivo = !launcherAtivo;
            intakeTimer.reset();

            if (!launcherAtivo) intakeAutomatico = false;
        }
        lbAnterior = lbAtual;

        launcher.setPower(launcherAtivo ? compensar(1.0) : 0);


        // =================================================
        // INTAKE AUTOMÁTICO DO LANÇAMENTO
        // =================================================
        if (launcherAtivo && !intakeAutomatico && intakeTimer.seconds() >= 2.5) {
            intakeAutomatico = true;
        }


        // =================================================
        // POTÊNCIA FINAL DO INTAKE
        // =================================================
        double intakePower;

        if (launcherAtivo)
            intakePower = intakeAutomatico ? 0.7 : 0;
        else
            intakePower = intakeManual ? 0.7 : 0;

        intake.setPower(compensar(intakePower));

        if (!intake2Travado)
            intake2.setPower(compensar(intakePower));
        else
            intake2.setPower(0);


        // =================================================
        // 🅱️ DRIVER B — CÂMERA
        // =================================================
        double torretaPower = gamepad2.right_stick_x;
        cameraMotor.setPower(compensar(torretaPower * 0.6));


        telemetry.addData("Distancia", distancia);
        telemetry.addData("Intake2 Travado", intake2Travado);
        telemetry.addData("Launcher", launcherAtivo);
        telemetry.update();
    }
}
