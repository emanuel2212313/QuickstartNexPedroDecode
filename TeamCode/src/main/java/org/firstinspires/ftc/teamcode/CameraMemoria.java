package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@TeleOp
public class CameraMemoria extends OpMode {

    DcMotor fe, fd, td, te;
    DcMotor intake, launcher, cameraMotor;
    Servo servoEmpurrador;

    double AzulFE, AzulTD, VermelhoTE, VermelhoFD;
    double SpinMode = 1;

    final double SERVO_REPOUSO = 0.2;
    final double SERVO_EMPURRE = 0.5;

    Limelight3A limelight;
    final int TARGET_ID = 20;

    final double kP = 0.030;
    final double kD = 0.003;
    final double DEAD_DEG = 0.4;
    final double MAX_POWER = 0.6;

    double ultimoErro = 0;
    double ultimoGiro = 0;
    double tempoUltimaTag = 0;

    boolean launcherAtivo = false, lbAnterior = false;
    boolean intakeAtivo = false, rtAnterior = false;

    VoltageSensor batteryVoltage;

    @Override
    public void init() {

        fd = hardwareMap.get(DcMotor.class, "FD");
        fe = hardwareMap.get(DcMotor.class, "FE");
        te = hardwareMap.get(DcMotor.class, "TE");
        td = hardwareMap.get(DcMotor.class, "TD");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        cameraMotor = hardwareMap.get(DcMotor.class, "cameraMotor");

        servoEmpurrador = hardwareMap.get(Servo.class, "servoEmpurrador");

        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        servoEmpurrador.setDirection(Servo.Direction.REVERSE);
        servoEmpurrador.setPosition(SERVO_REPOUSO);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    private double compensar(double power) {
        return Math.max(-1, Math.min(1, power * (13.0 / batteryVoltage.getVoltage())));
    }

    @Override
    public void loop() {

        // ===== CÂMERA COM MEMÓRIA =====
        double tx = 0;
        boolean viuTag = false;

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    if (fr.getFiducialId() == TARGET_ID) {
                        tx = fr.getTargetXDegrees();
                        viuTag = true;
                        break;
                    }
                }
            }
        }

        if (viuTag) {
            double erro = tx;
            double giro = (erro * kP) + ((erro - ultimoErro) * kD);
            if (Math.abs(erro) < DEAD_DEG) giro = 0;
            giro = Math.max(-MAX_POWER, Math.min(MAX_POWER, giro));

            cameraMotor.setPower(giro);
            ultimoErro = erro;
            ultimoGiro = giro;
            tempoUltimaTag = getRuntime();
        } else {
            if (getRuntime() - tempoUltimaTag < 0.25)
                cameraMotor.setPower(ultimoGiro * 0.6);
            else
                cameraMotor.setPower(0);
        }

        // ===== MOVIMENTO =====
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        AzulFE = y + x + gamepad1.right_stick_x;
        AzulTD = y + x - gamepad1.right_stick_x;
        VermelhoTE = y - x + gamepad1.right_stick_x;
        VermelhoFD = y - x - gamepad1.right_stick_x;

        fe.setPower(AzulFE);
        td.setPower(AzulTD);
        te.setPower(VermelhoTE);
        fd.setPower(VermelhoFD);

        // ===== INTAKE =====
        boolean rt = gamepad1.right_trigger > 0.5;
        if (rt && !rtAnterior) intakeAtivo = !intakeAtivo;
        rtAnterior = rt;

        intake.setPower(intakeAtivo ? compensar(0.8) : 0);

        // ===== LANÇADOR =====
        boolean lb = gamepad1.left_bumper;
        if (lb && !lbAnterior) launcherAtivo = !launcherAtivo;
        lbAnterior = lb;

        launcher.setPower(launcherAtivo ? compensar(0.8) : 0);

        // ===== SERVO =====
        servoEmpurrador.setPosition(
                gamepad1.left_trigger > 0.1 ? SERVO_EMPURRE : SERVO_REPOUSO
        );

        telemetry.addData("Tag", viuTag);
        telemetry.update();
    }
}
