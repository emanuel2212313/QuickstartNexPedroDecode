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
public class OiAmigo extends OpMode {

    DcMotor fe, fd, td, te;
    DcMotor intake;
    DcMotor launcher;

    // ===== MOTOR DA CÂMERA =====
    DcMotor cameraMotor;

    Servo servoEmpurrador;

    double AzulFE, AzulTD, VermelhoTE, VermelhoFD;
    double SpinMode = 1;

    final double SERVO_REPOUSO = 0.2;
    final double SERVO_EMPURRE = 0.5;

    // ===== LIMELIGHT =====
    Limelight3A limelight;
    final int TARGET_ID = 20;

    // ===== CONTROLE GIMBAL DA CÂMERA =====
    final double kP = 0.030;   // proporcional
    final double kD = 0.003;   // amortecimento
    final double DEAD_DEG = 0.4;
    final double MAX_POWER = 0.6;

    double ultimoErro = 0;

    // ===== RB PUNCH =====
    boolean rbAtivo = false;
    double tempoRB = 0;
    int faseRB = 0;

    final double RB_PULL_POWER = -0.5;
    final double RB_PUSH_POWER = 0.7;
    final double RB_PULL_TIME  = 0.18;
    final double RB_PUSH_TIME  = 0.18;

    // ===== TOGGLES =====
    boolean launcherAtivo = false;
    boolean lbAnterior = false;

    boolean intakeAtivo = false;
    boolean rtAnterior = false;

    // ===== VOLTAGEM =====
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

        servoEmpurrador.setDirection(Servo.Direction.REVERSE);
        servoEmpurrador.setPosition(SERVO_REPOUSO);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== LIMELIGHT =====
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    // ===== COMPENSAÇÃO DE VOLTAGEM =====
    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void loop() {

        // ================= ALINHAMENTO GIMBAL DA CÂMERA =================
        double txDegrees = 0;
        boolean viuTag = false;

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        if (fr.getFiducialId() == TARGET_ID) {
                            txDegrees = fr.getTargetXDegrees();
                            viuTag = true;
                            break;
                        }
                    }
                }
            }
        }

        if (viuTag) {
            double erro = txDegrees;
            double derivada = erro - ultimoErro;

            double giroCamera = (erro * kP) + (derivada * kD);
            ultimoErro = erro;

            if (Math.abs(erro) < DEAD_DEG) {
                giroCamera = 0;
            }

            giroCamera = Math.max(-MAX_POWER, Math.min(MAX_POWER, giroCamera));
            cameraMotor.setPower(giroCamera);
        } else {
            cameraMotor.setPower(0);
            ultimoErro = 0;
        }

        // ================= MOVIMENTO =================
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        double FAzul = y + x;
        double FVermelho = y - x;

        if (gamepad1.right_stick_button) {
            SpinMode = (SpinMode == 0) ? 1 : 0;
        }

        if (SpinMode == 0) {
            AzulFE = FAzul + (gamepad1.right_stick_x * 1.8);
            AzulTD = FAzul - (gamepad1.right_stick_x * 1.8);
            VermelhoTE = FVermelho + (gamepad1.right_stick_x * 1.8);
            VermelhoFD = FVermelho - (gamepad1.right_stick_x * 1.8);
        } else {
            AzulFE = FAzul + gamepad1.right_stick_x;
            AzulTD = FAzul - gamepad1.right_stick_x;
            VermelhoTE = FVermelho + gamepad1.right_stick_x;
            VermelhoFD = FVermelho - gamepad1.right_stick_x;
        }

        fd.setPower(VermelhoFD);
        te.setPower(VermelhoTE);
        fe.setPower(AzulFE);
        td.setPower(AzulTD);

        // ================= INTAKE + RB =================
        boolean rtPressionado = gamepad1.right_trigger > 0.5;

        if (!rbAtivo) {
            if (rtPressionado && !rtAnterior) intakeAtivo = !intakeAtivo;
            rtAnterior = rtPressionado;

            if (intakeAtivo) intake.setPower(compensar(0.8));
            else intake.setPower(0);
        }

        if (gamepad1.right_bumper && !rbAtivo) {
            rbAtivo = true;
            faseRB = 1;
            tempoRB = getRuntime();
            intakeAtivo = false;
            intake.setPower(0);
        }

        if (rbAtivo) {
            double agora = getRuntime();

            if (faseRB == 1) {
                intake.setPower(compensar(RB_PULL_POWER));
                if (agora - tempoRB >= RB_PULL_TIME) {
                    faseRB = 2;
                    tempoRB = agora;
                }
            } else if (faseRB == 2) {
                intake.setPower(compensar(RB_PUSH_POWER));
                if (agora - tempoRB >= RB_PUSH_TIME) {
                    rbAtivo = false;
                    faseRB = 0;
                    intake.setPower(0);
                }
            }
        }

        // ================= LANÇADOR =================
        boolean lbAtual = gamepad1.left_bumper;
        if (lbAtual && !lbAnterior) launcherAtivo = !launcherAtivo;
        lbAnterior = lbAtual;

        if (launcherAtivo) launcher.setPower(compensar(0.8));
        else launcher.setPower(0);

        // ================= SERVO EMPURRADOR =================
        if (gamepad1.left_trigger > 0.1)
            servoEmpurrador.setPosition(SERVO_EMPURRE);
        else
            servoEmpurrador.setPosition(SERVO_REPOUSO);

        telemetry.addData("Tag Vista", viuTag);
        telemetry.addData("tx (graus)", txDegrees);
        telemetry.addData("Bateria (V)", batteryVoltage.getVoltage());
        telemetry.update();
    }
}