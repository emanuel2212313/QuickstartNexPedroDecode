package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import java.util.List;

@TeleOp
public class CamTest extends OpMode {

    // ================= MOTORES =================
    DcMotor fe, fd, td, te;
    DcMotor intake;
    DcMotor launcher;
    DcMotor cameraMotor;

    double AzulFE, AzulTD, VermelhoTE, VermelhoFD;
    double SpinMode = 1;

    // ================= LIMELIGHT =================
    Limelight3A limelight;
    final int TARGET_ID = 20;

    // ================= CAMERA / ODOMETRIA =================
    final double TICKS_POR_GRAU = 28.68; // goBILDA 19.2:1
    final double kP = 0.015;
    final double kD = 0.004;
    final double DEAD_DEG = 0.4;
    final double MAX_POWER = 0.6;

    double anguloCamera = 0;
    double anguloGol = 0;
    boolean golConhecido = false;
    double ultimoErro = 0;

    // ================= LANÇADOR =================
    boolean launcherAtivo = false;
    boolean lbAnterior = false;

    // ================= VOLTAGEM =================
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
        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        // ================= LIMELIGHT =================
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    // ================= COMPENSAÇÃO DE VOLTAGEM =================
    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void loop() {

        // ================= ODOMETRIA DA CÂMERA =================
        anguloCamera = cameraMotor.getCurrentPosition() / TICKS_POR_GRAU;

        // ================= VISÃO =================
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

        // ================= ATUALIZA POSIÇÃO DO GOL =================
        if (viuTag) {
            anguloGol = anguloCamera + txDegrees;
            golConhecido = true;
        }

        // ================= CONTROLE DA CÂMERA =================
        double giroCamera = 0;

        if (golConhecido) {
            double erro = anguloGol - anguloCamera;
            double derivada = erro - ultimoErro;
            ultimoErro = erro;

            giroCamera = (erro * kP) + (derivada * kD);

            if (Math.abs(erro) < DEAD_DEG) {
                giroCamera = 0;
            }
        } else {
            // nunca viu o gol → varredura inicial lenta
            giroCamera = 0.12;
        }

        giroCamera = Math.max(-MAX_POWER, Math.min(MAX_POWER, giroCamera));
        cameraMotor.setPower(compensar(giroCamera));

        // ================= MOVIMENTO =================
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotRobo = gamepad1.right_stick_x;

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

        // ================= LANÇADOR =================
        boolean lbAtual = gamepad1.left_bumper;
        if (lbAtual && !lbAnterior) {
            launcherAtivo = !launcherAtivo;
        }
        lbAnterior = lbAtual;

        if (launcherAtivo) launcher.setPower(compensar(1.0));
        else launcher.setPower(0);

        // ================= TELEMETRIA =================
        telemetry.addData("Gol conhecido", golConhecido);
        telemetry.addData("Ângulo câmera", anguloCamera);
        telemetry.addData("Ângulo gol", anguloGol);
        telemetry.addData("Tag vista", viuTag);
        telemetry.addData("Bateria", batteryVoltage.getVoltage());
        telemetry.update();
    }
}
