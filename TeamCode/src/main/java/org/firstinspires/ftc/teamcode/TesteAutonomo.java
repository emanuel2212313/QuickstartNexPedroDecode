package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class TesteAutonomo extends OpMode {

    DcMotor fe, fd, te, td;
    VoltageSensor batteryVoltage;

    double tempoInicio = 0;
    boolean andando = false;

    @Override
    public void init() {

        fe = hardwareMap.get(DcMotor.class, "FE");
        fd = hardwareMap.get(DcMotor.class, "FD");
        te = hardwareMap.get(DcMotor.class, "TE");
        td = hardwareMap.get(DcMotor.class, "TD");

        // Mesmas direções do seu TeleOp
        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    // ===== COMPENSAÇÃO DE VOLTAGEM =====
    private double compensar(double power) {
        double fator = 13.0 / batteryVoltage.getVoltage();
        return Math.max(-1, Math.min(1, power * fator));
    }

    @Override
    public void start() {
        tempoInicio = getRuntime();
        andando = true;
    }

    @Override
    public void loop() {

        if (andando) {

            if (getRuntime() - tempoInicio < 1.5) {
                // ANDAR PRA TRÁS
                double power = compensar(-0.5);

                fe.setPower(power);
                fd.setPower(power);
                te.setPower(power);
                td.setPower(power);

            } else {
                // PARA
                fe.setPower(0);
                fd.setPower(0);
                te.setPower(0);
                td.setPower(0);

                andando = false;
            }
        }

        telemetry.addData("Tempo", getRuntime() - tempoInicio);
        telemetry.addData("Bateria (V)", batteryVoltage.getVoltage());
        telemetry.update();
    }
}
