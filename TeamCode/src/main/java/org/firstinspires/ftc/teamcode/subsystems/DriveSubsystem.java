package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;

public class DriveSubsystem {

    DcMotor fe, fd, te, td;
    VoltageSensor battery;

    public DriveSubsystem(HardwareMap hw, VoltageSensor battery) {
        this.battery = battery;

        fe = hw.get(DcMotor.class, "FE");
        fd = hw.get(DcMotor.class, "FD");
        te = hw.get(DcMotor.class, "TE");
        td = hw.get(DcMotor.class, "TD");

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private double compensar(double p) {
        return Math.max(-1, Math.min(1, p * (13.0 / battery.getVoltage())));
    }

    public void drive(double x, double y, double rot) {

        double feP = y + x + rot;
        double fdP = y - x - rot;
        double teP = y - x + rot;
        double tdP = y + x - rot;

        fe.setPower(compensar(feP));
        fd.setPower(compensar(fdP));
        te.setPower(compensar(teP));
        td.setPower(compensar(tdP));
    }
}
