package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MoveClass {
    DcMotor lf,rf,lb,rb;
    public MoveClass(DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb) {
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;
    }
    public void driveFieldCentric(double strafe, double forward, double rotate, double heading, double driveTrainPower) {
        double rotX = strafe * Math.cos(heading) -forward * Math.sin(heading);
        double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        double frontLeftPower = driveTrainPower * (rotY + rotX + rotate) / denominator;
        double backLeftPower = driveTrainPower * (rotY - rotX + rotate) / denominator;
        double frontRightPower = driveTrainPower * (rotY - rotX - rotate) / denominator;
        double backRightPower = driveTrainPower * (rotY + rotX - rotate) / denominator;
        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);
    }
    public void driveRobotCentric(double strafe, double forward, double rotate, double driveTrainPower) {
        driveFieldCentric(strafe,forward,rotate,0,driveTrainPower);
    }
}
