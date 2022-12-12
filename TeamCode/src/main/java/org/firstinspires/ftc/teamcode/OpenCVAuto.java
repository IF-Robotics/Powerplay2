package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class OpenCVAuto extends Hardwaremap {

    public signalPosition getSignalPosition() {
        return signalPosition.Error;
    }

    public enum signalPosition{
        One,
        Two,
        Three,
        Timeout,
        Error
    }

    public void strafe(double power, int direction, int time) {
        front_Left.setPower(power*direction);
        back_Leftx.setPower(-1*power*direction);
        back_Right.setPower(power*direction);
        front_Right.setPower(-1*power*direction);
        sleep(time);
        front_Left.setPower(0);
        back_Leftx.setPower(0);
        back_Right.setPower(0);
        front_Right.setPower(0);
        sleep(100);
    }
    public void drive(double power, int direction, int time) {
        front_Left.setPower(-power*direction);
        back_Leftx.setPower(-power*direction);
        back_Right.setPower(-power*direction);
        front_Right.setPower(-power*direction);
        sleep(time);
        front_Left.setPower(0);
        back_Leftx.setPower(0);
        back_Right.setPower(0);
        front_Right.setPower(0);
        sleep(100);
    }
}
