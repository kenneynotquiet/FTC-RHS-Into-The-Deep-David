package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Intake extends Mechanism {

    DcMotorEx intake; // Intake declaration

    public String intakeName = "intake"; // Reference to name in config

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, intakeName);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.a) {
            intake(1);
        } else if (gamepad.b) {
            outtake(-1);
        } else {
            stop();
        }
    }

    /**
     * Turns on the intake at the certain speed
     *
     * @param speed The speed that the intake will intake at. Will always end up
     *              positive
     */
    public void intake(double speed) {
        intake.setPower(Math.abs(speed));
    }

    /**
     * Turns on the outtake at the certain speed
     *
     * @param speed The speed that the intake will outtake at. Will always end up
     *              negative
     */
    public void outtake(double speed) {
        intake.setPower(-Math.abs(speed));
    }

    /**
     * Stops the intake
     */
    public void stop() {
        intake.setPower(0);
    }
}