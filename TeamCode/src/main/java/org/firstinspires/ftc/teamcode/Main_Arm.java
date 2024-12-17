package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Main_Arm {
//Create motor/servo Objects
    public DcMotor armMotor;
    public DcMotor armPivot;
    public DcMotor armPivot2;
    public Servo clawPivot;
    public Servo clawPivot2;
    public Servo clawRotate;
    public CRServo intakeGo;

//    Initializing Hardware
    public void init (HardwareMap HardwareMap){
        armMotor = HardwareMap.get(DcMotor.class, "armMotor");
        armPivot = HardwareMap.get(DcMotor.class, "armPivot");
        armPivot2 = HardwareMap.get(DcMotor.class, "armPivot2");
        clawPivot = HardwareMap.get(Servo.class, "clawPivot");
        clawPivot2 = HardwareMap.get(Servo.class, "clawPivot2");
        clawRotate = HardwareMap.get(Servo.class, "clawRotate");
        intakeGo = HardwareMap.get(CRServo.class, "intakeGo");
    }





}
