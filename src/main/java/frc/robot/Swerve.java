import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;


public final class Swerve {
    private final RelativeEncoder encoder;
    private final int steerPort;
    private final int drivePort;
    private final double ang=0; 
    private final double offset; // radians
    private final CANSparkMax steerMotor;
    private final  CANSparkMax driveMotor;
    public static void main ( int steerArg, int driveArg, double off){
        steerPort = steerArg;
        drivePort = driveArg;
        offset = off;
        steerMotor = new CANSparkMax(steerPort,MotorType.kBrushless);
        driveMotor = new CANSparkMax(drivePort,MotorType.kBrushless);
    }
    private final double kp = 2.;
    private final double ki=0.1;
    private final double kd = 0.1;
    private final PIDController PID = new PIDController(kp, ki, kd);
   
    public static void idle() {
        steerMotor.set(PID.calculate(encoder.getPosition(), offset/Math.PI));
        
    }
    public static void steer(double refAng){
        if (Math.abs(refAng-ang)>Math.abs(refAng-ang-2*Math.PI)){
            steerMotor.set(PID.calculate(steerMotor.getEncoder().getPosition(), (ang+(ang-refAng-2*Math.PI))/Math.PI));
            ang = steerMotor.getEncoder().getPosition()*Math.PI;
        } else {
            steerMotor.set(PID.calculate(br_steer_steerMotor.getEncoder().getPosition(), refAng/Math.PI));
            ang = steerMotor.getEncoder().getPosition()*Math.PI;

        }
        
    }

}