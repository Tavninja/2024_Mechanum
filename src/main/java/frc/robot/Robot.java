
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  //Set Motor values --------------------------------------------------------------
  /**
   * The drivetrain (mecanum)
   * @param m_bottomLeftMotor The rear left motor in the drivetrain
   * @param m_bottomRightMotor The rear right motor in the drivetrain
   * @param m_topLeftMotor The front left motor in the drivetrain
   * @param m_topRightMotor The front left motor in the drivetrain
   * 
   */
  private final static CANSparkMax m_bottomLeftMotor = new CANSparkMax(4, MotorType.kBrushed);
  private final static CANSparkMax m_bottomRightMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final static CANSparkMax m_topLeftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final static CANSparkMax m_topRightMotor = new CANSparkMax(2, MotorType.kBrushed);
  /**
   * @param m_NotePusher The motor that extends and retracts the note mechanism
   * @param m_NoteFlipper The motor That Flips the note
   * @param m_NoteShoot The motor that pushes the note into the shooters 
   */
  private final static CANSparkMax m_NotePusher = new CANSparkMax(6, MotorType.kBrushless);
    private final static CANSparkMax m_NoteShoot = new CANSparkMax(11, MotorType.kBrushless);
    private final TalonFX m_NoteFlipper = new TalonFX(10);
  /**
   * @param m_LeftShoot the motor that controls the left flywheel 
   * @param m_RightShoot the motor that controls the right flywheel
   */
  private final TalonFX m_LeftShooter = new TalonFX(5);
  private final TalonFX m_RightShooter = new TalonFX(7);
  /**
   * @param m_LeftClimer the motor that extends and retracts the left claw
   * @param m_RightClimer the motor that extends and retracts the right claw
   */
  private final TalonFX m_LeftClimer = new TalonFX(8);
  private final TalonFX m_RightClimer = new TalonFX(9);

  private final double fastShooterSpeed = .5;
  private final double NoteFlipperSpeed = .1;
  private final double NotePusherSpeed = .5;
  private final double climerSpeed = .35;

/*   Translation2d m_frontLeftLocation = new Translation2d(-.2794,.2667);
  Translation2d m_frontRightLocation = new Translation2d(.2794,.2667);
  Translation2d m_backLeftLocation = new Translation2d(-.2794,-.2667);
  Translation2d m_backRightLocation = new Translation2d(.2794,-.2667);
  MecanumDriveKinematics m_Kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
*/
  private SlewRateLimiter filter = new SlewRateLimiter(2.5);
  private SlewRateLimiter filter2 = new SlewRateLimiter(2.5);

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server; 

  // booleans ---------------------------------------
 
  private static Boolean LeftShooterout = false;
  private static Boolean LeftShooterin = false;

  private static Boolean RightShooterout = false;
  private static Boolean RightShooterin = false;

  private static Boolean NoteFlipperout = false; 
  private static Boolean NoteFlipperin = false;

  private static Boolean NotePusherout = false;
  private static Boolean NotePusherin = false;

  private static Boolean LeftClimerout = false;
  private static Boolean LeftClimerin = false;

  private static Boolean RightClimerout = false;
  private static Boolean RightClimerin = false;

  private static boolean LimitSwitchin = false;
  private static boolean LimitSwitchout = false;

  private static boolean test = false;

  // controllers ------------------------------------

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_opperatorController = new XboxController(1);

  @Override
  public void teleopPeriodic() {

// Mecanum Drive Math --------------------------------------------------------------

    //------------------------------------------------

    double y =  MathUtil.applyDeadband(-m_driverController.getLeftY(),.2);
    double x =  MathUtil.applyDeadband(m_driverController.getLeftX(),.2);
    double x2 = MathUtil.applyDeadband(m_driverController.getRightX(),.2);

    m_topLeftMotor.setInverted(true);
    m_topRightMotor.setInverted(false);
    m_bottomLeftMotor.setInverted(true);
    m_bottomRightMotor.setInverted(false); 
    
    double m_topLeftPower = (y+x+x2);
    double m_topRightPower = (y-x-x2);
    double m_bottomLeftPower = (y-x+x2);
    double m_bottomRightPower = (y+x-x2);

    m_topLeftMotor.set(m_topLeftPower);
    m_topRightMotor.set(m_topRightPower);
    m_bottomLeftMotor.set(m_bottomLeftPower);
    m_bottomRightMotor.set(m_bottomRightPower);


  // -------------------------------------------------------- 
/* 
    //ChassisSpeeds speeds= new ChassisSpeeds(-filter2.calculate(x),-filter.calculate(y),x2*40);
    ChassisSpeeds speeds= new ChassisSpeeds(x,y,x2*40);
    MecanumDriveWheelSpeeds wheelSpeeds = m_Kinematics.toWheelSpeeds(speeds);

    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond/1;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond/1;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond/1;
    double backRight = wheelSpeeds.rearRightMetersPerSecond/1;
     
    if (x2>= 0.1 || x2<=.1){
      frontRight= frontRight*-1;
      backLeft = backLeft*-1;
    } else {
      frontRight= frontRight*1;
      backLeft = backLeft*1;
    }

    m_topLeftMotor.set(frontLeft);
    m_topRightMotor.set(frontRight);
    m_bottomLeftMotor.set(backLeft);
    m_bottomRightMotor.set(backRight);
*/
// opperator controls -------------------------------------------------------------

  //Note Shoot -------------------------------------
   if  (m_opperatorController.getRightTriggerAxis() > .5){ // shooter in
    m_LeftShooter.set(-fastShooterSpeed);
    m_RightShooter.set(fastShooterSpeed);
        LeftShooterin = true;
        RightShooterin = true;
  } else if (m_opperatorController.getLeftTriggerAxis() > .5) { // shooter out
    m_LeftShooter.set(fastShooterSpeed);
    m_RightShooter.set(-fastShooterSpeed);
        LeftShooterout = true;
        RightShooterout = true;
  } else { // shooter stop
    m_LeftShooter.set(0); 
    m_RightShooter.set(0);
        LeftShooterin = false;
        LeftShooterout = false;
        RightShooterin = false;
        RightShooterout = false;
  }

  // note in and stop -------------------------------
  if (m_opperatorController.getRightBumper()){ //note in
    m_NotePusher.set(-NotePusherSpeed);
        NotePusherin = true;
        NotePusherout = false;
  } else if (m_opperatorController.getLeftBumper()){
    m_NotePusher.set(NotePusherSpeed);
        NotePusherin = false;
        NotePusherout = true;
  } else { // note pusher stop
    m_NotePusher.set(0);
        NotePusherin = false;
        NotePusherout = false;
        LimitSwitchin = false;
        LimitSwitchout = false;
  }

  // reset flipper ----------------------------------
  if (m_opperatorController.getAButton()) { // flipper left
    m_NoteFlipper.set(NoteFlipperSpeed);
    NoteFlipperin = true;
  } else if (m_opperatorController.getBButton()){ // flipper right
    m_NoteFlipper.set(-NoteFlipperSpeed);
    NoteFlipperout = true;
  } else { // flipper stop
    m_NoteFlipper.set(0);
    NoteFlipperout = false;
    NoteFlipperin = false;
  }

  //Climer ------------------------------------------

  if (m_driverController.getLeftBumper()){
    m_LeftClimer.set(climerSpeed);
        LeftClimerin = true;
  } else if (m_driverController.getLeftTriggerAxis() > .5){
    m_LeftClimer.set(-climerSpeed);
      LeftClimerout = true;
  } else if (m_driverController.getRightBumper()){
    m_RightClimer.set(-climerSpeed);
        RightClimerin = true;
  } else if (m_driverController.getRightTriggerAxis() > .5){
    m_RightClimer.set(climerSpeed);
        RightClimerout = true;
  } else {
    m_LeftClimer.set(0);
    m_RightClimer.set(0);
        LeftClimerin = false;
        RightClimerin = false;
        LeftClimerout = false;
        RightClimerout = false;
  }

// Push the note into shooter ------------------------
  if (m_opperatorController.getXButton()){
    m_NoteShoot.set(-.5);
  } else {
    m_NoteShoot.set(0);
  }



// SmartDashboard ----------------------------------------------------

    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("2x", x2);
  
 
  SmartDashboard.putBoolean("left_Climer_in", LeftClimerin.booleanValue());
  SmartDashboard.putBoolean("left_Climer_out", LeftClimerout.booleanValue());

  SmartDashboard.putBoolean("Right_Climer_in", RightClimerin.booleanValue());
  SmartDashboard.putBoolean("Right_Climer_out", RightClimerout.booleanValue());

  SmartDashboard.putBoolean("left_Shooter_in", LeftShooterin.booleanValue());
  SmartDashboard.putBoolean("left_Shooter_out", LeftShooterout.booleanValue());

  SmartDashboard.putBoolean("Right_Shooter_in", RightShooterin.booleanValue());
  SmartDashboard.putBoolean("Right_Shooter_out", RightShooterout.booleanValue());

  SmartDashboard.putBoolean("Note_Pusher_in", NotePusherin.booleanValue());
  SmartDashboard.putBoolean("Note_Pusher_out", NotePusherout.booleanValue());

  SmartDashboard.putBoolean("Note_Fliper_in", NoteFlipperin.booleanValue());
  SmartDashboard.putBoolean("Note_Fliper_out", NoteFlipperout.booleanValue());

  SmartDashboard.putBoolean("Limit_Switch_in", LimitSwitchin);
  SmartDashboard.putBoolean("Limit_Switch_out", LimitSwitchout);

  SmartDashboard.putNumber("topLeftPower2", (m_topLeftPower));
  SmartDashboard.putNumber("toprightPower", (m_topRightPower));
  SmartDashboard.putNumber("bottomLeftPower2", (m_bottomLeftPower));
  SmartDashboard.putNumber("m_bottomRightPower2", (m_bottomRightPower));

  SmartDashboard.putNumber("F y", filter.calculate(m_driverController.getLeftY()));
  SmartDashboard.putNumber("F x", filter.calculate(m_driverController.getLeftX()));
  SmartDashboard.putNumber("F x2",filter.calculate(m_driverController.getRightX()));

  SmartDashboard.putNumber("y", y);
  SmartDashboard.putNumber("x", x); 
  SmartDashboard.putNumber("x2", x2);
  
  ///SmartDashboard.putNumber("front Left", frontLeft);
  ///SmartDashboard.putNumber("front Right", frontRight);
  ///SmartDashboard.putNumber("back Left", backLeft);
  ///SmartDashboard.putNumber("back Right", backRight);
  
}
// Auto Time Contants ------------------------------------------------

  // Auto Time Contants
 private Timer timer;
 private double step1Time = 1;             //
 private double step2Time = step1Time + 1; //
 private double step3Time = step2Time + .5; //
 private double step4Time = step3Time + 1; //
 private double step5Time = step4Time + 1; //
 private double step6Time = step5Time + 1; //
 private static final String Stay_Still = "Stay Still";
 private static final String Backward = "Backward";
 private static final String Shoot_Back_Short = "Shoot_Back_Short";
 private static final String Shoot_Back_Long = "Shoot_Back_Long";


 private String m_autoSelected;
 private final SendableChooser<String> m_chooser = new SendableChooser<>();

// Movement functions ------------------------------------------------

 static void Forward(){
  m_topLeftMotor.set(-.5);
  m_topRightMotor.set(-.5);
  m_bottomLeftMotor.set(-.5);
  m_bottomRightMotor.set(.5);
 }
 static void Backwards(){
  m_topLeftMotor.set(.5);
  m_topRightMotor.set(.5);
  m_bottomLeftMotor.set(.5);
  m_bottomRightMotor.set(-.5);
 }
 static void Stoped(){
  m_topLeftMotor.set(0);
  m_topRightMotor.set(0);
  m_bottomLeftMotor.set(0);
  m_bottomRightMotor.set(0);
 }
@Override
 public void robotInit() {

// Camera ----------------------------------------------------------
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

// Auto Switcher ----------------------------------------------------
   m_chooser.setDefaultOption("Stay Still", Stay_Still);
   m_chooser.addOption("Backward", Backward);
   m_chooser.addOption("Shoot_Back_Short", Shoot_Back_Short);
   m_chooser.addOption("Shoot_Back_Long", Shoot_Back_Long);

   SmartDashboard.putData("Auto choices", m_chooser);
 }

 @Override
 public void autonomousInit() {
   m_autoSelected = m_chooser.getSelected();
   System.out.println("Auto selected: " + m_autoSelected);

   timer = new Timer();
   timer.start();
 }

 // Autonomous ---------------------------------------------------

 @Override
 public void autonomousPeriodic() {
   switch (m_autoSelected) {
     case Backward: //--------------------------Red 1
     if (timer.get() <= step1Time) {
      Backwards();
    } else if (timer.get() <= step2Time) {
      Backwards();
    } else if (timer.get() <= step3Time) {
      Stoped();
    }else if (timer.get() <= step4Time) {
      Stoped();
    } else if (timer.get() <= step5Time) {
      Stoped();
    } else if (timer.get() <= step6Time){
      Stoped();
    } else {
      Stoped();
    }
      break;

     case Shoot_Back_Short: //--------------------------Red 2
     if (timer.get() <= step1Time) {
      Stoped();
    } else if (timer.get() <= step2Time) {
      Stoped();
          m_LeftShooter.set(-fastShooterSpeed);
          m_RightShooter.set(fastShooterSpeed);
    } else if (timer.get() <= step3Time) {
          m_NoteShoot.set(-.5);
    }else if (timer.get() <= step4Time) {
      Backwards();
          m_NoteShoot.set(0);
          m_LeftShooter.set(0);
          m_RightShooter.set(0);
    } else if (timer.get() <= step5Time) {
      Backwards();
    } else if (timer.get() <= step6Time){
      Stoped();
    } else {
      Stoped();
    }
      break;

     case Shoot_Back_Long: //--------------------------Red 3
       if (timer.get() <= step1Time) {
      Stoped();
    } else if (timer.get() <= step2Time) {
      Stoped();
          m_LeftShooter.set(-fastShooterSpeed);
          m_RightShooter.set(fastShooterSpeed);
    } else if (timer.get() <= step3Time) {
          m_NoteShoot.set(-.5);
    }else if (timer.get() <= step4Time) {
      Backwards();
          m_NoteShoot.set(0);
          m_LeftShooter.set(0);
          m_RightShooter.set(0);
    } else if (timer.get() <= step5Time) {
      Backwards();
    } else if (timer.get() <= step6Time){
      Backwards();
    } else {
      Stoped();
    }
      break;

     case Stay_Still:
     default:
         Stoped();
   }
  }

// End ----------------------------------------------------------

  private void Stopped() {
  // TODO Auto-generated method stub
  throw new UnsupportedOperationException("Unimplemented method 'Stopped'");
}
  @Override
  public void robotPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
