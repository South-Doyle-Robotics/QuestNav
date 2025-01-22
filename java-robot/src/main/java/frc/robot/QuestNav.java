package frc.robot;

import java.nio.charset.StandardCharsets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNav {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
  private IntegerSubscriber m_msgMISO = nt4Table.getIntegerTopic("msgMISO").subscribe(0);
  private IntegerPublisher m_msgMOSI = nt4Table.getIntegerTopic("msgMOSI").publish();
  private DoubleArrayPublisher initPose = nt4Table.getDoubleArrayTopic("resetpose").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  private String m_lastQuestMessage = new String();
  private double m_lastQuestMessageTime = 0.0;

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private double m_initialYaw = 0.0;
  private Pose2d resetPose = new Pose2d();

  public QuestNav() {
    zeroHeading();
  }

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    Pose2d pose = getQuestNavPose();
    
    return new Pose2d(
      pose.getTranslation().minus(resetPose.getTranslation()),
      pose.getRotation().minus(resetPose.getRotation())
    );
  }

  // Gets the battery percent of the Quest.
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp.
  public double timestamp() {
    return questTimestamp.get();
  }

  // Zero the relativerobot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public void resetPose() {
    System.out.printf("Resetting zero pose to current...\n");
    resetPose = getPose();
    if (questMiso.get() != 99) {
     questMosi.set(1);
    }
  }

  public void setInitialPose(Pose2d pose) {
    if (questMiso.get() == 99) {
      initPose.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
      questMosi.set(2);
    }
  }

  public void testMessages() {
    if (m_msgMISO.get() == 0) {
      System.out.printf("testing QuestNav message system...\n");
      m_msgMOSI.set(1);
    }
  }

  public void testMessagesOff() {
    m_msgMOSI.set(0);
  }

  public void checkMessages() {
    if (m_msgMISO.get() == 1) {
      long[] message = nt4Table.getEntry("message").getIntegerArray(new long[] {0});
      
      char[] msgBytes = new char[message.length];
      for (int i = 0; i < message.length; i++)
        msgBytes[i] = (char)message[i];
      String msgStr = new String(msgBytes);

      boolean renderMsg = msgStr.contains("render graph API");
      boolean shouldDisplay = (double)System.currentTimeMillis() * 0.001 > m_lastQuestMessageTime + 0.25 || !msgStr.contentEquals(m_lastQuestMessage);
      if (shouldDisplay) {
        m_lastQuestMessage = msgStr;
        m_lastQuestMessageTime = (double)System.currentTimeMillis() * 0.001;
        System.out.printf("QuestNav -> %s\n", msgStr);
      }

      m_msgMOSI.set(0);
    }
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret -= m_initialYaw;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  private Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  private Pose2d getQuestNavPose() {
    var oculousPositionCompensated = getQuestNavTranslation();
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }
}
