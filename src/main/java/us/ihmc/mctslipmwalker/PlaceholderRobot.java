package us.ihmc.mctslipmwalker;

import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;

public class PlaceholderRobot
{
   private final RobotDefinition robotDefinition;
   private final RigidBodyDefinition rootBodyDefinition;
   private final Robot robot;

   public PlaceholderRobot()
   {
      robotDefinition = new RobotDefinition("placeholder");
      rootBodyDefinition = new RigidBodyDefinition("rootBody");
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);
      robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   public Robot getRobot()
   {
      return robot;
   }

}
