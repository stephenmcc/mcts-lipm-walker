package us.ihmc.mctslipmwalker.simulation;

import com.google.common.util.concurrent.AtomicDouble;
import us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController;
import us.ihmc.mctslipmwalker.planners.SteppingStrategy;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

public class LIPMFlatGroundWalkerSimulation
{
   public static void main(String[] args)
   {
      PlaceholderRobot placeholderRobot = new PlaceholderRobot();

      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2("LIPM_MCTS_Simulation", SimulationConstructionSet2.doNothingPhysicsEngine());
      scs2.addRobot(placeholderRobot.getRobot());
      double dt = 1.0e-3;

      // Setup controller
//      SteppingStrategy steppingStrategy = SteppingStrategy.HEURISTIC;
//      SteppingStrategy steppingStrategy = SteppingStrategy.MCTS_REPLAY;
      SteppingStrategy steppingStrategy = SteppingStrategy.MCTS_ONLINE;

      ControllerDefinition controllerDefinition = LIPMWalker.createControllerDefinition(dt, steppingStrategy);
      RobotControllerManager controllerManager = placeholderRobot.getRobot().getControllerManager();
      LIPMWalker controller = (LIPMWalker) controllerDefinition.newController(controllerManager.getControllerInput(), controllerManager.getControllerOutput());
      scs2.addYoGraphic(controller.getSCS2YoGraphics());
      placeholderRobot.getRobot().addController(controller);

      GappedTerrain.addSCSGraphics(scs2);

      scs2.setDT(dt);
      scs2.setRealTimeRateSimulation(true);
      scs2.initializeBufferSize(20000);
      scs2.setBufferRecordTickPeriod(5);
//      scs2.setBufferRecordTickPeriod(10);

//      try
//      {
//         AtomicDouble desiredVelocity = setupJoystickListener();
//         scs2.addBeforePhysicsCallback(time ->
//                                       {
//                                          controller.getDesiredCruiseVelocity().set(desiredVelocity.get());
//                                       });
//      }
//      catch (JoystickNotFoundException e)
//      {
//         e.printStackTrace();
//      }

      scs2.start(true, false, false);
   }

   public static AtomicDouble setupJoystickListener() throws JoystickNotFoundException
   {
      AtomicDouble desiredVelocityToSet = new AtomicDouble();
      SharedMemoryMessager messager = new SharedMemoryMessager(XBoxOneJavaFXController.XBoxOneControllerAPI);
      new XBoxOneJavaFXController(messager);

      messager.addTopicListener(XBoxOneJavaFXController.LeftStickYAxis, desiredVelocityToSet::set);
      messager.startMessager();
      return desiredVelocityToSet;
   }
}
