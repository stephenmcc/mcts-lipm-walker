package us.ihmc.mctslipmwalker;

import com.google.common.util.concurrent.AtomicDouble;
import us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicBox3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

public class LIPMFlatGroundWalkerSimulation
{
   public static void main(String[] args)
   {
      PlaceholderRobot placeholderRobot = new PlaceholderRobot();

      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2("AcrobotSimulation", SimulationConstructionSet2.doNothingPhysicsEngine());
      scs2.addRobot(placeholderRobot.getRobot());
      double dt = 1.0e-3;

      // Setup controller
      ControllerDefinition controllerDefinition = LIPMWalker.createControllerDefinition(dt);
      RobotControllerManager controllerManager = placeholderRobot.getRobot().getControllerManager();
      LIPMWalker controller = (LIPMWalker) controllerDefinition.newController(controllerManager.getControllerInput(), controllerManager.getControllerOutput());
      scs2.addYoGraphic(controller.getSCS2YoGraphics());
      placeholderRobot.getRobot().addController(controller);

      // Add ground graphic
//      YoGraphicBox3DDefinition groundBox = new YoGraphicBox3DDefinition();
//      groundBox.setName("groundBox");
//      groundBox.setColor(ColorDefinitions.Green());
//      groundBox.setPosition(YoGraphicDefinitionFactory.newYoTuple3DDefinition(0.0, 0.0, -0.5, ReferenceFrame.getWorldFrame()));
//      groundBox.setSize(YoGraphicDefinitionFactory.newYoTuple3DDefinition(10.0, 10.0, 1.0, ReferenceFrame.getWorldFrame()));
//      groundBox.setOrientation(YoGraphicDefinitionFactory.newYoOrientation3DDefinition(new Quaternion(), ReferenceFrame.getWorldFrame()));
//      scs2.addYoGraphic(groundBox);

      GappedTerrain.addSCSGraphics(scs2);

      scs2.setDT(dt);
      scs2.setRealTimeRateSimulation(true);
      scs2.initializeBufferSize(16000);
      scs2.setBufferRecordTickPeriod(10);

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
