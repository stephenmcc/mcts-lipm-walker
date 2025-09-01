package us.ihmc.mctslipmwalker.simulation;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mctslipmwalker.planners.LIPMWalkerDesireds;
import us.ihmc.mctslipmwalker.planners.SteppingStrategy;
import us.ihmc.mctslipmwalker.planners.SteppingStrategyInterface;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCylinder3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.euclid.tools.EuclidCoreTools.square;

public class LIPMWalker implements Controller, SCS2YoGraphicHolder
{
   public static final double GRAVITY = 9.81;
   public static final double HEIGHT = 1.0;
   public static final double OMEGA = Math.sqrt(GRAVITY / HEIGHT);
   public static final double MIN_STEP_TIME = 0.2;
   public static final double ALPHA_STEP_LENGTH = 0.5;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final double dt;

   private final YoDouble basePosition1D = new YoDouble("basePosition_x", registry);
   private final YoDouble baseToICP1D = new YoDouble("baseToICP_x", registry);
   private final YoDouble copPosition1D = new YoDouble("copPosition_x", registry);
   private final YoDouble comPosition1D = new YoDouble("comPosition_x", registry);
   private final YoDouble comVelocity1D = new YoDouble("comVelocity_x", registry);
   private final YoDouble comAcceleration1D = new YoDouble("comAcceleration_x", registry);
   private final YoDouble icpPosition1D = new YoDouble("icpPosition_x", registry);

   private final YoFramePoint3D comPosition;
   private final YoFrameVector3D comVelocity;
   private final YoFramePoint3D icpPosition;

   private final YoFramePoint3D basePosition;
   private final YoFramePoint3D comBaseMidpoint;
   private final YoFrameVector3D baseToCom;
   private final YoDouble baseToComLength;

   private final LIPMWalkerDesireds walkerDesireds;
   private final YoDouble lastStepTime = new YoDouble("lastStepTime", registry);
   private final YoDouble time = new YoDouble("time", registry);

   private final SteppingStrategyInterface steppingStrategyInterface;

   public LIPMWalker(double dt, SteppingStrategy steppingStrategy)
   {
      this.dt = dt;

      comPosition = new YoFramePoint3D("comPosition", ReferenceFrame.getWorldFrame(), registry);
      comVelocity = new YoFrameVector3D("comVelocity", ReferenceFrame.getWorldFrame(), registry);
      icpPosition = new YoFramePoint3D("icpPosition", ReferenceFrame.getWorldFrame(), registry);

      basePosition = new YoFramePoint3D("basePosition", ReferenceFrame.getWorldFrame(), registry);
      comBaseMidpoint = new YoFramePoint3D("comBaseMidpoint", ReferenceFrame.getWorldFrame(), registry);
      baseToCom = new YoFrameVector3D("baseToCom", ReferenceFrame.getWorldFrame(), registry);
      baseToComLength = new YoDouble("baseToComLength", registry);

      walkerDesireds = new LIPMWalkerDesireds();
      registry.addChild(walkerDesireds.getRegistry());

      steppingStrategyInterface = steppingStrategy.build(walkerDesireds, registry);

      comPosition.set(0.0, 0.0, HEIGHT);
      basePosition.set(0.0, 0.0, 0.0);

      lastStepTime.set(-1.0);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      walkerDesireds.update();

      // CoP control so sim doesn't keep falling when stopped
      double footSize = 0.02;
      baseToICP1D.set(icpPosition1D.getValue() - basePosition1D.getValue());
      if (Math.abs(walkerDesireds.getDesiredCruiseVelocity()) < 1.0e-3 && Math.abs(baseToICP1D.getValue()) < footSize)
      { // add small cop feedback
         double k = 0.1;
         double baseToCoP = EuclidCoreTools.clamp((1.0 + k) * baseToICP1D.getValue(), footSize);
         copPosition1D.set(basePosition1D.getValue() + baseToCoP);
      }
      else
      {
         copPosition1D.set(basePosition1D.getValue());
      }

      // Do dynamics and update
      double baseToCoMX = comPosition1D.getValue() - copPosition1D.getValue();
      comAcceleration1D.set(baseToCoMX * square(OMEGA));

      if (baseToCoM() < 2.0)
      {
         comVelocity1D.add(dt * comAcceleration1D.getValue());
         comPosition1D.add(dt * comVelocity1D.getValue());
      }

      icpPosition1D.set(comPosition1D.getValue() + 1.0 / OMEGA * comVelocity1D.getValue());

      if (time.getValue() - lastStepTime.getValue() > MIN_STEP_TIME)
      { // consider taking a step
         double xb = steppingStrategyInterface.compute(time.getValue(), comPosition1D.getValue(), comVelocity1D.getValue(), basePosition1D.getValue(), icpPosition1D.getValue());

         boolean tookAStep = Math.abs(xb - basePosition1D.getValue()) > 1e-7;
         if (tookAStep)
         {
            basePosition1D.set(xb);
            snapToTerrain();
            lastStepTime.set(time.getValue());
         }
      }

      // Update 3D values/graphics
      comPosition.set(comPosition1D.getValue(), 0.0, HEIGHT);
      comVelocity.set(comVelocity1D.getValue(), 0.0, 0.0);
      basePosition.set(basePosition1D.getValue(), 0.0, 0.0);
      icpPosition.set(icpPosition1D.getValue(), 0.0, 0.0);
      baseToCom.sub(comPosition, basePosition);
      baseToComLength.set(baseToCom.norm());
      comBaseMidpoint.interpolate(comPosition, basePosition, 0.5);

      // Tick timer
      time.add(dt);
   }

   private void snapToTerrain()
   {
      double xBase = basePosition1D.getValue();
      for (int i = 0; i < GappedTerrain.TERRAIN.size() - 1; i++)
      {
         if (MathTools.intervalContains(xBase, GappedTerrain.TERRAIN.get(i).getLeft(), GappedTerrain.TERRAIN.get(i).getRight()))
         {
            break;
         }
         else if (MathTools.intervalContains(xBase, GappedTerrain.TERRAIN.get(i).getRight(), GappedTerrain.TERRAIN.get(i + 1).getLeft()))
         {
            double dist0 = Math.abs(GappedTerrain.TERRAIN.get(i).getRight() - xBase);
            double dist1 = Math.abs(GappedTerrain.TERRAIN.get(i + 1).getLeft() - xBase);
            basePosition1D.set(dist0 < dist1 ? GappedTerrain.TERRAIN.get(i).getRight() : GappedTerrain.TERRAIN.get(i + 1).getLeft());
         }
      }
   }

   private double baseToCoM()
   {
      return Math.abs(basePosition1D.getValue() - comPosition1D.getValue());
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("comPositionGraphic",
                                                                    comPosition,
                                                                    0.03,
                                                                    ColorDefinitions.Red()));

      YoGraphicCylinder3DDefinition baseCylinder = new YoGraphicCylinder3DDefinition();
      baseCylinder.setName("baseCylinder");
      baseCylinder.setColor(ColorDefinitions.Black());
      baseCylinder.setAxis(YoGraphicDefinitionFactory.newYoTuple3DDefinition(baseToCom));
      baseCylinder.setCenter(YoGraphicDefinitionFactory.newYoTuple3DDefinition(comBaseMidpoint));
      baseCylinder.setLength(YoGraphicDefinitionFactory.toPropertyName(baseToComLength));
      baseCylinder.setRadius(0.01);
      group.addChild(baseCylinder);

      return group;
   }

   public static ControllerDefinition createControllerDefinition(double dt, SteppingStrategy steppingStrategy)
   {
      return (input, output) -> new LIPMWalker(dt, steppingStrategy);
   }
}