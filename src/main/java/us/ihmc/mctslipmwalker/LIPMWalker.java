package us.ihmc.mctslipmwalker;

import us.ihmc.commons.DeadbandTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
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
   private static final double GRAVITY = 9.81;
   private static final double HEIGHT = 1.0;
   private static final double OMEGA = Math.sqrt(GRAVITY / HEIGHT);
   private static final double MIN_STEP_TIME = 0.18;

   /* When walking at peak velocity v_p, step at length alpha * v_p */
   private static final double ALPHA_STEP_LENGTH = 0.4;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final double dt;
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;

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

   private final double[] maxVelocityReferences;
   private final double[] cruiseVelocityReferences;

   private final YoDouble desiredCruiseVelocity = new YoDouble("desiredWalkerVelocity", registry);
   private final YoDouble desiredPeakVelocity = new YoDouble("desiredPeakVelocity", registry);
   private final YoDouble lastStepTime = new YoDouble("lastStepTime", registry);
   private final YoDouble time = new YoDouble("time", registry);

   public LIPMWalker(ControllerInput controllerInput, ControllerOutput controllerOutput, double dt)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
      this.dt = dt;

      comPosition = new YoFramePoint3D("comPosition", ReferenceFrame.getWorldFrame(), registry);
      comVelocity = new YoFrameVector3D("comVelocity", ReferenceFrame.getWorldFrame(), registry);
      icpPosition = new YoFramePoint3D("icpPosition", ReferenceFrame.getWorldFrame(), registry);

      basePosition = new YoFramePoint3D("basePosition", ReferenceFrame.getWorldFrame(), registry);
      comBaseMidpoint = new YoFramePoint3D("comBaseMidpoint", ReferenceFrame.getWorldFrame(), registry);
      baseToCom = new YoFrameVector3D("baseToCom", ReferenceFrame.getWorldFrame(), registry);
      baseToComLength = new YoDouble("baseToComLength", registry);

      comPosition.set(0.0, 0.0, HEIGHT);
      basePosition.set(0.0, 0.0, 0.0);

      int numReferences = 1000;
      maxVelocityReferences = new double[numReferences];
      cruiseVelocityReferences = new double[numReferences];

      for (int i = 0; i < numReferences; i++)
      {
         maxVelocityReferences[i] = i * 2.0 / numReferences;
         cruiseVelocityReferences[i] = computeCruiseVelocity(ALPHA_STEP_LENGTH * maxVelocityReferences[i], maxVelocityReferences[i], OMEGA);
      }

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
      for (int i = 0; i < cruiseVelocityReferences.length; i++)
      {
         if (cruiseVelocityReferences[i] > Math.abs(desiredCruiseVelocity.getValue()))
         {
            desiredPeakVelocity.set(Math.signum(desiredCruiseVelocity.getValue()) * maxVelocityReferences[i]);
            break;
         }
      }

      double footSize = 0.02;
      baseToICP1D.set(icpPosition1D.getValue() - basePosition1D.getValue());
      if (Math.abs(desiredCruiseVelocity.getValue()) < 1.0e-3 && Math.abs(baseToICP1D.getValue()) < footSize)
      { // add small cop feedback
         double k = 0.1;
         double baseToCoP = EuclidCoreTools.clamp((1.0 + k) * baseToICP1D.getValue(), footSize);
         copPosition1D.set(basePosition1D.getValue() + baseToCoP);
      }
      else
      {
         copPosition1D.set(basePosition1D.getValue());
      }

      double baseToCoMX = comPosition1D.getValue() - copPosition1D.getValue();
      comAcceleration1D.set(baseToCoMX * square(OMEGA));

      if (baseToCoM() < 2.0)
      {
         comVelocity1D.add(dt * comAcceleration1D.getValue());
         comPosition1D.add(dt * comVelocity1D.getValue());
      }

      icpPosition1D.set(comPosition1D.getValue() + 1.0 / OMEGA * comVelocity1D.getValue());

//      if (baseToCoM() > 0.3)
//      { // take a step
//         basePosition1D.set(icpPosition1D.getValue());
//      }

      if (time.getValue() - lastStepTime.getValue() > MIN_STEP_TIME)
      { // consider taking a step
         boolean tookAStep = updateAndPossiblyTakeStep();
         if (tookAStep)
            lastStepTime.set(time.getValue());
      }

      comPosition.set(comPosition1D.getValue(), 0.0, HEIGHT);
      comVelocity.set(comVelocity1D.getValue(), 0.0, 0.0);
      basePosition.set(basePosition1D.getValue(), 0.0, 0.0);
      icpPosition.set(icpPosition1D.getValue(), 0.0, 0.0);
      baseToCom.sub(comPosition, basePosition);
      baseToComLength.set(baseToCom.norm());
      comBaseMidpoint.interpolate(comPosition, basePosition, 0.5);

      time.add(dt);
   }

   private boolean updateAndPossiblyTakeStep()
   {
      if (Math.abs(desiredCruiseVelocity.getValue()) < 1.0e-3)
      {
         basePosition1D.set(icpPosition1D.getValue());
         return true;
      }
      else if (baseToICP1D.getValue() * Math.signum(desiredCruiseVelocity.getValue()) <= 1.0e-7)
      {
         double additionalSteppingDistance = 0.03;
         basePosition1D.set(icpPosition1D.getValue() - additionalSteppingDistance * Math.signum(desiredCruiseVelocity.getValue()));
         return true;
      }
      else if (Math.abs(comVelocity1D.getValue()) > Math.abs(desiredPeakVelocity.getValue()))
      {
         double stepLag = desiredPeakVelocity.getValue() * (1.0 / OMEGA - 0.5 * ALPHA_STEP_LENGTH);
         basePosition1D.set(icpPosition1D.getValue() - stepLag);
         return true;
      }

      return false;
   }

   public YoDouble getDesiredCruiseVelocity()
   {
      return desiredCruiseVelocity;
   }

   private double baseToCoM()
   {
      return Math.abs(basePosition1D.getValue() - comPosition1D.getValue());
   }

   private static double computeCruiseVelocity(double stepLength, double peakVelocity, double omega)
   {
      double x0 = -0.5 * stepLength;
      double xd0 = peakVelocity;

      double a = 0.5 * (x0 + xd0 / omega);
      double b = x0;
      double c = 0.5 * (x0 - xd0 / omega);

      double x_quad = solveQuadratic(a, b, c);
      double dt = Math.log(x_quad) / omega;

      return stepLength / dt;
   }

   private static double solveQuadratic(double a, double b, double c)
   {
      return (-b + Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
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

   public static ControllerDefinition createControllerDefinition(double dt)
   {
      return (input, output) -> new LIPMWalker(input, output, dt);
   }

   public static void main(String[] args)
   {
      double desiredPeakVelocity = 0.121;
      double stepLength = ALPHA_STEP_LENGTH * desiredPeakVelocity;

      double cruiseVelocity = computeCruiseVelocity(stepLength, desiredPeakVelocity, OMEGA);

      System.out.println("desiredPeakVelocity = " + desiredPeakVelocity);
      System.out.println("cruiseVelocity = " + cruiseVelocity);

//      double a = 0.5 * (-0.5 * stepLength + peakVelocity / OMEGA);
//      double b = 0.5 * (-0.5 * stepLength - peakVelocity / OMEGA);
//      DoubleUnaryOperator xt = t -> a * Math.exp(OMEGA * t) + b * Math.exp(-OMEGA * t);
//      System.out.println("x(0) = " + xt.applyAsDouble(0.0));
//      System.out.println("x(0.672) = " + xt.applyAsDouble(0.672));

//      double t = 0.0;
//      double dt = 1.0e-3;
//      double x = -0.5 * stepLength;
//      double xd = peakVelocity;
//
//      for (int i = 0; i < 672; i++)
//      {
//         double xdd = OMEGA * OMEGA * x;
//         xd += dt * xdd;
//         x += dt * xd;
//      }
//
//      System.out.println("x = " + x);
//      System.out.println("xd = " + xd);
   }
}