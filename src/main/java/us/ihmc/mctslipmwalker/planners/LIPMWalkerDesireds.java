package us.ihmc.mctslipmwalker.planners;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.mctslipmwalker.planners.LIPMTools.computeCycleDuration;
import static us.ihmc.mctslipmwalker.planners.LIPMTools.computeTimeToReachVelocity;
import static us.ihmc.mctslipmwalker.simulation.LIPMWalker.ALPHA_STEP_LENGTH;
import static us.ihmc.mctslipmwalker.simulation.LIPMWalker.OMEGA;

public class LIPMWalkerDesireds
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble desiredCruiseVelocity = new YoDouble("desiredCruiseVelocity", registry);
   private final YoDouble desiredPeakVelocity = new YoDouble("desiredPeakVelocity", registry);
   private final YoDouble nominalStepDuration = new YoDouble("nominalStepDuration", registry);

   private final double[] maxVelocityReferences;
   private final double[] stepTimes;
   private final double[] cruiseVelocityReferences;

   public LIPMWalkerDesireds()
   {
      this(0.0);
   }

   public LIPMWalkerDesireds(double desiredCruiseVelocity)
   {
      this.desiredCruiseVelocity.set(desiredCruiseVelocity);

      int numReferences = 1000;
      maxVelocityReferences = new double[numReferences];
      stepTimes = new double[numReferences];
      cruiseVelocityReferences = new double[numReferences];

      for (int i = 1; i < numReferences; i++)
      {
         maxVelocityReferences[i] = i * 2.0 / numReferences;

         double stepLength = ALPHA_STEP_LENGTH * maxVelocityReferences[i];
         stepTimes[i] = computeCycleDuration(stepLength, maxVelocityReferences[i]);
         cruiseVelocityReferences[i] = stepLength / stepTimes[i];
      }

      update();
   }

   public void update()
   {
      this.desiredPeakVelocity.set(getPeakVelocityFromCruiseVelocity(desiredCruiseVelocity.getValue()));
      this.nominalStepDuration.set(computeCycleDuration(ALPHA_STEP_LENGTH * desiredPeakVelocity.getValue(), desiredPeakVelocity.getValue()));
   }

   public double getDesiredCruiseVelocity()
   {
      return desiredCruiseVelocity.getValue();
   }

   public double getDesiredPeakVelocity()
   {
      return desiredPeakVelocity.getValue();
   }

   public double getNominalStepDuration()
   {
      return nominalStepDuration.getValue();
   }

   public void setDesiredCruiseVelocity(double desiredCruiseVelocity)
   {
      this.desiredCruiseVelocity.set(desiredCruiseVelocity);
   }

   public void setDesiredPeakVelocity(double desiredPeakVelocity)
   {
      this.desiredPeakVelocity.set(desiredPeakVelocity);
   }

   public void setNominalStepDuration(double nominalStepDuration)
   {
      this.nominalStepDuration.set(nominalStepDuration);
   }

   public double getNominalStepLength()
   {
      return ALPHA_STEP_LENGTH * desiredPeakVelocity.getValue();
   }

   public double getStepLag()
   {
      return computeStepLag(desiredPeakVelocity.getValue());
   }

   public double getPeakVelocityFromCruiseVelocity(double desiredCruiseVelocity)
   {
      if (Math.abs(desiredCruiseVelocity) < 1e-7)
         return 0.0;

      for (int i = 0; i < cruiseVelocityReferences.length; i++)
      {
         if (cruiseVelocityReferences[i] > Math.abs(desiredCruiseVelocity))
         {
            return Math.signum(desiredCruiseVelocity) * maxVelocityReferences[i];
         }
      }

      return Math.signum(desiredCruiseVelocity * maxVelocityReferences[maxVelocityReferences.length - 1]);
   }

   public double getStepTimeFromCruiseVelocity(double desiredCruiseVelocity)
   {
      for (int i = 0; i < cruiseVelocityReferences.length; i++)
      {
         if (cruiseVelocityReferences[i] > Math.abs(desiredCruiseVelocity))
         {
            return stepTimes[i];
         }
      }

      return stepTimes[stepTimes.length - 1];
   }

   public static double computeStepLag(double desiredPeakVelocity)
   {
      return desiredPeakVelocity * (1.0 / OMEGA - 0.5 * ALPHA_STEP_LENGTH);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public static void main(String[] args)
   {
      // should equal
      double t1 = computeCycleDuration(0.2, 0.4);
      double t2 = computeTimeToReachVelocity(-0.1, 0.4, 0.4);
      System.out.println(t1);
      System.out.println(t2);
   }
}
