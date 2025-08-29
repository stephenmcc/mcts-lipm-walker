package us.ihmc.mctslipmwalker;

import static us.ihmc.mctslipmwalker.LIPMWalker.ALPHA_STEP_LENGTH;
import static us.ihmc.mctslipmwalker.LIPMWalker.OMEGA;

public class LIPMWalkerVelocityHelper
{
   private final double[] maxVelocityReferences;
   private final double[] stepTimes;
   private final double[] cruiseVelocityReferences;

   public LIPMWalkerVelocityHelper()
   {
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
   }

   public double getPeakVelocityFromCruiseVelocity(double desiredCruiseVelocity)
   {
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

   private static double computeCycleDuration(double stepLength, double peakVelocity)
   {
      double x0 = -0.5 * stepLength;
      double xd0 = peakVelocity;

      double a = 0.5 * (x0 + xd0 / OMEGA);
      double b = x0;
      double c = 0.5 * (x0 - xd0 / OMEGA);

      double x_quad = solveQuadratic(a, b, c);
      return Math.log(x_quad) / OMEGA;
   }

   public static double computeTimeToReachVelocity(double x0, double xd0, double xd1)
   {
      double a = 0.5 * (x0 * OMEGA + xd0);
      double b = - xd1;
      double c = -0.5 * (x0 * OMEGA - xd0);
      double x_quad = solveQuadratic(a, b, c);

      return Math.log(x_quad) / OMEGA;
   }

   public static double computePositionAtTime(double x0, double xd0, double t)
   {
      double a = 0.5 * (x0 + xd0 / OMEGA);
      double b = 0.5 * (x0 - xd0 / OMEGA);
      return a * Math.exp(OMEGA * t) + b * Math.exp(-OMEGA * t);
   }

   public static double computeVelocityAtTime(double x0, double xd0, double t)
   {
      double a = 0.5 * (OMEGA * x0 + xd0);
      double b = -0.5 * (OMEGA * x0 - xd0);
      return a * Math.exp(OMEGA * t) + b * Math.exp(-OMEGA * t);
   }

   public static double computeICPAtTime(double x0, double xd0, double t)
   {
      double x = computePositionAtTime(x0, xd0, t);
      double xd = computeVelocityAtTime(x0, xd0, t);
      return x + xd / OMEGA;
   }

   private static double solveQuadratic(double a, double b, double c)
   {
      return (-b + Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
   }

   public static double computeStepLag(double desiredPeakVelocity)
   {
      return desiredPeakVelocity * (1.0 / OMEGA - 0.5 * ALPHA_STEP_LENGTH);
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
