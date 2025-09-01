package us.ihmc.mctslipmwalker.planners;

import static us.ihmc.mctslipmwalker.simulation.LIPMWalker.OMEGA;

public class LIPMTools
{
   public static double computeCycleDuration(double stepLength, double peakVelocity)
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
}
