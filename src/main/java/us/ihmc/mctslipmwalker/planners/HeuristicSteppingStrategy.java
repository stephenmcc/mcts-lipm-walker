package us.ihmc.mctslipmwalker.planners;

import static us.ihmc.mctslipmwalker.planners.LIPMWalkerDesireds.computeStepLag;

public class HeuristicSteppingStrategy implements SteppingStrategyInterface
{
   private final LIPMWalkerDesireds walkerDesireds;

   public HeuristicSteppingStrategy(LIPMWalkerDesireds walkerDesireds)
   {
      this.walkerDesireds = walkerDesireds;
   }

   public double compute(double t, double x, double xd, double xb, double x_icp)
   {
      if (Math.abs(walkerDesireds.getDesiredCruiseVelocity()) < 1.0e-3)
      {
         return x_icp;
      }

      double baseToICP = x_icp - xb;
      if (baseToICP * Math.signum(walkerDesireds.getDesiredCruiseVelocity()) <= 1.0e-7)
      {
         double additionalSteppingDistance = 0.03;
         return x_icp - additionalSteppingDistance * Math.signum(walkerDesireds.getDesiredCruiseVelocity());
      }

      if (Math.abs(xd) > Math.abs(walkerDesireds.getDesiredPeakVelocity()))
      {
         double desiredPeakVelocity = walkerDesireds.getDesiredPeakVelocity();
         double stepLag = computeStepLag(desiredPeakVelocity);
         return x_icp - stepLag;
      }

      return xb;
   }
}
