package us.ihmc.mctslipmwalker.planners;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

import static us.ihmc.mctslipmwalker.planners.LIPMTools.computeTimeToReachVelocity;
import static us.ihmc.mctslipmwalker.planners.LIPMWalkerDesireds.computeStepLag;

public class MCTSOnlineSteppingStrategy implements SteppingStrategyInterface
{
   private final LIPMWalkerDesireds walkerDesireds;
   private final YoDouble nextStepTime;
   private final YoDouble nextStepLocation;

   public MCTSOnlineSteppingStrategy(LIPMWalkerDesireds walkerDesireds, YoRegistry registry)
   {
      this.walkerDesireds = walkerDesireds;
      this.nextStepTime = new YoDouble("nextStepTime", registry);
      this.nextStepLocation = new YoDouble("nextStepLocation", registry);

      nextStepTime.setToNaN();
      nextStepLocation.setToNaN();
   }

   @Override
   public double compute(double t, double x, double xd, double xb, double x_icp)
   {
      if (Math.abs(walkerDesireds.getDesiredCruiseVelocity()) < 1e-4)
      {
         return x_icp;
      }

      if (nextStepTime.isNaN())
      {
         return planNextStep(t, x, xd, xb, x_icp);
      }
      else if (t > nextStepTime.getValue())
      {
         double xbNew = nextStepLocation.getValue();
         return planNextStep(t, x, xd, xbNew, x_icp);
      }
      else
      {
         return xb;
      }
   }

   private double planNextStep(double t, double x, double xd, double xb, double x_icp)
   {
      try
      {
         MCTSWalkerPlanner planner = new MCTSWalkerPlanner(x, xd, xb, walkerDesireds);

         if (planner.plan())
         {
            List<MCTSWalkerNode> stepPlan = planner.getStepPlan();
            nextStepTime.set(t + stepPlan.get(0).getT());
            nextStepLocation.set(stepPlan.get(0).getXb());
         }
         else
         { // heuristic strategy

            if (Math.abs(walkerDesireds.getDesiredCruiseVelocity()) < 1.0e-3)
            {
               return x_icp;
            }

            double tStep = computeTimeToReachVelocity(x - xb, xd, walkerDesireds.getDesiredPeakVelocity());
            if (Double.isNaN(tStep) || Double.isInfinite(tStep))
            {
               double additionalSteppingDistance = 0.03;
               nextStepLocation.set(x_icp - additionalSteppingDistance * Math.signum(walkerDesireds.getDesiredCruiseVelocity()));
               nextStepTime.set(t);
            }
            else
            {
               double desiredPeakVelocity = walkerDesireds.getDesiredPeakVelocity();
               double stepLag = computeStepLag(desiredPeakVelocity);
               nextStepLocation.set(x_icp - stepLag);
               nextStepTime.set(t + tStep);
            }
         }
      }
      catch (Exception e)
      {
         LogTools.info("inputs:");
         LogTools.info("x =  " + x + ";");
         LogTools.info("xd = " + xd + ";");
         LogTools.info("xb = " + xb + ";");
         LogTools.info("xc = " + walkerDesireds.getDesiredCruiseVelocity() + ";");

         System.out.flush();
         e.printStackTrace();

         System.exit(0);
      }

      return xb;
   }

   public static void main(String[] args)
   {
      double x =  1.6765126320859327;
      double xd = 0.20392164172054497;
      double xb = 1.72504302089579;
      double xc = 0.1;
      LIPMWalkerDesireds desireds = new LIPMWalkerDesireds(xc);
      desireds.update();

      MCTSWalkerPlanner planner = new MCTSWalkerPlanner(x, xd, xb, desireds);
      planner.plan();
      planner.getStepPlan();
   }
}
