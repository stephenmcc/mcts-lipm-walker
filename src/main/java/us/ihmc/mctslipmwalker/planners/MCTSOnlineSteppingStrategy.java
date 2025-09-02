package us.ihmc.mctslipmwalker.planners;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

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
         return planNextStep(t, x, xd, xb);
      }
      else if (t > nextStepTime.getValue())
      {
         double xbNew = nextStepLocation.getValue();
         return planNextStep(t, x, xd, xbNew);
      }
      else
      {
         return xb;
      }
   }

   private double planNextStep(double t, double x, double xd, double xb)
   {
      try
      {
         MCTSWalkerPlanner planner = new MCTSWalkerPlanner(x, xd, xb, walkerDesireds);
         planner.plan();
         List<MCTSWalkerNode> stepPlan = planner.getStepPlan();
         nextStepTime.set(t + stepPlan.get(0).getT());
         nextStepLocation.set(stepPlan.get(0).getXb());
      }
      catch (Exception e)
      {
         System.out.println("inputs:");
         System.out.println("x:  " + x);
         System.out.println("xd: " + xd);
         System.out.println("xb: " + xb);
         System.out.println("xcruise: " + walkerDesireds.getDesiredCruiseVelocity());

         System.out.flush();
         System.exit(0);
      }

      return xb;
   }

   public static void main(String[] args)
   {
      double x = 0.5273799450025973;
      double xd = -0.003981355307947314;
      double xb = 0.54030039096677;
      double xcruise = -0.1;
      LIPMWalkerDesireds desireds = new LIPMWalkerDesireds(xcruise);
      desireds.update();

      MCTSWalkerPlanner planner = new MCTSWalkerPlanner(x, xd, xb, desireds);
      planner.plan();
   }
}
