package us.ihmc.mctslipmwalker.planners;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class MCTSReplaySteppingStrategy implements SteppingStrategyInterface
{
   private final LIPMWalkerDesireds walkerDesireds;
   private final List<MCTSWalkerNode> stepPlan;
   private YoInteger nextStepIndex;

   public MCTSReplaySteppingStrategy(LIPMWalkerDesireds walkerDesireds, YoRegistry registry)
   {
      this.walkerDesireds = walkerDesireds;

      MCTSWalkerPlanner planner = new MCTSWalkerPlanner(0.0, 0.0, 0.0, walkerDesireds);
      planner.plan();
      stepPlan = planner.getStepPlan();

      nextStepIndex = new YoInteger("nextStepIndex", registry);
   }

   @Override
   public double compute(double t, double x, double xd, double xb, double x_icp)
   {
      // play back of the stepping plan

      if (isDone())
      {
         return x_icp;
      }

      if (t > stepPlan.get(nextStepIndex.getValue()).getT())
      {
         double step = stepPlan.get(nextStepIndex.getValue()).getXb();
         nextStepIndex.increment();
         return step;
      }

      if (isDone())
      {
         walkerDesireds.setDesiredCruiseVelocity(0.0);
      }

      return xb;
   }

   private boolean isDone()
   {
      return nextStepIndex.getValue() >= stepPlan.size();
   }
}
