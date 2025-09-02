package us.ihmc.mctslipmwalker.planners;

import us.ihmc.yoVariables.registry.YoRegistry;

public enum SteppingStrategy
{
   HEURISTIC,
   MCTS_REPLAY,
   MCTS_ONLINE;

   public SteppingStrategyInterface build(LIPMWalkerDesireds walkerDesireds, YoRegistry registry)
   {
      switch (this)
      {
         case HEURISTIC:
            return new HeuristicSteppingStrategy(walkerDesireds);
         case MCTS_REPLAY:
            double desiredCruiseVelocity = 0.1;
            walkerDesireds.setDesiredCruiseVelocity(desiredCruiseVelocity);
            walkerDesireds.update();
            return new MCTSReplaySteppingStrategy(walkerDesireds, registry);
         case MCTS_ONLINE:
            return new MCTSOnlineSteppingStrategy(walkerDesireds, registry);
      }

      throw new RuntimeException("Implement me!");
   }
}
