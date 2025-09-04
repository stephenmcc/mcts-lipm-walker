package us.ihmc.mctslipmwalker.planners;

import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;

public class MCTSWalkerPlanner
{
   private static final boolean PRINT_TIMINGS = false;
   public static final int MAX_SEARCH_DEPTH = 5;
   private final MCTSWalkerNode rootNode;

   public MCTSWalkerPlanner(double x, double xd, double xb, LIPMWalkerDesireds walkerDesireds)
   {
      rootNode = new MCTSWalkerNode(null, x, xd, xb, 0.0, walkerDesireds, 0);
   }

   public boolean plan()
   {
      long t0 = System.nanoTime();

      for (int i = 0; i < 30000; i++)
      {
         doIteration();
      }

      long t1 = System.nanoTime();

      if (PRINT_TIMINGS)
         LogTools.info("Plan time: " + (t1 - t0) / 1000000 + " ms");

      return rootNode.getNumberOfChildren() > 0;
   }

   public List<MCTSWalkerNode> getStepPlan()
   {
      List<MCTSWalkerNode> nodes = new ArrayList<>();
      MCTSWalkerNode bestChild = rootNode.getBestChildUCB(0.0);
      if (bestChild != null)
         packSolution(nodes, bestChild);
      return nodes;
   }

   private static void packSolution(List<MCTSWalkerNode> nodes, MCTSWalkerNode nodeToPack)
   {
      nodes.add(nodeToPack);

      // done planning, time to exploit!
      double alphaExplore = 0.0;
      nodeToPack = nodeToPack.getBestChildUCB(alphaExplore);

      if (nodeToPack != null)
         packSolution(nodes, nodeToPack);
   }

   private void doIteration()
   {
      // Selection -- use tree policy to select best child node
      MCTSWalkerNode rolloutNode = getNextNodeFromTreePolicy();
      if (rolloutNode == null)
         return;

      // Perform rollout to max depth
      double score = rolloutNode.doRollout();

      // Backpropogate score
      rolloutNode.backPropagate(score);
   }

   private MCTSWalkerNode getNextNodeFromTreePolicy()
   {
      MCTSWalkerNode currentNode = rootNode;

      if (rootNode.isFullyExpanded() && rootNode.getNumberOfChildren() == 0)
      { // planner failed
         return null;
      }

      while (!currentNode.isTerminalNode())
      {
         if (currentNode.isFullyExpanded())
         {
            double alphaExplore = 1.0;
            currentNode = currentNode.getBestChildUCB(alphaExplore);
         }
         else
         {
            return currentNode.expand();
         }
      }

      return currentNode;
   }
}
