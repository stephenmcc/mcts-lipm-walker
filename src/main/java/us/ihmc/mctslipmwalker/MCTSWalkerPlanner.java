package us.ihmc.mctslipmwalker;

import java.util.ArrayList;
import java.util.List;

public class MCTSWalkerPlanner
{
   public static final int MAX_SEARCH_DEPTH = 6;
   private final MCTSWalkerNode rootNode;
   private int iteration;

   public MCTSWalkerPlanner(double x, double xd, double xb, double desiredCruiseVelocity, double desiredPeakVelocity, double nominalStepDuration)
   {
      LIPMWalkerDesireds walkerDesireds = new LIPMWalkerDesireds(desiredCruiseVelocity, desiredPeakVelocity, nominalStepDuration);
      rootNode = new MCTSWalkerNode(null, x, xd, xb, 0.0, walkerDesireds, 0);
   }

   public void plan()
   {
      long t0 = System.nanoTime();

      for (int i = 0; i < 200000; i++)
      {
         doIteration();
      }

      long t1 = System.nanoTime();
      System.out.println("Plan time: " + (t1 - t0) / 1000000 + " ms");

      List<MCTSWalkerNode> nodes = new ArrayList<>();
      packOptimalSolution(nodes, rootNode);

      for (int i = 0; i < nodes.size(); i++)
      {
         System.out.println(nodes.get(i));
      }
   }

   private static void packOptimalSolution(List<MCTSWalkerNode> nodes, MCTSWalkerNode nodeToPack)
   {
      nodes.add(nodeToPack);
      nodeToPack = nodeToPack.getBestChildUCB(0.0);

      if (nodeToPack != null)
         packOptimalSolution(nodes, nodeToPack);
   }

   private void doIteration()
   {
      // Selection -- use tree policy to select best child node
      MCTSWalkerNode rolloutNode = getNextNodeFromTreePolicy();

      if (rolloutNode != null)
      {
         // Perform rollout to max depth
         double score = rolloutNode.doRollout();

         // Backpropogate score
         rolloutNode.backPropagate(score);
      }

      iteration++;
   }

   private MCTSWalkerNode getNextNodeFromTreePolicy()
   {
      MCTSWalkerNode currentNode = rootNode;

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

   public static void main(String[] args)
   {
      double x = 0.0;
      double xd = 0.0;
      double xb = 0.0;

      LIPMWalkerVelocityHelper velocityHelper = new LIPMWalkerVelocityHelper();
      double xCruise = 0.2;
      double peakVelocity = velocityHelper.getPeakVelocityFromCruiseVelocity(xCruise);
      double stepTime = velocityHelper.getStepTimeFromCruiseVelocity(xCruise);

      MCTSWalkerPlanner planner = new MCTSWalkerPlanner(x, xd, xb, xCruise, peakVelocity, stepTime);
      planner.plan();

      System.out.println("");
//      List<MCTSWalkerNode> optimalPath = new ArrayList<>();
//      buildOptimalPath(optimalPath, planner.rootNode);

   }
}
