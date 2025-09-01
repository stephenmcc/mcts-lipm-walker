package us.ihmc.mctslipmwalker.planners;

import java.util.ArrayList;
import java.util.List;

public class MCTSWalkerPlanner
{
   public static final int MAX_SEARCH_DEPTH = 7;
   private final MCTSWalkerNode rootNode;
   private int iteration;

   public MCTSWalkerPlanner(double x, double xd, double xb, LIPMWalkerDesireds walkerDesireds)
   {
      rootNode = new MCTSWalkerNode(null, x, xd, xb, 0.0, walkerDesireds, 0);
   }

   public void plan()
   {
      long t0 = System.nanoTime();

      for (int i = 0; i < 800000; i++)
      {
         doIteration();
      }

      long t1 = System.nanoTime();
      System.out.println("Plan time: " + (t1 - t0) / 1000000 + " ms");

      List<MCTSWalkerNode> stepPlan = getStepPlan();
      for (int i = 0; i < stepPlan.size(); i++)
      {
         System.out.println(stepPlan.get(i));
      }
   }

   public List<MCTSWalkerNode> getStepPlan()
   {
      List<MCTSWalkerNode> nodes = new ArrayList<>();
      packSolution(nodes, rootNode.getBestChildUCB(0.0));
      return nodes;
   }

   private static void packSolution(List<MCTSWalkerNode> nodes, MCTSWalkerNode nodeToPack)
   {
      nodes.add(nodeToPack);
      nodeToPack = nodeToPack.getBestChildUCB(0.0);

      if (nodeToPack != null)
         packSolution(nodes, nodeToPack);
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

      double desiredCruiseVelocity = 0.15;
      LIPMWalkerDesireds walkerDesireds = new LIPMWalkerDesireds(desiredCruiseVelocity);

      MCTSWalkerPlanner planner = new MCTSWalkerPlanner(x, xd, xb, walkerDesireds);
      planner.plan();

      System.out.println("");
//      List<MCTSWalkerNode> solution = new ArrayList<>();
//      packSolution(solution, planner.rootNode);

   }
}
