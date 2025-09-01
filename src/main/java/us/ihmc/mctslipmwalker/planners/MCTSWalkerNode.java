package us.ihmc.mctslipmwalker.planners;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mctslipmwalker.simulation.GappedTerrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.mctslipmwalker.planners.LIPMTools.*;
import static us.ihmc.mctslipmwalker.planners.MCTSWalkerActions.*;
import static us.ihmc.mctslipmwalker.planners.MCTSWalkerPlanner.MAX_SEARCH_DEPTH;
import static us.ihmc.mctslipmwalker.simulation.LIPMWalker.MIN_STEP_TIME;
import static us.ihmc.mctslipmwalker.simulation.LIPMWalker.OMEGA;

public class MCTSWalkerNode
{
   static int maxDepth = 0;

   private final MCTSWalkerNode parent;
   private final List<MCTSWalkerNode> children = new ArrayList<>();

   private final double x;
   private final double xd;
   private final double xb;
   private final double t;

   private final LIPMWalkerDesireds walkerDesireds;
   private final TDoubleArrayList stepVelocities = new TDoubleArrayList();

   private double q;
   private int n;
   private int depth;

   private final TIntArrayList untriedActions = new TIntArrayList(MCTSWalkerActions.ALL_ACTIONS);

   public MCTSWalkerNode(MCTSWalkerNode parent, double x, double xd, double xb, double t, LIPMWalkerDesireds walkerDesireds, int depth)
   {
      this.parent = parent;
      this.x = x;
      this.xd = xd;
      this.xb = xb;
      this.t = t;
      this.depth = depth;

      this.walkerDesireds = walkerDesireds;
   }

   public boolean isValidNode()
   {
      double xICP = x + xd / OMEGA;
      if (Math.abs(xICP - xb) > 0.3)
         return false;
      for (int i = 0; i < GappedTerrain.TERRAIN.size() - 1; i++)
      {
         if (MathTools.intervalContains(xb, GappedTerrain.TERRAIN.get(i).getLeft(), GappedTerrain.TERRAIN.get(i).getRight()))
         {
            return true;
         }
      }
      return false;
   }

   public double computeScore()
   {
      double averageVelocityScore = computeAverageVelocityScore();

      stepVelocities.clear();
      populateStepVelocities(stepVelocities);

      double backStepPenalty = 0.0;
      for (int i = 0; i < stepVelocities.size(); i++)
      {
         if (stepVelocities.get(i) * Math.signum(walkerDesireds.getDesiredCruiseVelocity()) < 0.0)
            backStepPenalty -= 5.0; // com moving wrong way, penalize
      }

      double velocityVariancePenalty = 0.0;
      double stepVelocityAvg = stepVelocities.sum() / stepVelocities.size();
      for (int i = 0; i < stepVelocities.size(); i++)
      {
         velocityVariancePenalty += EuclidCoreTools.square(stepVelocities.get(i) - stepVelocityAvg);
      }
      velocityVariancePenalty = -0.5 * Math.sqrt(velocityVariancePenalty / stepVelocities.size());

      return averageVelocityScore + backStepPenalty + velocityVariancePenalty;
   }

   public MCTSWalkerNode expand()
   {
      maxDepth = Math.max(maxDepth, depth + 1);

      if (EuclidCoreTools.epsilonEquals(x, xb, 1e-6) && EuclidCoreTools.epsilonEquals(xd, 0.0, 1e-6))
      {
         double xStep = 0.03;
         double dt = MIN_STEP_TIME;
         MCTSWalkerNode child = generateChild(xb - xStep * Math.signum(walkerDesireds.getDesiredCruiseVelocity()), dt);
         children.add(child);
         untriedActions.clear();
         return child;
      }
      else
      {
         while (!untriedActions.isEmpty())
         { // try to expand until a valid action is found
            int randomAction = untriedActions.removeAt(random.nextInt(untriedActions.size()));
            MCTSWalkerNode child = generateNodeFromAction(randomAction);

            if (child.isValidNode())
            {
               children.add(child);
               return child;
            }
         }

         if (children.isEmpty() && parent != null)
         {
            parent.children.remove(this);
         }

         return null;
      }
   }

   private MCTSWalkerNode generateNodeFromAction(int action)
   {
      double tNominal = computeTimeToReachVelocity(x - xb, xd, walkerDesireds.getDesiredPeakVelocity());
      if (tNominal < 0.0)
         tNominal = MIN_STEP_TIME; // min step time

      double stepLag = walkerDesireds.getStepLag();
      double t = tNominal * toDTScaleFactor(action);
      double icp = xb + computeICPAtTime(x - xb, xd, t);
      double xb = icp - toICPScaleFactor(action) * stepLag * Math.signum(walkerDesireds.getDesiredCruiseVelocity());
      return generateChild(xb, t);
   }

   public boolean isFullyExpanded()
   {
      return untriedActions.isEmpty();
   }

   public boolean isTerminalNode()
   {
      return depth >= MAX_SEARCH_DEPTH;
   }

   private MCTSWalkerNode generateChild(double xb, double dt)
   {
      double x = this.xb + computePositionAtTime(this.x - this.xb, this.xd, dt);
      double xd = computeVelocityAtTime(this.x - this.xb, this.xd, dt);
      return new MCTSWalkerNode(this, x, xd, xb, t + dt, walkerDesireds, depth + 1);
   }

   private double computeAverageVelocityScore()
   {
      double vAverage = computeAverageVelocityFromStart();
      return -EuclidCoreTools.square(vAverage - walkerDesireds.getDesiredCruiseVelocity());
   }

   private void populateStepVelocities(TDoubleArrayList stepVelocities)
   {
      if (parent != null)
      {
         stepVelocities.add((x - parent.x) / t);
         parent.populateStepVelocities(stepVelocities);
      }
   }

   private double computeAverageVelocityFromStart()
   {
      double distanceFromStart = x - getRoot().x;
      double timeFromStart = t - getRoot().t;
      return distanceFromStart / timeFromStart;
   }

   private MCTSWalkerNode getRoot()
   {
      return parent == null ? this : parent.getRoot();
   }

   public void backPropagate(double score)
   {
      n++;
      q += score;

      if (parent != null)
      {
         parent.backPropagate(score);
      }
   }

   public MCTSWalkerNode getBestChildUCB(double alphaExplore)
   {
      MCTSWalkerNode bestChild = null;
      double bestScore = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < children.size(); i++)
      {
         MCTSWalkerNode child = children.get(i);

         double ucb = (child.q / child.n) + alphaExplore * Math.sqrt(Math.log(n) / child.n);

         if (ucb > bestScore)
         {
            bestScore = ucb;
            bestChild = child;
         }
      }

      return bestChild;
   }

   private static final Random random = new Random(32);

   public double doRollout()
   {
      if (depth == MAX_SEARCH_DEPTH)
      {
         return computeScore();
      }
      else
      {
         // generate random action
         int maxAttempts = 4;
         for (int i = 0; i < maxAttempts; i++)
         {
            int randomAction = random.nextInt(NUMBER_OF_ACTIONS);
            MCTSWalkerNode child = generateNodeFromAction(randomAction);

            if (child.isValidNode())
            {
               return child.doRollout();
            }
         }

         return -2.0;
      }
   }

   public double getT()
   {
      return t;
   }

   public double getXb()
   {
      return xb;
   }

   public double getXd()
   {
      return xd;
   }

   public double getX()
   {
      return x;
   }

   @Override
   public String toString()
   {
      return "t = " + t + ", xb = " + xb + ", xd = " + xd + ", x = " + x + ", v_avg = " + (x / t);
   }
}
