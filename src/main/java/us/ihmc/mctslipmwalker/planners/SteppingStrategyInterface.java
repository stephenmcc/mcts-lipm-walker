package us.ihmc.mctslipmwalker.planners;

public interface SteppingStrategyInterface
{
   /**
    * Computes new base position given state of the LIPM
    * @param t simulator time (sec)
    * @param x position of CoM
    * @param xd velocity of CoM
    * @param xb base position
    * @param x_icp ICP position
    * @return new base position, which can be same as current base position if no step is taken
    */
   double compute(double t, double x, double xd, double xb, double x_icp);
}
