package us.ihmc.mctslipmwalker.planners;

public class MCTSWalkerActions
{
   private static final double[] CHILD_DT_SCALE_FACTORS = new double[] {0.5, 0.7, 0.85, 1.0, 1.15, 1.3, 1.5};
   private static final double[] CHILD_ICP_SCALE_FACTORS = new double[] {0.25, 0.5, 0.75, 1.0, 1.25, 1.4, 1.6, 1.8};
   public static final int NUMBER_OF_ACTIONS = CHILD_DT_SCALE_FACTORS.length * CHILD_ICP_SCALE_FACTORS.length;
   public static final int[] ALL_ACTIONS = new int[NUMBER_OF_ACTIONS];

   static
   {
      for (int i = 0; i < NUMBER_OF_ACTIONS; i++)
      {
         ALL_ACTIONS[i] = i;
      }
   }

   public static double toDTScaleFactor(int action)
   {
      return CHILD_DT_SCALE_FACTORS[action % CHILD_DT_SCALE_FACTORS.length];
   }

   public static double toICPScaleFactor(int action)
   {
      return CHILD_ICP_SCALE_FACTORS[action / CHILD_ICP_SCALE_FACTORS.length];
   }
}
