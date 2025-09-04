package us.ihmc.mctslipmwalker.planners;

public class MCTSWalkerActions
{
   /* Actions when no direction change is needed */
   private static final double[] NOMINAL_CHILD_DT_SCALE_FACTORS = new double[] {0.5, 0.7, 0.85, 1.0, 1.15, 1.3};
   private static final double[] NOMINAL_CHILD_ICP_SCALE_FACTORS = new double[] {0.25, 0.6, 1.0, 1.25, 1.5, 1.8};
   public static final int NOMINAL_NUMBER_OF_ACTIONS = NOMINAL_CHILD_DT_SCALE_FACTORS.length * NOMINAL_CHILD_ICP_SCALE_FACTORS.length;
   public static final int[] NOMINAL_ALL_ACTIONS = new int[NOMINAL_NUMBER_OF_ACTIONS];

   /* Actions when changing direction */
   private static final double[] CHANGING_DIRECTION_CHILD_ICP_SCALE_FACTORS = new double[] {0.3, 0.6, 0.85, 1.0, 1.1, 1.25, 1.6, 2.5, 4.0, 6.0, 8.0, 10.0};
   public static final int CHANGING_DIRECTION_NUMBER_OF_ACTIONS = CHANGING_DIRECTION_CHILD_ICP_SCALE_FACTORS.length;
   public static final int[] CHANGING_DIRECTION_ACTIONS = new int[CHANGING_DIRECTION_NUMBER_OF_ACTIONS];

   static
   {
      for (int i = 0; i < NOMINAL_NUMBER_OF_ACTIONS; i++)
      {
         NOMINAL_ALL_ACTIONS[i] = i;
      }

      for (int i = 0; i < CHANGING_DIRECTION_NUMBER_OF_ACTIONS; i++)
      {
         CHANGING_DIRECTION_ACTIONS[i] = i;
      }
   }

   public static double toDTScaleFactor(int action)
   {
      return NOMINAL_CHILD_DT_SCALE_FACTORS[action % NOMINAL_CHILD_DT_SCALE_FACTORS.length];
   }

   public static double toICPScaleFactor(int action)
   {
      return NOMINAL_CHILD_ICP_SCALE_FACTORS[action / NOMINAL_CHILD_DT_SCALE_FACTORS.length];
   }

   public static double toICPScaleFactorChangingDirection(int action)
   {
      return CHANGING_DIRECTION_CHILD_ICP_SCALE_FACTORS[action];
   }
}
