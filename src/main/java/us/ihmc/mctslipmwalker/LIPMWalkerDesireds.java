package us.ihmc.mctslipmwalker;

import static us.ihmc.mctslipmwalker.LIPMWalker.ALPHA_STEP_LENGTH;
import static us.ihmc.mctslipmwalker.LIPMWalkerVelocityHelper.computeStepLag;

public class LIPMWalkerDesireds
{
   private final double desiredCruiseVelocity;
   private final double desiredPeakVelocity;
   private final double nominalStepDuration;

   public LIPMWalkerDesireds(double desiredCruiseVelocity, double desiredPeakVelocity, double nominalStepDuration)
   {
      this.desiredCruiseVelocity = desiredCruiseVelocity;
      this.desiredPeakVelocity = desiredPeakVelocity;
      this.nominalStepDuration = nominalStepDuration;
   }

   public double getNominalStepDuration()
   {
      return nominalStepDuration;
   }

   public double getDesiredPeakVelocity()
   {
      return desiredPeakVelocity;
   }

   public double getDesiredCruiseVelocity()
   {
      return desiredCruiseVelocity;
   }

   public double getNominalStepLength()
   {
      return ALPHA_STEP_LENGTH * desiredPeakVelocity;
   }

   public double getStepLag()
   {
      return computeStepLag(desiredPeakVelocity);
   }
}
