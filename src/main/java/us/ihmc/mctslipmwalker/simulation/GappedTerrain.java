package us.ihmc.mctslipmwalker.simulation;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicBox3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

public class GappedTerrain
{
   private static final Random random = new Random(32983);
   public static final List<Pair<Double, Double>> TERRAIN = new ArrayList<>();

   static
   {
      double middleBlockSize = 0.2;
      double xNeg = -0.5 * middleBlockSize;
      double xPos = 0.5 * middleBlockSize;
      TERRAIN.add(Pair.of(xNeg, xPos));

      int numBlocks = 30;

      double gapMin = 0.03;
      double gapMax = 0.09;
      double terrainMin = 0.02;
      double terrainMax = 0.04;

      for (int i = 0; i < numBlocks; i++)
      {
         {
            double gapPos = EuclidCoreRandomTools.nextDouble(random, gapMin, gapMax);
            double terrainPos = EuclidCoreRandomTools.nextDouble(random, terrainMin, terrainMax);
            TERRAIN.add(Pair.of(xPos + gapPos, xPos + gapPos + terrainPos));
            xPos = xPos + gapPos + terrainPos;
         }

         {
            double gapNeg = EuclidCoreRandomTools.nextDouble(random, gapMin, gapMax);
            double terrainNeg = EuclidCoreRandomTools.nextDouble(random, terrainMin, terrainMax);
            TERRAIN.add(Pair.of(xNeg - gapNeg - terrainNeg, xNeg - gapNeg));
            xNeg = xNeg - gapNeg - terrainNeg;
         }
      }

      TERRAIN.sort(Comparator.comparingDouble(Pair::getLeft));
   }

   public static void addSCSGraphics(SimulationConstructionSet2 scs2)
   {
      for (int i = 0; i < TERRAIN.size(); i++)
      {
         YoGraphicBox3DDefinition groundBox = new YoGraphicBox3DDefinition();
         groundBox.setName("groundBox" + i);
         groundBox.setColor(ColorDefinitions.Green());

         double x0 = TERRAIN.get(i).getLeft();
         double x1 = TERRAIN.get(i).getRight();

         double xCenter = 0.5 * (x0 + x1);
         double xLength = x1 - x0;
         double height = 0.05;

         groundBox.setPosition(YoGraphicDefinitionFactory.newYoTuple3DDefinition(xCenter, 0.0, -0.5 * height, ReferenceFrame.getWorldFrame()));
         groundBox.setSize(YoGraphicDefinitionFactory.newYoTuple3DDefinition(xLength, 0.5, height, ReferenceFrame.getWorldFrame()));
         groundBox.setOrientation(YoGraphicDefinitionFactory.newYoOrientation3DDefinition(new Quaternion(), ReferenceFrame.getWorldFrame()));

         scs2.addYoGraphic(groundBox);
      }
   }
}
