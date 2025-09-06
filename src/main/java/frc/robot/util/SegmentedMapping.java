package frc.robot.util;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

// Provide the mapping of a source variable to a destination value
// e.g. (0.0, 0.0);
//      (1.0, 10.0);
//      (2.0, 30.0);
// get(1.5) will return 20.0;
//
// Just a wrapper to InterpolatingDoubleTreeMap for now, to see if
// the library does what we need.
public class SegmentedMapping {

    public void addPoint(double source, double destination)
    {
        mappingTree.put(source,destination);
    }

    public double mapPoint(double source)
    {
        // This should LERP between points 
        return mappingTree.get(source);
    }


  /* public double mapSourceToDest(double source)
    {
        if (source < sourceMap.firstElement())
        {
            return destMap.firstElement();
        }
        else if (source > sourceMap.lastElement())
        {
            return destMap.lastElement();
        }

        int startSrcIdx = -1;
        int endSrcIdx = -1;
        for (int srcIdx = 0; srcIdx < sourceMap.size(); ++srcIdx)
        {
            if (source >= sourceMap.elementAt(srcIdx))
            {
                startSrcIdx = srcIdx;
                endSrcIdx = srcIdx + 1;
                break;
            }
        }

        return 0.0;

    }*/

    private

        InterpolatingDoubleTreeMap mappingTree = new InterpolatingDoubleTreeMap();
}
