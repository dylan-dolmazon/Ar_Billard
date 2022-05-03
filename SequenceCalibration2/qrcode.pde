
import boofcv.abst.distort.FDistort;
import boofcv.abst.geo.Estimate1ofEpipolar;
import boofcv.alg.distort.PointToPixelTransform_F32;
import boofcv.alg.distort.PointTransformHomography_F32;
import boofcv.alg.filter.blur.GBlurImageOps;
import boofcv.core.image.GConvertImage;
import boofcv.core.image.GeneralizedImageOps;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_F32;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.FMatrixRMaj;
import org.ejml.ops.ConvertMatrixData;
import processing.core.PConstants;
import processing.core.PImage;

import java.util.ArrayList;
import java.util.List;


  public PointTransformHomography_F32 searchHomography( 
                     double xi0, double yi0,
                     double xi1, double yi1,
                     double xi2, double yi2,
                     double xi3, double yi3,
                     double xo0, double yo0,
                     double xo1, double yo1,
                     double xo2, double yo2,
                     double xo3, double yo3 )
  {
    
    // Homography estimation algorithm.  Requires a minimum of 4 points
    Estimate1ofEpipolar computeHomography = FactoryMultiView.homographyDLT(true);

    // Specify the pixel coordinates from destination to target
    ArrayList<AssociatedPair> associatedPairs = new ArrayList<AssociatedPair>();
    associatedPairs.add(new AssociatedPair(new Point2D_F64(xo0,yo0),new Point2D_F64(xi0,yi0)));
    associatedPairs.add(new AssociatedPair(new Point2D_F64(xo1,yo1),new Point2D_F64(xi1,yi1)));
    associatedPairs.add(new AssociatedPair(new Point2D_F64(xo2,yo2),new Point2D_F64(xi2,yi2)));
    associatedPairs.add(new AssociatedPair(new Point2D_F64(xo3,yo3),new Point2D_F64(xi3,yi3)));

    // Compute the homography
    DMatrixRMaj H = new DMatrixRMaj(3,3);
    computeHomography.process(associatedPairs, H);
    FMatrixRMaj H32 = new FMatrixRMaj(3,3);
    ConvertMatrixData.convert(H,H32);

    // Create the transform for distorting the image
    PointTransformHomography_F32 homography = new PointTransformHomography_F32(H32);

    return homography;
  }
