#include "common/refline/reference_line_provider.h"

using namespace msquare;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");

  ReferenceLineProvider reflineProvider;

  int len = 45;

  double xxx[] = {-53.9549788953456, -33.7110413891053,  -21.6341883874536,
                  -16.6549822878697, -14.6507657765184,  -12.8532284916779,
                  -11.4244335524653, -10.0316420134322,  -3.64310035744568,
                  -2.04440760004453, -0.579603527701213, 1.21949181205985,
                  3.20881967752900,  5.40091015215013,   8.41580584875849,
                  28.8553669103606,  47.4528971657992,   49.1687944976378,
                  50.7409636765434,  52.2817468778448,   53.7998299822435,
                  55.2983514036098,  56.9173801298010,   58.5027598146807,
                  60.0898665876706,  61.6558542013540,   63.1928154253849,
                  64.6914316560423,  66.1878577044860,   67.5346770551522,
                  68.8447087060712,  70.1084150940332,   71.3221710409440,
                  72.4944434218040,  73.6205364176961,   74.6613027354383,
                  75.6425079796990,  76.5651497701332,   77.4157005046162,
                  78.1676158231211,  78.7188554769360,   79.2569070421613,
                  79.7098507305332,  79.8375427891483,   80.0201523107275,
                  80.3533221078866,  82.7451094503046};

  double yyy[] = {3.92571963309789,   1.48021600258355,   0.0269450718803672,
                  -0.277168388664794, -0.343888714558763, -0.372591732274671,
                  -0.353362837669947, -0.302005260923530, 0.0407378243375917,
                  0.0781946598999898, 0.0653009108002778, -0.0116290887842160,
                  -0.158742800117776, -0.357943192431937, -0.698527336503573,
                  -3.11373769385811,  -5.33740028462521,  -5.18294804183260,
                  -5.26799839958276,  -5.33102819395616,  -5.36795150878596,
                  -5.34058294853546,  -5.25690422444667,  -5.07880097704828,
                  -4.81416336160564,  -4.43700424518334,  -3.97552438107346,
                  -3.40957583242246,  -2.77116354760926,  -2.10480016191043,
                  -1.38219777812100,  -0.583387085838622, 0.277799443906523,
                  1.20526911816636,   2.20906611859694,   3.28312899454049,
                  4.43947741241039,   5.65417513143043,   6.93306133123656,
                  8.28823108833139,   9.70806483660788,   11.1835150141689,
                  12.7294094473181,   13.0825826801722,   14.0614354326082,
                  17.8402535990561,   38.2970894672306};

  std::vector<RefLinePoint> trajtory;
  std::vector<double> vx, vy;
  for (int i = 0; i < len; i++) {
    RefLinePoint pp;
    pp.x = xxx[i];
    pp.y = yyy[i];
    // pp.z = 0.0;
    trajtory.push_back(pp);
    vx.push_back(xxx[i]);
    vy.push_back(yyy[i]);
  }
  // reflineProvider.Init(trajtory);
  // reflineProvider.Printtraj();
  reflineProvider.SetTrajectoryPoints(trajtory);
  reflineProvider.GetReferenceLine(true);
  reflineProvider.Printtraj();

  // reflineProvider.GetReferenceLine(false);

  FrenetCoordinateSystemParameters frenet_parameters_;

  frenet_parameters_.zero_speed_threshold = 0.1;
  frenet_parameters_.coord_transform_precision = 0.01;
  frenet_parameters_.step_s = 0.3;
  frenet_parameters_.coarse_step_s = 1.0;
  frenet_parameters_.optimization_gamma = 0.5;
  frenet_parameters_.max_iter = 15;

  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  frenet_coord_.reset(new FrenetCoordinateSystem(
      reflineProvider.smoother_vx, reflineProvider.smoother_vy,
      frenet_parameters_, 0.0, 110.0));

  reflineProvider.SetFrenetCoorSystemPre(frenet_coord_);

  // navi
  // momenta_msgs::NaviFusion navi;
  // navi.pose.x = 0.0;
  // navi.pose.y = 0.0;
  // geometry_msgs::Pose2D navi;
  Pose2D navi;
  navi.x = 0.0;
  navi.y = 0.0;
  navi.theta = 0.0;

  reflineProvider.SetOrigin(navi);

  reflineProvider.GetReferenceLine(false);

  return 0;
}