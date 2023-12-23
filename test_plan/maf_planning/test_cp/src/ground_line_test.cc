#include "common/math/vec2d.h"
#include "planner/behavior_planner/deciders/ground_line_decider.h"
#include "pnc/define/parking_vision_info.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <vector>

namespace msquare {
namespace parking {

std::vector<std::vector<double>> data1{
    {9.71830273, -11.59004402},  {9.73864365, -11.46709538},
    {9.76457405, -11.31554127},  {9.80686569, -11.10130405},
    {9.85764408, -10.89496803},  {9.91677284, -10.5731802},
    {9.97374916, -10.50770092},  {10.02041149, -10.32876301},
    {10.07936573, -10.19957733}, {10.105093, -9.93623447},
    {10.18606377, -9.9912796},   {10.14842987, -9.58689594},
    {10.25384045, -9.68049049},  {10.20074272, -9.26810551},
    {10.29704857, -9.34709549},  {10.23828411, -8.98672104},
    {10.34621906, -9.07859612},  {10.29128933, -8.71251202},
    {10.39417744, -8.78415585},  {10.3367815, -8.46341133},
    {10.44678879, -8.52980995},  {10.38367844, -8.24022484},
    {10.4619627, -8.22565174},   {10.45991707, -8.09512329},
    {10.53197193, -8.06991386},  {10.43099308, -7.76549482},
    {10.48361588, -7.72467279},  {10.47060776, -7.59289789},
    {10.49323559, -7.48736763},  {10.50617027, -7.38105059},
    {10.53287029, -7.28258038},  {10.55343628, -7.18969107},
    {10.56020546, -7.08776712},  {10.56290054, -6.98475885},
    {10.54658985, -6.87366247},  {10.53052044, -6.76143932},
    {10.51568413, -6.6847229},   {10.51722431, -6.58994055},
    {10.54864693, -6.38171148},  {10.56483078, -6.29660416},
    {10.59105301, -6.15184879},  {10.60387421, -5.99749613},
    {10.61047363, -5.91164589},  {10.62485886, -5.74762011},
    {10.63117313, -5.66784954},  {10.64329052, -5.51331949},
    {10.64899445, -5.43833303},  {10.66032696, -5.27699232},
    {10.66505909, -5.20665216},  {10.66866112, -5.14763546},
    {10.67510319, -5.06507587},  {10.68390656, -4.98081398},
    {10.69124031, -4.89951134},  {10.69675446, -4.82677937},
    {10.69680882, -4.76232004},  {10.69868755, -4.69428778},
    {10.69808006, -4.63855696},  {10.69942379, -4.56413937},
    {10.69840145, -4.38825655},  {10.69704151, -4.31443882},
    {10.69687843, -4.24845839},  {10.69473171, -4.17019558},
    {10.69307518, -4.09983969},  {10.68852139, -4.05580711},
    {10.68287277, -3.99201798},  {10.6648941, -3.80260277},
    {10.63515377, -3.72924805},  {10.591362, -3.67657661},
    {10.53121567, -3.62904906},  {10.48483849, -3.57246113},
    {10.46355152, -3.40007472},  {10.46632576, -3.34240627},
    {10.46724987, -3.29000211},  {10.46547031, -3.10659885},
    {10.46402645, -3.04897785},  {10.46046638, -2.99674773},
    {10.44158173, -2.83823872},  {10.7932415, -2.46705079},
    {10.79216862, -2.41717935},  {10.78751469, -2.3503623},
    {10.78132915, -2.28300285},  {10.7616415, -2.19925857},
    {10.73654652, 3.85233974},   {10.68499851, 3.95557165},
    {10.62329578, 4.099051},     {10.57033539, 4.25696611},
    {10.53016663, 4.38473272},   {3.16738129, 4.87737083},
    {3.05394983, 4.85945559},    {2.90765381, 4.83438396},
    {2.69009328, 4.89326811},    {2.64122868, 4.79703569},
    {2.65813875, -2.5908947},    {2.64659333, -2.64733982},
    {2.63890696, -2.69967294},   {2.61726213, -2.85948253},
    {2.60950494, -2.93241453},   {2.60471916, -3.10238171},
    {2.60696435, -3.17934322},   {2.60902572, -3.2203052},
    {2.61215663, -3.26733088},   {1.80180657, -10.83167934},
    {1.88621783, -10.86732006},  {1.97761655, -10.91845226},
    {2.09346724, -10.98195457},  {2.20470381, -11.04700279},
    {2.29222655, -11.1666851},   {2.36882281, -11.13669109},
    {2.35509014, -11.45829678},  {0, 0, 0.0}};

std::vector<int> grounp1{77, 5, 5, 5, 9, 8, 1};

std::vector<std::vector<double>> data2{{-109.585533142, 33.0217514038},
                                       {-109.409500122, 33.0194206238},
                                       {-109.229072571, 33.0314674377},
                                       {-109.077033997, 33.0399627686},
                                       {-108.89755249, 33.0487365723},
                                       {-108.740791321, 33.019115448},
                                       {-108.552124023, 33.0069961548},
                                       {-108.378646851, 33.0289459229},
                                       {-108.184646606, 33.0270500183},
                                       {-108.019317627, 33.0161819458},
                                       {-107.821533203, 33.0190963745},
                                       {-107.618408203, 33.0305175781},
                                       {-107.433105469, 33.0563583374},
                                       {-107.212020874, 33.0504722595},
                                       {-107.011665344, 33.0461997986},
                                       {-106.791168213, 33.032043457},
                                       {-106.562278748, 33.0285301208},
                                       {-106.373321533, 32.9985847473},
                                       {-106.147239685, 32.9830436707},
                                       {-105.918144226, 32.9675674438},
                                       {-105.69203186, 32.9462013245},
                                       {-105.467674255, 32.8895225525},
                                       {-105.243019104, 32.8633575439},
                                       {-105.001022339, 32.8151817322},
                                       {-104.792419434, 32.7289161682},
                                       {-104.574455261, 32.6527824402},
                                       {-103.301589966, 20.359708786},
                                       {-103.739067078, 20.4062252045},
                                       {-104.083999634, 20.4382324219},
                                       {-104.435523987, 20.4758758545},
                                       {-104.792694092, 20.5344829559},
                                       {-105.115539551, 20.5822105408},
                                       {-105.441703796, 20.6123733521},
                                       {-105.748855591, 20.6631507874},
                                       {-105.994155884, 20.682970047},
                                       {-106.265449524, 20.710269928},
                                       {-106.471847534, 20.7400970459},
                                       {-106.73135376, 20.7750797272},
                                       {-106.946754456, 20.8130626678},
                                       {-107.161903381, 20.8294696808},
                                       {-107.392547607, 20.8695259094},
                                       {-107.579490662, 20.8915939331},
                                       {-107.780204773, 20.9213104248},
                                       {-107.977363586, 20.9605808258},
                                       {-108.128196716, 20.9564037323},
                                       {-107.39099884, 29.117307663},
                                       {-107.295654297, 29.0457572937},
                                       {-107.186454773, 29.0057163239},
                                       {-107.095481873, 28.9533424377},
                                       {-106.972579956, 28.8738384247},
                                       {-106.884185791, 28.8113880157},
                                       {-106.793258667, 28.7445411682},
                                       {-107.530265808, 29.6998386383},
                                       {-107.470504761, 29.6344490051},
                                       {-107.416191101, 29.5023937225},
                                       {-107.397102356, 29.3751354218},
                                       {-107.381126404, 29.2605285645},
                                       {-107.32875061, 29.1566181183},
                                       {-108.191368103, 21.0578041077},
                                       {-108.287811279, 20.9435062408},
                                       {-108.416572571, 20.7976551056},
                                       {-108.546356201, 20.7644844055},
                                       {-116.295433044, 22.087720871},
                                       {-116.423843384, 22.1449069977},
                                       {-116.619720459, 22.1896324158},
                                       {-116.857177734, 22.2356338501},
                                       {-116.995895386, 22.2950305939},
                                       {-103.156265259, 20.7463970184},
                                       {-103.596038818, 20.7242641449},
                                       {-104.003967285, 20.6953258514},
                                       {-104.582244873, 20.7616500854},
                                       {-104.978408813, 20.7923469543},
                                       {-105.425537109, 20.8565883636},
                                       {-105.791069031, 20.8952236176},
                                       {-106.192749023, 20.9381427765},
                                       {-106.464561462, 20.9425430298},
                                       {-106.688468933, 20.9380893707},
                                       {-106.962837219, 20.9510250092},
                                       {-107.244781494, 20.9954986572},
                                       {-107.51272583, 21.0424594879},
                                       {-107.746276855, 21.0571517944},
                                       {-107.953811646, 21.0444507599},
                                       {-100.958778381, 28.4552974701},
                                       {-100.906394958, 28.1888828278},
                                       {-100.828964233, 27.9256591797},
                                       {-100.841094971, 27.6410827637},
                                       {-100.727928162, 27.3562488556},
                                       {-107.995872498, 21.1211414337},
                                       {-108.152336121, 21.0624065399},
                                       {-108.277679443, 21.0123233795},
                                       {-108.402160645, 20.9198055267},
                                       {-108.471839905, 20.8049182892},
                                       {-108.562698364, 20.6701526642},
                                       {-122.689674377, 34.2079658508},
                                       {-122.337173462, 34.2460517883},
                                       {-122.038246155, 34.2860565186},
                                       {-121.759056091, 34.3512916565},
                                       {-121.505508423, 34.454372406},
                                       {-121.237121582, 34.5364227295},
                                       {-122.763694763, 30.3230876923},
                                       {-122.60836792, 30.4714126587},
                                       {-122.532806396, 30.6681442261},
                                       {-122.469703674, 30.9012584686},
                                       {-122.369369507, 31.1032905579},
                                       {-115.954391479, 22.0265197754},
                                       {-116.060554504, 22.0159568787},
                                       {-116.152580261, 22.031452179},
                                       {-116.213165283, 22.0648403168},
                                       {-116.29775238, 22.1107234955},
                                       {-116.391410828, 22.1106967926},
                                       {-116.474090576, 22.1132011414},
                                       {-116.570167542, 22.1110286713},
                                       {-116.663284302, 22.1195240021},
                                       {-116.732536316, 22.1693973541},
                                       {-116.812637329, 22.2098560333},
                                       {-124.257484436, 22.5028305054},
                                       {-124.345451355, 22.7268543243},
                                       {-124.418289185, 22.9232254028},
                                       {-124.446105957, 23.1652431488},
                                       {-124.591171265, 23.3754711151},
                                       {-115.078781128, 29.9297237396},
                                       {-115.02520752, 29.98097229},
                                       {-114.967445374, 30.0619487762},
                                       {-114.915023804, 30.1680335999},
                                       {-114.881401062, 30.2726917267},
                                       {-114.845207214, 30.3742008209},
                                       {-114.796211243, 30.5028762817},
                                       {-105.056343079, 32.8866767883},
                                       {-104.814704895, 32.8008232117},
                                       {-104.523338318, 32.7205162048},
                                       {-104.289321899, 32.6135749817},
                                       {-103.942169189, 32.5328483582},
                                       {-103.645462036, 32.440826416},
                                       {-103.453422546, 32.2552223206},
                                       {-103.405433655, 32.0000801086},
                                       {-115.625587463, 29.7229366302},
                                       {-115.541244507, 29.7767314911},
                                       {-115.426002502, 29.8100643158},
                                       {-115.295669556, 29.8336429596},
                                       {-115.196350098, 29.8645114899},
                                       {-115.088432312, 29.925699234},
                                       {-115.030685425, 29.9787387848},
                                       {-115.772827148, 33.1793632507},
                                       {-115.664527893, 33.2848129272},
                                       {-115.511856079, 33.3774108887},
                                       {-115.330322266, 33.4182357788},
                                       {-115.141914368, 33.4441757202},
                                       {-114.987640381, 33.4703559875},
                                       {-114.804069519, 33.5033493042},
                                       {-114.648094177, 33.5111160278},
                                       {-114.463050842, 33.5279502869},
                                       {-114.310600281, 33.5477752686},
                                       {-114.123214722, 33.5645294189},
                                       {-113.977424622, 33.611240387},
                                       {-113.818893433, 33.6126861572},
                                       {-113.623725891, 33.5881576538},
                                       {-113.464317322, 33.5780601501},
                                       {-113.306732178, 33.574420929},
                                       {-113.149040222, 33.5656929016},
                                       {-112.989875793, 33.5374031067},
                                       {-112.829185486, 33.5234413147},
                                       {-112.667114258, 33.4946327209},
                                       {-112.505317688, 33.4651832581},
                                       {-112.34362793, 33.4315834045},
                                       {-112.204223633, 33.4060707092},
                                       {-112.042182922, 33.3793449402},
                                       {-111.882843018, 33.3153762817},
                                       {-111.743278503, 33.2630233765},
                                       {-122.850921631, 30.4390449524},
                                       {-122.638908386, 30.6507129669},
                                       {-122.522727966, 30.9010677338},
                                       {-122.505317688, 31.2056007385},
                                       {-122.486602783, 31.4705467224},
                                       {-107.476013184, 29.8041744232},
                                       {-107.415412903, 29.6678943634},
                                       {-107.387435913, 29.5551738739},
                                       {-107.347480774, 29.4125785828},
                                       {-107.309959412, 29.2666912079},
                                       {-107.270744324, 29.1528453827},
                                       {-107.250335693, 29.0270328522},
                                       {0.0, 0.0}};

std::vector<int> grounp2{44, 34, 10, 6, 16, 26, 14, 20, 5, 5, 1};

TEST(GroundLineTEST, DBSCAN) {
  std::vector<FusionFreespacePoint> input1;
  for (auto &v : data1) {
    Point3D p;
    FusionFreespacePoint fsp;
    p.x = v[0];
    p.y = v[1];
    p.z = 0.0;
    fsp.position = p;
    input1.emplace_back(fsp);
  }

  GroundLineDecider ground_line_decider = GroundLineDecider(1, 0.5);
  std::vector<std::vector<planning_math::Vec2d>> output1 =
      ground_line_decider.execute(input1);
  std::vector<int> result1;
  for (auto &v : output1) {
    result1.emplace_back(v.size());
  }
  sort(grounp1.begin(), grounp1.end());
  sort(result1.begin(), result1.end());
  std::cout << "target_output: ";
  for (auto &n : grounp1) {
    std::cout << " " << n;
  }
  std::cout << std::endl;
  std::cout << "test_result: ";
  for (auto &n : result1) {
    std::cout << " " << n;
  }
  std::cout << std::endl;
  EXPECT_TRUE((result1 == grounp1));
}

TEST(GroundLineTEST, DBSCAN2) {
  std::vector<FusionFreespacePoint> input2;
  for (auto &v : data2) {
    Point3D p;
    FusionFreespacePoint fsp;
    p.x = v[0];
    p.y = v[1];
    p.z = 0.0;
    fsp.position = p;
    input2.emplace_back(fsp);
  }

  GroundLineDecider ground_line_decider = GroundLineDecider(1, 0.5);
  std::vector<std::vector<planning_math::Vec2d>> output2 =
      ground_line_decider.execute(input2);
  std::vector<int> result2;
  for (auto &v : output2) {
    result2.emplace_back(v.size());
  }
  sort(grounp2.begin(), grounp2.end());
  sort(result2.begin(), result2.end());
  std::cout << "target_output: ";
  for (auto &n : grounp2) {
    std::cout << " " << n;
  }
  std::cout << std::endl;
  std::cout << "test_result: ";
  for (auto &n : result2) {
    std::cout << " " << n;
  }
  std::cout << std::endl;
  EXPECT_TRUE((result2 == grounp2));
}

} // namespace parking
} // namespace msquare