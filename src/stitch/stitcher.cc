// File: stitcher.cc
// Date: Wed Nov 22 15:45:13 2017 +0800
// Author: Yuxin Wu


#include "stitcher.hh"

#include <limits>
#include <string>
#include <cmath>
#include <queue>

#include "feature/matcher.hh"
#include "lib/imgproc.hh"
#include "lib/timer.hh"
#include "blender.hh"
#include "match_info.hh"
#include "transform_estimate.hh"
#include "camera_estimator.hh"
#include "camera.hh"
#include "warp.hh"
using namespace std;
using namespace pano;
using namespace config;

namespace pano {

// use in development
const static bool DEBUG_OUT = false;
const static char* MATCHINFO_DUMP = "log/matchinfo.txt";

Mat32f Stitcher::build() {
  vector<int> feature_less;                   // 记录缺少 feature 的图片序号
  calc_feature(feature_less);

  map<int, vector<int>> image_map = {         // 保存每张图片相邻的上下左右四个方向图片序号
    {1, {}}                                   // 序号 1 的图片是云台正朝下的图片，因此没有相邻的图片
  };
  vector<vector<int>> image_array = {         // 二维数组表示图片拼接的位置，用于生成 image_map（上边缘和左边缘多加一圈方便构造 image_map）
    {0, 32, 18, 11, 27, 8, 21, 5, 0, 32},
    {0, 32, 18, 11, 27, 8, 21, 5, 0, 32},
    {3, 29, 13, 23, 6, 15, 16, 25, 3, 29},
    {2, 28, 12, 20, 9, 10, 19, 26, 2, 28},
    {4, 24, 31, 17, 14, 30, 7, 22, 4, 24},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
  };

  // 构建 image_map
  for (int x = 1; x < (int)image_array.size() - 1; x++) {
    for (int y = 1; y < (int)image_array[x].size() - 1; y++) {
      // print_debug("%d -> %d, %d\n", image_array[x][y], image_array[x+1][y], image_array[x][y+1]);
      image_map.insert({ {image_array[x][y], { image_array[x+1][y], image_array[x][y+1], image_array[x-1][y], image_array[x][y-1] }} });
    }
  }

  pairwise_matches.resize(imgs.size());
  for (auto& k : pairwise_matches) k.resize(imgs.size());
  if (ORDERED_INPUT) {
    linear_pairwise_match();
  } else {
    pairwise_match();
    int image_count = (int)imgs.size();

    // 对于 feature 不足的图片，从 match 配置文件里面读取 match info
    for (int index = 0; index < (int)feature_less.size(); index++) {
      int image_index = feature_less[index];
      for (int j = 0; j < image_count; j++) {
        MatchInfo info1, info2;
        ifstream file1, file2;
        ostringstream stringStream1, stringStream2;
        stringStream1 << "../match/match_" << image_index << "_" << j << ".txt";
        stringStream2 << "../match/match_" << j << "_" << image_index << ".txt";
        string filename1 = stringStream1.str(), filename2 = stringStream2.str();
        file1.open(filename1);
        file2.open(filename2);
        pairwise_matches[image_index][j] = info1.deserialize(file1);
        pairwise_matches[j][image_index] = info2.deserialize(file2);
        file1.close();
        file2.close();
      }
    }

    // 对于每张图片，检测其相邻的上下左右图片的 match info
    // 如果 match info 为 0，则从 match 配置文件里面读取 match info
    for (int i = 0; i < image_count; i++) {
      vector<int> neighboors = image_map.at(i);
      for (int j = 0; j < (int)neighboors.size(); j++) {
        if (pairwise_matches[i][neighboors[j]].confidence == 0) {
          MatchInfo info1, info2;
          ifstream file1, file2;
          ostringstream stringStream1, stringStream2;
          stringStream1 << "../match/match_" << i << "_" << neighboors[j] << ".txt";
          stringStream2 << "../match/match_" << neighboors[j] << "_" << i << ".txt";
          string filename1 = stringStream1.str(), filename2 = stringStream2.str();
          file1.open(filename1);
          file2.open(filename2);
          pairwise_matches[i][neighboors[j]] = info1.deserialize(file1);
          pairwise_matches[neighboors[j]][i] = info2.deserialize(file2);
          file1.close();
          file2.close();
        }
      }
      // 对于不是相邻的图片，则将 match info 信息清空，减少其他位置图片的影响
      // 为了防止错拍图片导致拼接错乱
      for (int j = 0; j < image_count; j++) {
        if (!is_element_in_vector(neighboors, j)) {
          MatchInfo info;
          pairwise_matches[i][j] = info;
          // print_debug("%d -> %d: %f\n", i, j, pairwise_matches[i][j].confidence);
        }
      }
    }

    // save match info into file
    // for (int i = 0; i < image_count; i++) {
    //   for (int j = 0; j < image_count; j++) {
    //     ofstream file;
    //     ostringstream stringStream;
    //     stringStream << "../match/match_" << i << "_" << j << ".txt";
    //     string filename = stringStream.str();
    //     file.open(filename);
    //     pairwise_matches[i][j].serialize(file);
    //     file.close();
    //   }
    // }

    // read match info from file
    // for (int i = 0; i < image_count; i++) {
    //   for (int j = 0; j < image_count; j++) {
    //     if (pairwise_matches[i][j].confidence == 0) {
    //       MatchInfo info;
    //       ifstream file;
    //       ostringstream stringStream;
    //       stringStream << "../match/match_" << i << "_" << j << ".txt";
    //       string filename = stringStream.str();
    //       file.open(filename);
    //       pairwise_matches[i][j] = info.deserialize(file);
    //       file.close();
    //     }
    //   }
    // }
  }
  free_feature();
  //load_matchinfo(MATCHINFO_DUMP);
  if (DEBUG_OUT) {
    draw_matchinfo();
    dump_matchinfo(MATCHINFO_DUMP);
  }
  assign_center();

  if (ESTIMATE_CAMERA)
    estimate_camera();
  else
    build_linear_simple();		// naive mode
  pairwise_matches.clear();
  // TODO automatically determine projection method even in naive mode
  if (ESTIMATE_CAMERA)
    bundle.proj_method = ConnectedImages::ProjectionMethod::spherical;
  else
    bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
  print_debug("Using projection method: %d\n", bundle.proj_method);
  bundle.update_proj_range();

  return bundle.blend();
}

bool Stitcher::is_element_in_vector(vector<int>& v,int element) {
  vector<int>::iterator it;
  it = find(v.begin(), v.end(), element);
  if (it != v.end()) {
    return true;
  } else {
    return false;
  }
}

bool Stitcher::match_image(
    const PairWiseMatcher& pwmatcher, int i, int j) {
  auto match = pwmatcher.match(i, j);
  TransformEstimation transf(
      match, keypoints[i], keypoints[j],
      imgs[i].shape(), imgs[j].shape());	// from j to i. H(p_j) ~= p_i
  MatchInfo info;
  bool succ = transf.get_transform(&info);
  if (!succ) {
    if (-(int)info.confidence >= 8)	// reject for geometry reason
      print_debug("Reject bad match with %d inlier from %d to %d\n",
          -(int)info.confidence, i, j);
    return false;
  }
  auto inv = info.homo.inverse();	// TransformEstimation ensures invertible
  inv.mult(1.0 / inv[8]);	// TODO more stable?
  print_debug(
      "Connection between image %d and %d, ninliers=%lu/%d=%lf, conf=%f\n",
      i, j, info.match.size(), match.size(),
      info.match.size() * 1.0 / match.size(),
      info.confidence);

  // fill in pairwise matches
  pairwise_matches[i][j] = info;
  info.homo = inv;
  info.reverse();
  pairwise_matches[j][i] = move(info);
  return true;
}

void Stitcher::pairwise_match() {
  GuardedTimer tm("pairwise_match()");
  size_t n = imgs.size();
  vector<pair<int, int>> tasks;
  REP(i, n) REPL(j, i + 1, n) tasks.emplace_back(i, j);

  PairWiseMatcher pwmatcher(feats);

  int total_nr_match = 0;

#pragma omp parallel for schedule(dynamic)
  REP(k, (int)tasks.size()) {
    int i = tasks[k].first, j = tasks[k].second;
    bool succ = match_image(pwmatcher, i, j);
    if (succ)
      total_nr_match += pairwise_matches[i][j].match.size();
  }
  print_debug("Total number of matched keypoint pairs: %d\n", total_nr_match);
}

void Stitcher::linear_pairwise_match() {
  GuardedTimer tm("linear_pairwise_match()");
  int n = imgs.size();
  PairWiseMatcher pwmatcher(feats);
#pragma omp parallel for schedule(dynamic)
  REP(i, n) {
    int next = (i + 1) % n;
    if (!match_image(pwmatcher, i, next)) {
      if (i == n - 1)	// head and tail don't have to match
        continue;
      else
        error_exit(ssprintf("Image %d and %d don't match\n", i, next));
    }
    continue; // TODO FIXME a->b, b->a
    do {
      next = (next + 1) % n;
      if (next == i)
        break;
    } while (match_image(pwmatcher, i, next));
  }
}

void Stitcher::assign_center() {
  bundle.identity_idx = imgs.size() >> 1;
  //bundle.identity_idx = 0;
}

void Stitcher::estimate_camera() {
  vector<Shape2D> shapes;
  for (auto& m: imgs) shapes.emplace_back(m.shape());
  auto cameras = CameraEstimator{pairwise_matches, shapes}.estimate();

  // produced homo operates on [-w/2,w/2] coordinate
  REP(i, imgs.size()) {
    //cout << "Camera " << i << " " << cameras[i].R << ", " << cameras[i].K() << endl;
    bundle.component[i].homo_inv = cameras[i].K() * cameras[i].R;
    bundle.component[i].homo = cameras[i].Rinv() * cameras[i].K().inverse();
  }
}

void Stitcher::build_linear_simple() {
  // TODO bfs over pairwise to build bundle
  // assume pano pairwise
  int n = imgs.size(), mid = bundle.identity_idx;
  bundle.component[mid].homo = Homography::I();

  auto& comp = bundle.component;

  // accumulate the transformations
  if (mid + 1 < n) {
    comp[mid+1].homo = pairwise_matches[mid][mid+1].homo;
    REPL(k, mid + 2, n)
      comp[k].homo = comp[k - 1].homo * pairwise_matches[k-1][k].homo;
  }
  if (mid - 1 >= 0) {
    comp[mid-1].homo = pairwise_matches[mid][mid-1].homo;
    REPD(k, mid - 2, 0)
      comp[k].homo = comp[k + 1].homo * pairwise_matches[k+1][k].homo;
  }
  // comp[k]: from k to identity. [-w/2,w/2]

  // when estimate_camera is not used, homo is KRRK(2d-2d), not KR(2d-3d)
  // need to somehow normalize(guess) focal length to make non-flat projection work
  double f = -1;
  if (not TRANS)    // the estimation method only works under fixed-center projection
    f = Camera::estimate_focal(pairwise_matches);
  if (f <= 0) {
    print_debug("Cannot estimate focal. Will use a naive one.\n");
    f = 0.5 * (imgs[mid].width() + imgs[mid].height());
  }
  REP(i, n) {
    auto M = Homography{{
        1.0/f, 0,     0,
        0,     1.0/f, 0,
        0,     0,     1
    }};
    comp[i].homo = M * comp[i].homo;
  }
  bundle.calc_inverse_homo();
}

}	// namepsace pano

