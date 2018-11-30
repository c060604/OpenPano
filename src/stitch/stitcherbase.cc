//File: stitcherbase.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "stitcherbase.hh"
#include "lib/timer.hh"

using namespace std;

namespace pano {

void StitcherBase::calc_feature(vector<int>& feature_less) {
  GuardedTimer tm("calc_feature()");
  feats.resize(imgs.size());
  keypoints.resize(imgs.size());

  vector<Descriptor> feat;                          // 记录一个 size > 0 的 feature
  for (int i = 0; i < (int)imgs.size(); i++){
    imgs[i].load();
    feats[i] = feature_det->detect_feature(*imgs[i].img);
    if (feats[i].size() > 0) {
      feat = feats[i];
      break;
    }
  }
  // detect feature
#pragma omp parallel for schedule(dynamic)
  REP(k, (int)imgs.size()) {
    imgs[k].load();
    feats[k] = feature_det->detect_feature(*imgs[k].img);
    if (config::LAZY_READ)
      imgs[k].release();
    // if (feats[k].size() == 0)
    //   error_exit(ssprintf("Cannot find feature in image %d!\n", k));
    // print_debug("Image %d has %lu features\n", k, feats[k].size());
    if (feats[k].size() < 10) {
      feats[k] = feat;                            // TODO: 对于 size < 10 的 feature，用上面记录的 feature 替代。这是一个可调节参数，即使有 feature，也不一定能跟其他照片匹配。经测试 10 ~ 30 效果不错。
      feature_less.push_back(k);                  // 记录下这些 feature 不足的图片序号
    }
    keypoints[k].resize(feats[k].size());
    REP(i, feats[k].size())
      keypoints[k][i] = feats[k][i].coor;
  }
}

void StitcherBase::free_feature() {
  feats.clear(); feats.shrink_to_fit();  // free memory for feature
  keypoints.clear(); keypoints.shrink_to_fit();  // free memory for feature
}

}
