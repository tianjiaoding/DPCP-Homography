# Robust Homography Estimation via Dual Principal Component Pursuit

This repository contains implementation that robustly estimate a homography matrix from correspondences that contains a large number of outliers using Dual Principal Component Pursuit. For reference on formulation, theoretical guarantees and experiment results, please see [the following paper](https://openaccess.thecvf.com/content_CVPR_2020/papers/Ding_Robust_Homography_Estimation_via_Dual_Principal_Component_Pursuit_CVPR_2020_paper.pdf).


## Getting started
With `Matlab`, run `main_EPFL.m` or `main_HOMOGR.m`. The error metrics (e.g., rotation and translation error, transfer error) will show up in the console.

This implementation is an entire pipeline of structure from motion. Firstly, feature points are extracted from each view, and then are matched aross views to form correspondences. Then, the correspondences are normalized on each view, and then mapped into epipolar or homographic embeddings (§2). Now, Dual Principal Component Pursuit (§3) robustly estimate a nullspace from those embeddings (Table 1). Homographies are in turn extracted from such a nullspace. When camera calibration or intrinsic parameters are available, one gets a Euclidean homography, which can be decomposed to rotation and translation. The rotation and translation are compared with the ground-truth.

## Citation
```
@inproceedings{ding2020robust,
  title={Robust Homography Estimation via Dual Principal Component Pursuit},
  author={Ding, Tianjiao and Yang, Yunchen and Zhu, Zhihui and Robinson, Daniel P and Vidal, Ren{\'e} and Kneip, Laurent and Tsakiris, Manolis C},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  pages={6080--6089},
  year={2020}
}
```
