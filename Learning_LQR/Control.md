# 学习动态规划：
| 强化学习 RL（Reward） | 动态规划 DP（Cost） |
|---|---|
| $V_k(s)=\displaystyle\max_a\left[r(s,a)+V_{k+1}(s')\right]$ | $J_k(s)=\displaystyle\min_a\left[c(s,a)+J_{k+1}(s')\right]$ |


# 学习LQR
[LQR学习视频](https://www.bilibili.com/video/BV1TV4y1u7TR?vd_source=6a13bff8988d9cf79144c65d599cf24f&spm_id_from=333.788.videopod.sections)

## 学习 LQR

| L | Q | R |
|---|---|---|
| **Linear** | **Quadratic** | **Regulator** |
| 系统状态空间模型是线性的 | 代价函数是二次型 | 最优状态反馈调节器 |
| $x_{k+1}=Ax_k+Bu_k$ | $J=\displaystyle\sum_{k=0}^{\infty}\left(x_k^TQx_k+u_k^TRu_k\right)$ | $u_k=-Kx_k$ |

> 缺少有限时域的LQR的数学推导，这里面缺少矩阵的求导相关知识。然后就是有限LQR到无限LQR还没有学习。