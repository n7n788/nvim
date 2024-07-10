/*
* miyata 2024/07/01
*/

#ifndef ARTERY_FRENETPLANNING
#define ARTERY_FRENETPLANNING

#include <vector>

namespace artery
{

class FrenetPlanning
{
public:

    // FrenetFrameで作成された経路クラス
    struct FrenetPath
    {
        std::vector<double> t; // 時間ベクトル
        std::vector<double> x; // x座標ベクトル
        std::vector<double> y; // y座標ベクトル

        FrenetPath()
            : t(), x(), y() {}

        FrenetPath(const std::vector<double>& t, const std::vector<double>& x, const std::vector<double>& y)
            : t(t), x(x), y(y) {}
        
        FrenetPath(const FrenetPath&) = default;
        FrenetPath& operator=(const FrenetPath&) = default;
    };

    // 経路の候補を返すメソッド
    // TODO: 引数に目標到達地点を与える？
    std::vector<FrenetPath> getCandidatePath() const;
    // 経路の候補からコストが最小の経路を返す
    FrenetPath getMinCostPath(std::vector<FrenetPath>) const;
    // 2つの経路が衝突するかを判定
    bool checkPathCollistion(FrenetPath, FrenetPath) const; 
};
} // namespace artery

#endif /*ARTERY_FRENETPLANNING*/
