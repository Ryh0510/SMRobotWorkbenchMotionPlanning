#pragma once

#include "PlanningDataSource.h"
#include <MotionPlanningCore/MotionPlanning.h>
#include <functional>

namespace robot_qt_viewer
{
    // This adapter owns the spray schema; MotionPlanningCore only persists its
    // opaque metadata string and remains independent of the spray workbenches.
    class PlanningTrajectoryTransfer
    {
    public:
        using ForwardKinematics = std::function<Eigen::Isometry3d(const std::vector<double>&)>;
        static bool makeMotionPlan(const PlanningDataSnapshot& world,
            const std::string& robotId, const std::vector<std::string>& jointNames,
            motion_planning::StoredMotionPlan& plan, QString& error);
        static std::string encode(const PlanningDataSnapshot& snapshot);
        static bool decode(const std::string& metadata, PlanningDataSnapshot& snapshot);
        // referenceTimes are the ORIGINAL dense knot times, before CDF retiming.
        // They keep spray/pass boundaries aligned with the optimized samples.
        static bool makeAnalysisSnapshot(const motion_planning::StoredMotionPlan& plan,
            const ForwardKinematics& fk, const std::vector<double>& referenceTimes,
            PlanningDataSnapshot& snapshot, QString& error);
        static bool updateCartesianProcessData(motion_planning::StoredMotionPlan& plan,
            int insertedIndex = -1, int removedIndex = -1);
    };
}
