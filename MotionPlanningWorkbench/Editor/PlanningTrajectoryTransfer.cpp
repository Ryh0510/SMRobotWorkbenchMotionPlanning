#include "PlanningTrajectoryTransfer.h"
#include <nlohmann/json.hpp>
#include <QCryptographicHash>
#include <algorithm>
#include <cmath>

namespace robot_qt_viewer
{
    namespace
    {
        using Json = nlohmann::json;
        bool rigidPose(const Eigen::Isometry3d& pose)
        {
            const auto& r = pose.linear();
            return pose.matrix().allFinite() &&
                (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-8 &&
                std::abs(r.determinant() - 1.0) < 1e-4 &&
                (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-4;
        }
        struct SourcePoint
        {
            spraytrajectory::SprayPathPoint point;
            int pass = 0;
        };
        std::vector<SourcePoint> sourcePoints(const spraytrajectory::SprayTrajectory& trajectory)
        {
            std::vector<SourcePoint> points;
            for(const auto& segment : trajectory.segments) {
                for(auto point : segment.points) {
                    point.sprayEnabled = point.sprayEnabled && segment.sprayEnabled;
                    if(point.processId.empty()) point.processId = segment.processId;
                    points.push_back({std::move(point), segment.passIndex});
                }
            }
            return points;
        }
        void append(spraytrajectory::SprayTrajectory& trajectory, SourcePoint sample)
        {
            const auto& p = sample.point;
            if(trajectory.segments.empty() ||
                trajectory.segments.back().passIndex != sample.pass ||
                trajectory.segments.back().processId != p.processId ||
                trajectory.segments.back().sprayEnabled != p.sprayEnabled) {
                spraytrajectory::SpraySegment segment;
                segment.passIndex = sample.pass;
                segment.processId = p.processId;
                segment.sprayEnabled = p.sprayEnabled;
                trajectory.segments.push_back(std::move(segment));
            }
            robottrajectory::TimedCartesianPoint cartesian;
            cartesian.time = p.time; cartesian.tcpPose = p.tcpPose;
            cartesian.linearSpeed = p.velocity;
            trajectory.baseCartesianTrajectory.points.push_back(std::move(cartesian));
            trajectory.segments.back().points.push_back(std::move(sample.point));
        }
        bool validPoints(const std::vector<SourcePoint>& points, QString& error)
        {
            if(points.empty()) { error = QStringLiteral("The trajectory has no points."); return false; }
            double previous = -1.0;
            for(const auto& item : points) {
                const auto& p = item.point;
                if(!std::isfinite(p.time) || p.time < 0.0 || p.time < previous ||
                    !rigidPose(p.tcpPose) || !p.targetNormal.allFinite() ||
                    !std::isfinite(p.targetDistance) || p.targetDistance < 0.0 ||
                    !std::isfinite(p.velocity) || !std::isfinite(p.acceleration)) {
                    error = QStringLiteral("Trajectory contains invalid poses, process data or non-monotonic times.");
                    return false;
                }
                previous = p.time;
            }
            return true;
        }
    }

    std::string PlanningTrajectoryTransfer::encode(const PlanningDataSnapshot& snapshot)
    {
        Json points = Json::array();
        for(const auto& item : sourcePoints(snapshot.trajectory)) {
            const auto& p = item.point;
            std::vector<double> pose;
            for(int r = 0; r < 4; ++r) for(int c = 0; c < 4; ++c) pose.push_back(p.tcpPose.matrix()(r,c));
            points.push_back(Json{{"time",p.time}, {"pose",pose}, {"pass",item.pass},
                {"spray",p.sprayEnabled}, {"type",static_cast<int>(p.pointType)},
                {"process",p.processId}, {"region",p.workpieceRegionId},
                {"distance",p.targetDistance}, {"velocity",p.velocity}, {"acceleration",p.acceleration},
                {"normal",{p.targetNormal.x(),p.targetNormal.y(),p.targetNormal.z()}}, {"joints",p.jointValues}});
        }
        return Json{{"schema","smrobot.planning-process.v1"},
            {"objectId",snapshot.modelObjectId.toStdString()}, {"modelPath",snapshot.modelPath.toStdString()},
            {"modelName",snapshot.modelName.toStdString()}, {"name",snapshot.trajectory.name},
            {"revision",snapshot.revision}, {"sprayAxis",snapshot.sprayAxis}, {"feedAxis",snapshot.feedAxis},
            {"world",snapshot.worldCoordinates}, {"producer",snapshot.producer.toStdString()},
            {"points",std::move(points)}}.dump();
    }

    bool PlanningTrajectoryTransfer::decode(const std::string& metadata, PlanningDataSnapshot& snapshot)
    {
        try {
            const auto json = Json::parse(metadata);
            if(json.value("schema",std::string()) != "smrobot.planning-process.v1") return false;
            PlanningDataSnapshot result;
            result.modelObjectId = QString::fromStdString(json.at("objectId").get<std::string>());
            result.modelPath = QString::fromStdString(json.at("modelPath").get<std::string>());
            result.modelName = QString::fromStdString(json.at("modelName").get<std::string>());
            result.trajectory.name = json.at("name").get<std::string>();
            result.revision = json.at("revision").get<quint64>();
            result.sprayAxis = json.at("sprayAxis").get<int>();
            result.feedAxis = json.at("feedAxis").get<int>();
            result.worldCoordinates = json.at("world").get<bool>();
            result.producer = QString::fromStdString(json.at("producer").get<std::string>());
            for(const auto& row : json.at("points")) {
                SourcePoint item;
                auto& p = item.point;
                const auto pose = row.at("pose").get<std::vector<double>>();
                const auto normal = row.at("normal").get<std::vector<double>>();
                if(pose.size() != 16 || normal.size() != 3) return false;
                for(int r = 0; r < 4; ++r) for(int c = 0; c < 4; ++c) p.tcpPose.matrix()(r,c) = pose[r*4+c];
                p.targetNormal = Eigen::Vector3d(normal[0],normal[1],normal[2]);
                p.time = row.at("time").get<double>();
                item.pass = row.at("pass").get<int>();
                p.sprayEnabled = row.at("spray").get<bool>();
                p.pointType = static_cast<spraytrajectory::SprayPathPointType>(row.at("type").get<int>());
                p.processId = row.at("process").get<std::string>();
                p.workpieceRegionId = row.at("region").get<int>();
                p.targetDistance = row.at("distance").get<double>();
                p.velocity = row.at("velocity").get<double>();
                p.acceleration = row.at("acceleration").get<double>();
                p.jointValues = row.at("joints").get<std::vector<double>>();
                append(result.trajectory,std::move(item));
            }
            QString error;
            if(!result.worldCoordinates || !validPoints(sourcePoints(result.trajectory),error)) return false;
            snapshot = std::move(result);
            return true;
        } catch(const std::exception&) { return false; }
    }

    bool PlanningTrajectoryTransfer::makeMotionPlan(const PlanningDataSnapshot& world,
        const std::string& robotId, const std::vector<std::string>& jointNames,
        motion_planning::StoredMotionPlan& plan, QString& error)
    {
        const auto points = sourcePoints(world.trajectory);
        if(!world.worldCoordinates || robotId.empty() || !validPoints(points,error)) {
            if(error.isEmpty()) error = QStringLiteral("A robot and world-coordinate trajectory are required.");
            return false;
        }
        motion_planning::StoredMotionPlan result;
        result.processMetadata = encode(world);
        const auto digest = QCryptographicHash::hash(QByteArray::fromStdString(result.processMetadata),
            QCryptographicHash::Sha256).toHex().left(12).toStdString();
        result.id = robotId + "_freeform_r" + std::to_string(world.revision) + "_" + digest;
        result.name = "Freeform / " + world.trajectory.name;
        result.robotId = robotId; result.jointNames = jointNames;
        result.cartesianControlPoints.name = result.name;
        for(const auto& item : points) {
            robottrajectory::TimedCartesianPoint p;
            p.time = item.point.time; p.tcpPose = item.point.tcpPose; p.linearSpeed = item.point.velocity;
            result.cartesianControlPoints.points.push_back(std::move(p));
        }
        plan = std::move(result);
        return true;
    }

    bool PlanningTrajectoryTransfer::makeAnalysisSnapshot(const motion_planning::StoredMotionPlan& plan,
        const ForwardKinematics& fk, const std::vector<double>& referenceTimes,
        PlanningDataSnapshot& snapshot, QString& error)
    {
        PlanningDataSnapshot source;
        if(!decode(plan.processMetadata,source)) {
            error = QStringLiteral("This trajectory has no linked workpiece/process data. Start with a freeform trajectory.");
            return false;
        }
        const auto original = sourcePoints(source.trajectory);
        const bool joint = !plan.trajectory.empty();
        const std::size_t count = joint ? plan.trajectory.points.size() : plan.cartesianControlPoints.points.size();
        if(count == 0 || (joint && !fk) || (!referenceTimes.empty() && referenceTimes.size() != count)) {
            error = QStringLiteral("Actual TCP kinematics and matching reference knots are required for transfer."); return false;
        }
        PlanningDataSnapshot result = source;
        result.trajectory = {};
        result.trajectory.name = plan.name;
        result.producer = QStringLiteral("Motion Planning");
        double previousReference = -1.0;
        for(std::size_t i = 0; i < count; ++i) {
            const double time = joint ? plan.trajectory.points[i].time : plan.cartesianControlPoints.points[i].time;
            const double reference = referenceTimes.empty() ? time : referenceTimes[i];
            if(!std::isfinite(reference) || reference < previousReference ||
                reference < original.front().point.time - 1e-8 || reference > original.back().point.time + 1e-8) {
                error = QStringLiteral("Process correspondence is missing or invalid; the analysis input was not replaced."); return false;
            }
            previousReference = reference;
            auto upper = std::upper_bound(original.begin(),original.end(),reference,
                [](double t, const auto& p) { return t < p.point.time; });
            const auto index = upper == original.begin() ? 0 : static_cast<std::size_t>(upper-original.begin()-1);
            SourcePoint item = original[index];
            item.point.time = time;
            if(joint) {
                const auto& q = plan.trajectory.points[i].q;
                if(q.size() != plan.jointNames.size() ||
                    !std::all_of(q.begin(),q.end(),[](double x){ return std::isfinite(x); })) {
                    error = QStringLiteral("Optimized joint values are invalid."); return false;
                }
                try { item.point.tcpPose = fk(q); }
                catch(const std::exception& ex) { error = QString::fromUtf8(ex.what()); return false; }
                item.point.jointValues = q;
            } else {
                item.point.tcpPose = plan.cartesianControlPoints.points[i].tcpPose;
                item.point.jointValues.clear();
            }
            if(!rigidPose(item.point.tcpPose)) { error = QStringLiteral("Actual TCP pose is invalid."); return false; }
            append(result.trajectory,std::move(item));
        }
        // Derive speed/acceleration from the final world TCP and execution times,
        // rather than copying pre-optimization feed rates after CDF retiming.
        std::vector<spraytrajectory::SprayPathPoint*> points;
        for(auto& segment : result.trajectory.segments) for(auto& p : segment.points) points.push_back(&p);
        for(std::size_t i = 1; i < points.size(); ++i) {
            const double dt = points[i]->time - points[i-1]->time;
            if(dt > 1e-12) points[i]->velocity = (points[i]->tcpPose.translation()-points[i-1]->tcpPose.translation()).norm()/dt;
            else points[i]->velocity = 0.0;
            if(dt > 1e-12) points[i]->acceleration = (points[i]->velocity-points[i-1]->velocity)/dt;
        }
        if(points.size() > 1) { points.front()->velocity = points[1]->velocity; points.front()->acceleration = 0.0; }
        if(!validPoints(sourcePoints(result.trajectory),error)) return false;
        for(std::size_t i = 0; i < points.size(); ++i)
            result.trajectory.baseCartesianTrajectory.points[i].linearSpeed = points[i]->velocity;
        snapshot = std::move(result);
        return true;
    }

    bool PlanningTrajectoryTransfer::updateCartesianProcessData(motion_planning::StoredMotionPlan& plan,
        int insertedIndex, int removedIndex)
    {
        if(plan.processMetadata.empty()) return true;
        PlanningDataSnapshot snapshot;
        if(!decode(plan.processMetadata,snapshot)) return false;
        auto points = sourcePoints(snapshot.trajectory);
        if(insertedIndex >= 0) {
            const auto index = static_cast<std::size_t>(insertedIndex);
            if(index > points.size() || points.empty()) return false;
            const auto copy = points[index == 0 ? 0 : index-1];
            points.insert(points.begin()+index,copy);
        }
        if(removedIndex >= 0) {
            const auto index = static_cast<std::size_t>(removedIndex);
            if(index >= points.size()) return false;
            points.erase(points.begin()+index);
        }
        if(points.size() != plan.cartesianControlPoints.points.size()) return false;
        snapshot.trajectory = {};
        snapshot.trajectory.name = plan.name;
        for(std::size_t i = 0; i < points.size(); ++i) {
            const auto& p = plan.cartesianControlPoints.points[i];
            points[i].point.tcpPose = p.tcpPose;
            points[i].point.time = p.time;
            points[i].point.velocity = p.linearSpeed;
            points[i].point.jointValues.clear();
            append(snapshot.trajectory,std::move(points[i]));
        }
        QString error;
        if(!validPoints(sourcePoints(snapshot.trajectory),error)) return false;
        plan.processMetadata = encode(snapshot);
        return true;
    }
}
