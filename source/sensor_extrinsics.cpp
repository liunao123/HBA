#include <iostream>
#include <unordered_map>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


// TFTransform: 4x4 matrix wrapper
struct TFTransform
{
    Eigen::Matrix4d mat;
    TFTransform() : mat(Eigen::Matrix4d::Identity()) {}
    TFTransform(const Eigen::Matrix4d &m) : mat(m) {}
};

// SensorExtrinsics: manage TF tree and lookup
class SensorExtrinsics
{
public:
    // 实现与tf2_ros Buffer::lookupTransform一致的接口
    // target_frame: 目标坐标系，source_frame: 源坐标系，out: source在target下的变换
    // 递归查找source到target链路，累乘变换，严格参考tf2_ros Buffer::lookupTransform
    bool lookupTransform(const std::string &target_frame, const std::string &source_frame, Eigen::Matrix4d &out)
    {
        if (target_frame == source_frame)
        {
            out = Eigen::Matrix4d::Identity();
            return true;
        }
        // 记录访问，防止环
        std::unordered_set<std::string> visited;
        return lookupTransformImpl(target_frame, source_frame, out, visited);
    }

private:
    // 递归实现，visited防环
    bool lookupTransformImpl(const std::string &target, const std::string &source, Eigen::Matrix4d &out, std::unordered_set<std::string> &visited)
    {
        if (target == source)
        {
            out = Eigen::Matrix4d::Identity();
            return true;
        }
        if (visited.count(source))
            return false;
        visited.insert(source);
        // 先查找source的所有父节点
        if (tf_map_.count(source))
        {
            for (const auto &[parent, tf] : tf_map_[source])
            {
                Eigen::Matrix4d parent2target;
                if (lookupTransformImpl(target, parent, parent2target, visited))
                {
                    out = parent2target * tf.mat;
                    return true;
                }
            }
        }
        // 再查找source的所有子节点（逆向）
        for (const auto &[maybe_child, vec] : tf_map_)
        {
            for (const auto &[parent, tf] : vec)
            {
                if (parent == source)
                {
                    Eigen::Matrix4d child2target;
                    if (lookupTransformImpl(target, maybe_child, child2target, visited))
                    {
                        out = child2target * tf.mat.inverse();
                        return true;
                    }
                }
            }
        }
        return false;
    }

public:
    // 发布所有TF为ROS static transform
    void publishAllStaticTFROS(double rate_hz = 1.0)
    {
        // 需要ros/ros.h, tf2_ros/static_transform_broadcaster.h, geometry_msgs/TransformStamped.h
        // 假设已初始化ros::NodeHandle nh;
        tf2_ros::StaticTransformBroadcaster static_broadcaster;
        ros::Rate rate(rate_hz);
        while (ros::ok())
        {
            ros::Time now = ros::Time::now();
            for (const auto &[child, vec] : tf_map_)
            {
                for (const auto &[parent, tf] : vec)
                {
                    geometry_msgs::TransformStamped tf_msg;
                    tf_msg.header.stamp = now;
                    tf_msg.header.frame_id = parent;
                    tf_msg.child_frame_id = child;
                    // 取旋转和平移
                    Eigen::Matrix3d rot = tf.mat.block<3, 3>(0, 0);
                    Eigen::Quaterniond q(rot);
                    tf_msg.transform.translation.x = tf.mat(0, 3);
                    tf_msg.transform.translation.y = tf.mat(1, 3);
                    tf_msg.transform.translation.z = tf.mat(2, 3);
                    tf_msg.transform.rotation.x = q.x();
                    tf_msg.transform.rotation.y = q.y();
                    tf_msg.transform.rotation.z = q.z();
                    tf_msg.transform.rotation.w = q.w();
                    static_broadcaster.sendTransform(tf_msg);
                }
            }
            rate.sleep();
        }
    }
    // Load all extrinsic files from folder (format: parent child 4x4 matrix)
    void loadFolder(const std::string &folder)
    {
        for (const auto &entry : std::filesystem::directory_iterator(folder))
        {
            if (entry.is_regular_file())
            {
                loadFile(entry.path().string());
            }
        }
    }

    // Add a TF (parent->child)
    void addTF(const std::string &parent, const std::string &child, const Eigen::Matrix4d &tf)
    {
        // 存储parent->child和child->parent两个方向
        tf_map_[child].emplace_back(parent, TFTransform(tf));
    }

    // Lookup transform from src to dst (BFS, tf2-style)
    bool lookup(const std::string &src, const std::string &dst, Eigen::Matrix4d &out)
    {
        if (src == dst)
        {
            out = Eigen::Matrix4d::Identity();
            return true;
        }
        // BFS: node, accumulated transform (src->node)
        std::unordered_set<std::string> visited;
        std::vector<std::pair<std::string, Eigen::Matrix4d>> queue;
        queue.emplace_back(src, Eigen::Matrix4d::Identity());
        visited.insert(src);
        while (!queue.empty())
        {
            auto [cur, tf_cur] = queue.back();
            queue.pop_back();
            // forward edges: cur->child
            if (tf_map_.count(cur))
            {
                for (const auto &[child, tf] : tf_map_[cur])
                {
                    if (visited.count(child))
                        continue;
                    Eigen::Matrix4d tf_next = tf_cur * tf.mat;
                    if (child == dst)
                    {
                        out = tf_next;
                        return true;
                    }
                    queue.emplace_back(child, tf_next);
                    visited.insert(child);
                }
            }
            // backward edges: parent->cur (need inverse)
            for (const auto &[parent, vec] : tf_map_)
            {
                for (const auto &[maybe_child, tf] : vec)
                {
                    if (maybe_child == cur && !visited.count(parent))
                    {
                        Eigen::Matrix4d tf_next = tf_cur * tf.mat.inverse();
                        if (parent == dst)
                        {
                            out = tf_next;
                            return true;
                        }
                        queue.emplace_back(parent, tf_next);
                        visited.insert(parent);
                    }
                }
            }
        }
        return false;
    }

private:
    // child -> vector of (parent, transform)
    std::unordered_map<std::string, std::vector<std::pair<std::string, TFTransform>>> tf_map_;

    // 支持多父节点的递归DFS链路查找（返回所有到root的路径）
    bool getAllChainsToRoot(const std::string &frame, std::vector<std::vector<std::pair<std::string, TFTransform>>> &all_chains, std::vector<std::pair<std::string, TFTransform>> current_chain = {}, std::unordered_set<std::string> visited = {})
    {
        if (visited.count(frame))
            return false; // avoid loop
        visited.insert(frame);
        if (!tf_map_.count(frame))
        {
            all_chains.push_back(current_chain);
            return true;
        }
        bool found = false;
        for (const auto &[parent, tf] : tf_map_[frame])
        {
            current_chain.push_back({frame, tf});
            if (getAllChainsToRoot(parent, all_chains, current_chain, visited))
                found = true;
            current_chain.pop_back();
        }
        return found;
    }

    // Load one extrinsic file (format: parent child 4x4 matrix)
    void loadFile(const std::string &file)
    {
        // extract parent and child from filename (parent is 2nd, child is 4th, remove extension)
        std::string filename = file.substr(file.find_last_of("/\\") + 1);
        std::vector<std::string> tokens;
        std::stringstream ss(filename);
        std::string token;
        while (std::getline(ss, token, '_'))
            tokens.push_back(token);
        std::string parent, child;
        if (tokens.size() >= 4)
        {
            parent = tokens[3];
            child = tokens[1];
            size_t dot = child.find('.');
            if (dot != std::string::npos)
                child = child.substr(0, dot);
        }
        // use yaml-cpp to parse
        try
        {
            YAML::Node root = YAML::LoadFile(file);
            std::vector<double> quat(4, 0.0);
            std::vector<double> trans(3, 0.0);
            if (root["r_quaternion_wxyz"])
            {
                auto qdata = root["r_quaternion_wxyz"]["data"];
                for (int i = 0; i < 4; ++i)
                    quat[i] = qdata[i].as<double>();
            }
            if (root["t_metric_xyz"])
            {
                auto tdata = root["t_metric_xyz"]["data"];
                for (int i = 0; i < 3; ++i)
                    trans[i] = tdata[i].as<double>();
            }
            if (!parent.empty() && !child.empty())
            {
                Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
                Eigen::Matrix3d R = q.normalized().toRotationMatrix();
                Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
                mat.block<3, 3>(0, 0) = R;
                mat(0, 3) = trans[0];
                mat(1, 3) = trans[1];
                mat(2, 3) = trans[2];
                addTF(parent, child, mat);
                std::cout << "Loaded TF: " << parent << " -> " << child << std::endl;
                // std::cout << "mat TF:\n " << mat << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "YAML parse error in " << file << ": " << e.what() << std::endl;
        }
    }
};

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
     Eigen::Vector3d n = R.col(0);
     Eigen::Vector3d o = R.col(1);
     Eigen::Vector3d a = R.col(2);

     Eigen::Vector3d ypr(3);
     double y = atan2(n(1), n(0));
     double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
     double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
     ypr(0) = y;
     ypr(1) = p;
     ypr(2) = r;

     return ypr ; // * 180.0 / M_PI ;
}

// Demo usage
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_cache_demo");
    ros::NodeHandle nh;

    std::string directory = "/media/xf/Elements/id4_1202/calib/extrinsics/";
    
    SensorExtrinsics cache;
    cache.loadFolder(directory);
    // 发布所有TF为static transform（需要ROS环境）
    Eigen::Matrix4d tf;
    // 例：查找 parent=vehicle, child=cgi830，得到vehicle到cgi830的变换
    // std::string parent = "rfl", child = "cam2";
    std::string parent = "cgi830", child = "hesai128";
    if (cache.lookupTransform(parent, child, tf))
    {
        std::cout << "Transform from " << parent << " to " << child << ":\n"
                  << tf << std::endl;
        // 输出旋转部分的四元数和欧拉角
        Eigen::Matrix3d rot = tf.block<3, 3>(0, 0);
        Eigen::Quaterniond q(rot);
        
        std::cout << "translation (x, y, z): "
                  << tf(0, 3) << ", " << tf(1, 3) << ", " << tf(2, 3) << std::endl;
        std::cout << "Quaternion (w, x, y, z): "
                  << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
        Eigen::Vector3d euler = R2ypr(rot);  // rot.eulerAngles(0, 1, 2); // ZYX顺序
        std::cout << "Euler angles (ypr, rad): "
                  << euler.transpose() << std::endl;
        std::cout << "Euler angles (ypr, deg): "
                  << (euler * 180.0 / M_PI).transpose() << std::endl;
    }
    else
    {
        std::cout << "No transform found!" << std::endl;
    }

    cache.publishAllStaticTFROS();

    return 0;
}
