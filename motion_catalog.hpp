#pragma once

#include "standing_motion.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <openssl/sha.h>

namespace motion_catalog {

struct Entry {
    std::string name;
    std::string category;
    std::string priority;
    std::string type;
};

constexpr std::array<const char*, standing_motion::kArmJoints> kJointNames{{
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"}};

inline bool valid_name(const std::string& name) {
    if (name.empty() || name.front() < 'a' || name.front() > 'z') return false;
    for (char c : name)
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
            return false;
    return true;
}

inline nlohmann::json read_json(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open JSON file: " + path);
    try { return nlohmann::json::parse(file); }
    catch (const nlohmann::json::exception& error) {
        throw std::runtime_error("invalid JSON in " + path + ": " + error.what());
    }
}

inline std::string required_string(const nlohmann::json& object, const char* field) {
    if (!object.is_object() || !object.contains(field) || !object.at(field).is_string())
        throw std::runtime_error(std::string("missing string field: ") + field);
    return object.at(field).get<std::string>();
}

class Catalog {
public:
    static Catalog load(const std::string& path, size_t expected_count = 22) {
        const auto document = read_json(path);
        if (!document.is_object() || document.value("schema_version", 0) != 1 ||
            !document.contains("actions") || !document.at("actions").is_array())
            throw std::runtime_error("invalid motion catalog header");
        Catalog catalog;
        for (const auto& item : document.at("actions")) {
            Entry entry{required_string(item, "name"),
                        required_string(item, "category"),
                        required_string(item, "priority"),
                        required_string(item, "type")};
            if (!valid_name(entry.name) ||
                (entry.category != "basic" && entry.category != "reception" &&
                 entry.category != "presentation" && entry.category != "interaction" &&
                 entry.category != "elevator") ||
                (entry.priority != "P0" && entry.priority != "P1" && entry.priority != "P2") ||
                (entry.type != "pose" && entry.type != "motion") ||
                (entry.type == "pose" && entry.category != "basic") ||
                (entry.type == "motion" && entry.category == "basic"))
                throw std::runtime_error("invalid catalog entry: " + entry.name);
            if (!catalog.entries_.emplace(entry.name, entry).second)
                throw std::runtime_error("duplicate catalog name: " + entry.name);
        }
        if (catalog.entries_.size() != expected_count)
            throw std::runtime_error("motion catalog has unexpected action count");
        return catalog;
    }

    const Entry& at(const std::string& name) const {
        const auto found = entries_.find(name);
        if (found == entries_.end()) throw std::runtime_error("action is not in catalog: " + name);
        return found->second;
    }

    size_t size() const { return entries_.size(); }

private:
    std::map<std::string, Entry> entries_;
};

inline std::string sha256_file(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("cannot open motion: " + path);
    SHA256_CTX context;
    SHA256_Init(&context);
    std::array<char, 8192> buffer{};
    while (file) {
        file.read(buffer.data(), buffer.size());
        const auto count = file.gcount();
        if (count > 0) SHA256_Update(&context, buffer.data(), static_cast<size_t>(count));
    }
    if (!file.eof()) throw std::runtime_error("cannot read motion: " + path);
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256_Final(digest, &context);
    std::ostringstream result;
    result << std::hex << std::setfill('0');
    for (unsigned char byte : digest) result << std::setw(2) << static_cast<int>(byte);
    return result.str();
}

struct Asset {
    Entry entry;
    std::vector<standing_motion::Waypoint> waypoints;
    std::vector<int> keyframe_times_ms;
    int nominal_duration_ms = 0;
    bool loopable = false;
    bool interruptible = true;
    double max_velocity = standing_motion::kMaxArmVelocity;
    double max_acceleration = standing_motion::kMaxArmAcceleration;
};

struct Library {
    Catalog catalog;
    std::string motions_dir;
    std::string metadata_dir;
    bool require_approved = true;
    std::string poses_dir = "poses";

    std::string motion_path(const Entry& entry) const {
        const std::string& directory = entry.type == "pose" ? poses_dir : motions_dir;
        return directory + "/" + entry.category + "/" + entry.name + ".dat";
    }

    std::string metadata_path(const Entry& entry) const {
        return metadata_dir + "/" + entry.name + ".json";
    }

    Asset load_asset(const std::string& name, bool require_v3 = false,
                     bool allow_preview = false) const {
        const Entry& entry = catalog.at(name);
        const std::string path = motion_path(entry);
        Asset asset;
        asset.entry = entry;
        asset.waypoints = standing_motion::load_motion(path);
        if (entry.type == "pose" && asset.waypoints.size() != 1)
            throw std::runtime_error("pose needs exactly 1 frame: " + name);
        asset.keyframe_times_ms.push_back(0);
        for (size_t i = 1; i < asset.waypoints.size(); ++i) {
            asset.nominal_duration_ms += static_cast<int>(std::lround(
                asset.waypoints[i].requested_duration * 1000));
            asset.keyframe_times_ms.push_back(asset.nominal_duration_ms);
        }
        if (require_approved) {
            const auto meta = read_json(metadata_path(entry));
            const std::string status = required_string(meta, "status");
            const bool approved = status == "approved";
            const bool preview_status = allow_preview &&
                (status == "draft" || status == "tested");
            if (!meta.is_object() || !meta.contains("schema_version") ||
                !meta.at("schema_version").is_number_integer() ||
                (meta.at("schema_version").get<int>() != 2 &&
                 meta.at("schema_version").get<int>() != 3) ||
                required_string(meta, "name") != entry.name ||
                required_string(meta, "category") != entry.category ||
                required_string(meta, "robot_model") != "G1-7DOF" ||
                required_string(meta, "source") != "teach_with_name" ||
                (!approved && !preview_status) ||
                (approved && (required_string(meta, "verified_by").empty() ||
                              required_string(meta, "verified_at").empty())) ||
                required_string(meta, "sha256") != sha256_file(path))
                throw std::runtime_error("action metadata status or file hash is invalid: " + name);
            if (!meta.contains("version") || !meta.at("version").is_number_integer() ||
                meta.at("version").get<int>() < 1 ||
                !meta.contains("scenes") || !meta.at("scenes").is_array() ||
                meta.at("scenes").empty())
                throw std::runtime_error("action metadata is incomplete: " + name);
            required_string(meta, "label");
            required_string(meta, "description");
            const std::string context = required_string(meta, "movement_context");
            if (context != "stationary")
                throw std::runtime_error("standing controller requires stationary action: " + name);
            if (meta.at("schema_version").get<int>() == 3) {
                if (required_string(meta, "type") != entry.type ||
                    !meta.contains("file_format_version") || meta.at("file_format_version") != 1 ||
                    !meta.contains("joint_count") || meta.at("joint_count") != 14 ||
                    !meta.contains("joint_names") || !meta.at("joint_names").is_array() ||
                    meta.at("joint_names").size() != kJointNames.size() ||
                    !meta.contains("keyframe_times_ms") ||
                    !meta.at("keyframe_times_ms").is_array() ||
                    meta.at("keyframe_times_ms").size() != asset.waypoints.size() ||
                    !meta.contains("nominal_duration_ms") ||
                    !meta.at("nominal_duration_ms").is_number_integer() ||
                    meta.at("nominal_duration_ms").get<int>() != asset.nominal_duration_ms ||
                    !meta.contains("entry_pose") || !meta.at("entry_pose").is_array() ||
                    !meta.contains("exit_pose") || !meta.at("exit_pose").is_array() ||
                    meta.at("entry_pose").size() != kJointNames.size() ||
                    meta.at("exit_pose").size() != kJointNames.size() ||
                    !meta.contains("loopable") || !meta.at("loopable").is_boolean() ||
                    !meta.contains("interruptible") || !meta.at("interruptible").is_boolean() ||
                    !meta.contains("allowed_robot_modes") ||
                    !meta.at("allowed_robot_modes").is_array() ||
                    meta.at("allowed_robot_modes") != nlohmann::json::array({"standing"}) ||
                    !meta.contains("safety") || !meta.at("safety").is_object())
                    throw std::runtime_error("invalid v3 motion metadata: " + name);
                for (size_t i = 0; i < kJointNames.size(); ++i) {
                    if (meta.at("joint_names").at(i) != kJointNames[i] ||
                        !meta.at("entry_pose").at(i).is_number() ||
                        !meta.at("exit_pose").at(i).is_number() ||
                        std::abs(meta.at("entry_pose").at(i).get<double>() -
                                 asset.waypoints.front().arm[i]) > 1e-5 ||
                        std::abs(meta.at("exit_pose").at(i).get<double>() -
                                 asset.waypoints.back().arm[i]) > 1e-5)
                        throw std::runtime_error("joint order or motion anchors changed: " + name);
                }
                for (size_t i = 0; i < asset.keyframe_times_ms.size(); ++i)
                    if (!meta.at("keyframe_times_ms").at(i).is_number_integer() ||
                        meta.at("keyframe_times_ms").at(i).get<int>() != asset.keyframe_times_ms[i])
                        throw std::runtime_error("keyframe timing changed: " + name);
                const auto& safety = meta.at("safety");
                if (!safety.contains("max_velocity_rad_s") ||
                    !safety.at("max_velocity_rad_s").is_number() ||
                    !safety.contains("max_acceleration_rad_s2") ||
                    !safety.at("max_acceleration_rad_s2").is_number() ||
                    required_string(safety, "joint_limits_profile") !=
                        "config/arm_joint_limits.json")
                    throw std::runtime_error("motion safety metadata missing: " + name);
                asset.max_velocity = safety.at("max_velocity_rad_s").get<double>();
                asset.max_acceleration = safety.at("max_acceleration_rad_s2").get<double>();
                if (!std::isfinite(asset.max_velocity) || !std::isfinite(asset.max_acceleration) ||
                    asset.max_velocity <= 0 || asset.max_acceleration <= 0 ||
                    asset.max_velocity > standing_motion::kMaxArmVelocity ||
                    asset.max_acceleration > standing_motion::kMaxArmAcceleration)
                    throw std::runtime_error("motion safety limits exceed controller limits: " + name);
                asset.loopable = meta.at("loopable").get<bool>();
                asset.interruptible = meta.at("interruptible").get<bool>();
            } else if (require_v3) {
                throw std::runtime_error("sequence requires v3 metadata: " + name);
            }
        }
        return asset;
    }

    std::vector<standing_motion::Waypoint> load(const std::string& name) const {
        return load_asset(name).waypoints;
    }
};

}  // namespace motion_catalog
