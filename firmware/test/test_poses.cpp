// Generate and validate resting poses using forward kinematics.
// Outputs JSON with pose angles, foot positions, stability metrics.
//
// Build: make -C firmware/test
// Run:   firmware/test/pose_generator

#include "mock_arduino.h"
#include "kinematics.h"
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

struct Pose {
    std::string name;
    float angles[8];  // FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
    std::string description;
};

struct PoseMetrics {
    float min_foot_y_mm;     // lowest foot — negative = underground
    float max_foot_y_mm;     // highest foot
    float body_height_mm;    // body center Y
    float pitch_deg;         // body tilt
    float roll_deg;
    float support_polygon_area_mm2;  // area of foot contact quadrilateral
    bool all_feet_grounded;
    bool stable;             // all feet grounded + low tilt
};

PoseMetrics evaluate_pose(const Pose& pose) {
    PoseMetrics m = {};
    BodyState state = compute_body_state(pose.angles);

    m.body_height_mm = state.position.y * 1000;
    m.pitch_deg = state.pitch * 180 / M_PI;
    m.roll_deg = state.roll * 180 / M_PI;

    m.min_foot_y_mm = 1e6;
    m.max_foot_y_mm = -1e6;
    m.all_feet_grounded = true;

    float foot_xs[4], foot_zs[4];
    for (int i = 0; i < 4; i++) {
        float fy = state.legs[i].foot.y * 1000;
        if (fy < m.min_foot_y_mm) m.min_foot_y_mm = fy;
        if (fy > m.max_foot_y_mm) m.max_foot_y_mm = fy;
        // Grounded if foot Y < 15mm (foot radius + margin)
        if (fy > 15.0f) m.all_feet_grounded = false;
        foot_xs[i] = state.legs[i].foot.x * 1000;
        foot_zs[i] = state.legs[i].foot.z * 1000;
    }

    // Support polygon area (shoelace formula on FL, FR, RR, RL)
    int order[] = {0, 1, 3, 2}; // FL, FR, RR, RL
    float area = 0;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        area += foot_xs[order[i]] * foot_zs[order[j]];
        area -= foot_xs[order[j]] * foot_zs[order[i]];
    }
    m.support_polygon_area_mm2 = fabsf(area) / 2;

    m.stable = m.all_feet_grounded
               && fabsf(m.pitch_deg) < 5.0f
               && fabsf(m.roll_deg) < 5.0f;

    return m;
}

void print_pose(const Pose& pose, const PoseMetrics& m) {
    printf("{\"name\":\"%s\",\"description\":\"%s\","
           "\"angles\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f],"
           "\"body_height_mm\":%.1f,"
           "\"min_foot_y_mm\":%.1f,\"max_foot_y_mm\":%.1f,"
           "\"pitch_deg\":%.2f,\"roll_deg\":%.2f,"
           "\"support_area_mm2\":%.0f,"
           "\"all_grounded\":%s,\"stable\":%s}\n",
           pose.name.c_str(), pose.description.c_str(),
           pose.angles[0], pose.angles[1], pose.angles[2], pose.angles[3],
           pose.angles[4], pose.angles[5], pose.angles[6], pose.angles[7],
           m.body_height_mm,
           m.min_foot_y_mm, m.max_foot_y_mm,
           m.pitch_deg, m.roll_deg,
           m.support_polygon_area_mm2,
           m.all_feet_grounded ? "true" : "false",
           m.stable ? "true" : "false");
}

// Search for optimal knee angle to put feet exactly on ground for a given hip angle
float find_grounding_knee(float hip_angle) {
    // Binary search for knee angle that puts foot at y=FOOT_R
    float lo = -1.5f, hi = 0.0f;
    for (int i = 0; i < 30; i++) {
        float mid = (lo + hi) / 2;
        LegFK fk = leg_fk(FL, hip_angle, mid);
        float foot_y = fk.foot.y + standing_height();
        if (foot_y < FOOT_R_M) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return (lo + hi) / 2;
}

int main() {
    // Generate poses with computed ground-contact knee angles
    std::vector<Pose> poses;

    // 1. Standing (default)
    {
        Pose p;
        p.name = "stand";
        p.description = "Default standing pose";
        float k = find_grounding_knee(FK_STAND_HIP);
        for (int i = 0; i < 4; i++) { p.angles[i*2] = FK_STAND_HIP; p.angles[i*2+1] = k; }
        poses.push_back(p);
    }

    // 2. Low stand (crouched)
    {
        Pose p;
        p.name = "crouch";
        p.description = "Low crouched stance, wider base";
        float hip = 0.5f;
        float k = find_grounding_knee(hip);
        for (int i = 0; i < 4; i++) { p.angles[i*2] = hip; p.angles[i*2+1] = k; }
        poses.push_back(p);
    }

    // 3. Tall stand (legs more straight)
    {
        Pose p;
        p.name = "tall";
        p.description = "Tall upright stance";
        float hip = 0.15f;
        float k = find_grounding_knee(hip);
        for (int i = 0; i < 4; i++) { p.angles[i*2] = hip; p.angles[i*2+1] = k; }
        poses.push_back(p);
    }

    // 4. Sit (front straight, rear tucked)
    {
        Pose p;
        p.name = "sit";
        p.description = "Sitting - front legs straight, rear tucked";
        float front_hip = 0.05f;
        float front_knee = find_grounding_knee(front_hip);
        float rear_hip = 1.0f;
        float rear_knee = -1.4f;
        p.angles[0] = front_hip; p.angles[1] = front_knee;
        p.angles[2] = front_hip; p.angles[3] = front_knee;
        p.angles[4] = rear_hip;  p.angles[5] = rear_knee;
        p.angles[6] = rear_hip;  p.angles[7] = rear_knee;
        poses.push_back(p);
    }

    // 5. Lie down (all legs folded)
    {
        Pose p;
        p.name = "lie_down";
        p.description = "Lying flat, all legs folded";
        for (int i = 0; i < 4; i++) { p.angles[i*2] = 0.8f; p.angles[i*2+1] = -1.2f; }
        poses.push_back(p);
    }

    // 6. Play bow (front down, rear up)
    {
        Pose p;
        p.name = "play_bow";
        p.description = "Play bow - front low, rear high";
        float front_hip = 0.7f;
        float front_knee = -1.0f;
        float rear_hip = 0.15f;
        float rear_knee = find_grounding_knee(rear_hip);
        p.angles[0] = front_hip; p.angles[1] = front_knee;
        p.angles[2] = front_hip; p.angles[3] = front_knee;
        p.angles[4] = rear_hip;  p.angles[5] = rear_knee;
        p.angles[6] = rear_hip;  p.angles[7] = rear_knee;
        poses.push_back(p);
    }

    // 7. Alert (forward lean)
    {
        Pose p;
        p.name = "alert";
        p.description = "Alert stance - slight forward lean";
        float front_hip = 0.2f;
        float front_knee = find_grounding_knee(front_hip);
        float rear_hip = 0.4f;
        float rear_knee = find_grounding_knee(rear_hip);
        p.angles[0] = front_hip; p.angles[1] = front_knee;
        p.angles[2] = front_hip; p.angles[3] = front_knee;
        p.angles[4] = rear_hip;  p.angles[5] = rear_knee;
        p.angles[6] = rear_hip;  p.angles[7] = rear_knee;
        poses.push_back(p);
    }

    // 8. Rest (relaxed, slightly splayed)
    {
        Pose p;
        p.name = "rest";
        p.description = "Relaxed rest, slightly lower than stand";
        float hip = 0.4f;
        float k = find_grounding_knee(hip);
        for (int i = 0; i < 4; i++) { p.angles[i*2] = hip; p.angles[i*2+1] = k; }
        poses.push_back(p);
    }

    // Evaluate and print all poses
    for (auto& pose : poses) {
        PoseMetrics m = evaluate_pose(pose);
        print_pose(pose, m);
    }

    return 0;
}
