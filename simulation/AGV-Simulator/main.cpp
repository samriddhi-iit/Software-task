#include "draw.hpp"
#include "geometry.hpp"
#include "simulation.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;


int main() {
    agent myagent;
    vector <point>centroids;
    

    myagent.calculate_1 = [&](const envmap& curmap,
                             const array<pair<point, point>, playercount>& playerdata,
                             const array<point, rays>& raycasts, 
                             const agent& curplayer,
                             ftype& a, ftype& steer) {
       point myPos = playerdata[0].first;
    ftype myAngle = playerdata[0].second.y;

    // -------------------------
    // 2. BUILD SAFE GRID
    // -------------------------
    vector<point> centroids;

    ftype step = 2.0f / 19.0f;

    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 20; j++) {

            point p{-1.0f + i * step, -1.0f + j * step};

            bool too_close = false;
            ftype safety = 0.02f;

            for (const auto& obs : curmap) {
                for (const auto& v : obs) {
                    ftype dx = p.x - v.x;
                    ftype dy = p.y - v.y;

                    if (dx*dx + dy*dy < safety) {
                        too_close = true;
                        break;
                    }
                }
                if (too_close) break;
            }

            if (!too_close) {
                centroids.push_back(p);
                drawpoint(p, GREEN);
            } else {
                drawpoint(p, RED);
            }
        }
    }

    // -------------------------
    // 3. GOAL
    // -------------------------
    point goal = {-0.9f, -0.7f};
    drawpoint(goal, BLUE);
    ftype d_goal = dist(myPos, goal);

// STOP CONDITION
    if (d_goal < 0.01f) {
       
    a = 0.0f;
    steer = 0.0f;
     return; 

    drawpoint(goal, BLUE);
     // still show goal
    
 }

    // -------------------------
    // 4. PICK TARGET
    // -------------------------
   
    ftype best = 1e9;

    static point current_target;
    static bool has_target = false;

// pick new target only if needed
if (!has_target || dist(myPos, current_target) < 0.1f) {

    ftype best = 1e9;
    has_target = false;

    for (auto& pt : centroids) {

        ftype d_goal = dist(pt, goal);
        ftype d_car = dist(myPos, pt);

        point rel = rotate(pt - myPos, {0,0}, -myAngle);

        if (rel.x > 0 && d_car < 0.5f && d_goal < best) {
            best = d_goal;
            current_target = pt;
            has_target = true;
        }
    }
}


    point target = current_target;
    drawline(myPos, target, WHITE);

    // -------------------------
    // 5. CONTROL
    // -------------------------
    point local = rotate(target - myPos, {0,0}, -myAngle);
    ftype desired = atan2(local.y, local.x);

    while (desired > PI) desired -= 2 * PI;
    while (desired < -PI) desired += 2 * PI;

    // clamp steering
    steer = max(min(desired, (ftype)0.4), (ftype)-0.4);

    // smart movement
    if (d_goal < 0.2f)
    a = 0.1f;
else if (fabs(desired) > 0.3)
    a = 0.0f;
else
    a = 0.3f;
};








        


    array<agent, playercount> myagents;
    for (int i = 0; i < playercount; i++) {
        myagents[i] = myagent;
    }

    ftype simultime = 30;
    simulationinstance s(myagents, simultime);
    s.run();

    return 0;
} //is closes main()