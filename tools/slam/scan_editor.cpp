// Fast interactive scan pose editor using OpenCV
// Loads pre-exported binary data, renders at 60fps
// Click to select scans, arrow keys to nudge, R/T to rotate
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cmath>
#include <vector>
#include <set>
#include <fstream>

struct Vec2 { float x, y; };

// Global state
std::vector<Vec2> map_pts;
std::vector<Vec2> poses_xy;
std::vector<float> poses_yaw;
std::vector<std::vector<Vec2>> scans;  // body-frame
int N = 0;

// View
float view_cx = 2.0f, view_cy = 1.0f;
float view_scale = 80.0f;  // pixels per meter
int W = 1400, H = 1000;

// Selection
float sel_x = 0, sel_y = 0, sel_radius = 0.8f;
std::set<int> selected;
bool has_selection = false;

// Undo
struct UndoEntry { std::vector<int> ids; std::vector<float> px, py, yaw; };
std::vector<UndoEntry> undo_stack;

cv::Point world2screen(float wx, float wy) {
    return cv::Point(
        (int)((wx - view_cx) * view_scale + W/2),
        (int)(H/2 - (wy - view_cy) * view_scale)
    );
}

Vec2 screen2world(int sx, int sy) {
    return { (sx - W/2) / view_scale + view_cx,
             (H/2 - sy) / view_scale + view_cy };
}

void project_scan(int i, std::vector<Vec2>& out) {
    out.clear();
    float c = cosf(poses_yaw[i]), s = sinf(poses_yaw[i]);
    for (auto& p : scans[i]) {
        out.push_back({
            c*p.x - s*p.y + poses_xy[i].x,
            s*p.x + c*p.y + poses_xy[i].y
        });
    }
}

void save_undo() {
    UndoEntry e;
    for (int i : selected) {
        e.ids.push_back(i);
        e.px.push_back(poses_xy[i].x);
        e.py.push_back(poses_xy[i].y);
        e.yaw.push_back(poses_yaw[i]);
    }
    undo_stack.push_back(e);
}

void draw(cv::Mat& img) {
    img.setTo(cv::Scalar(20, 20, 20));

    // Map walls (red)
    for (int i = 0; i < (int)map_pts.size(); i += 3) {
        auto p = world2screen(map_pts[i].x, map_pts[i].y);
        if (p.x >= 0 && p.x < W && p.y >= 0 && p.y < H)
            img.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(60, 60, 180);
    }

    // Projected scan points (colored by scan index)
    std::vector<Vec2> proj;
    for (int i = 0; i < N; i++) {
        project_scan(i, proj);
        float hue = (float)i / N * 180;
        cv::Scalar color;
        {
            cv::Mat hsv(1,1,CV_8UC3, cv::Scalar((int)hue, 200, 220));
            cv::Mat bgr;
            cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
            auto c = bgr.at<cv::Vec3b>(0,0);
            color = cv::Scalar(c[0], c[1], c[2]);
        }
        bool is_sel = selected.count(i) > 0;
        for (auto& pt : proj) {
            auto p = world2screen(pt.x, pt.y);
            if (p.x >= 0 && p.x < W && p.y >= 0 && p.y < H) {
                if (is_sel)
                    cv::circle(img, p, 2, cv::Scalar(0, 255, 0), -1);
                else
                    img.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b((int)color[0], (int)color[1], (int)color[2]);
            }
        }
    }

    // Trajectory
    for (int i = 1; i < N; i++) {
        auto p1 = world2screen(poses_xy[i-1].x, poses_xy[i-1].y);
        auto p2 = world2screen(poses_xy[i].x, poses_xy[i].y);
        cv::line(img, p1, p2, cv::Scalar(80, 80, 80), 1);
    }

    // Selection circle
    if (has_selection) {
        auto c = world2screen(sel_x, sel_y);
        int r = (int)(sel_radius * view_scale);
        cv::circle(img, c, r, cv::Scalar(255, 255, 255), 1);
    }

    // HUD
    char buf[256];
    snprintf(buf, sizeof(buf), "Sel: %d scans | Radius: %.2fm | Nudge: 5cm/0.5deg",
             (int)selected.size(), sel_radius);
    cv::putText(img, buf, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1);
    cv::putText(img, "Click=sel Arrows=nudge R/T=rot Z=undo S=save +/-=radius Q=quit Scroll=zoom",
                cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180,180,180), 1);
    if (!selected.empty()) {
        int lo = *selected.begin(), hi = *selected.rbegin();
        snprintf(buf, sizeof(buf), "Scans #%d-#%d", lo, hi);
        cv::putText(img, buf, cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
    }
}

// Mouse callback
void on_mouse(int event, int x, int y, int flags, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        auto w = screen2world(x, y);
        sel_x = w.x; sel_y = w.y;
        has_selection = true;
        selected.clear();

        // Find scans with projected points in radius
        std::vector<Vec2> proj;
        for (int i = 0; i < N; i++) {
            project_scan(i, proj);
            for (auto& pt : proj) {
                float dx = pt.x - sel_x, dy = pt.y - sel_y;
                if (dx*dx + dy*dy < sel_radius*sel_radius) {
                    selected.insert(i);
                    break;
                }
            }
        }
        printf("Selected %d scans near (%.2f, %.2f)\n", (int)selected.size(), sel_x, sel_y);
    }
    else if (event == cv::EVENT_MOUSEWHEEL) {
        int delta = cv::getMouseWheelDelta(flags);
        if (delta > 0) view_scale *= 1.15f;
        else view_scale /= 1.15f;
        view_scale = std::max(10.0f, std::min(500.0f, view_scale));
    }
    else if (event == cv::EVENT_RBUTTONDOWN) {
        // Pan: center view on click
        auto w = screen2world(x, y);
        view_cx = w.x; view_cy = w.y;
    }
}

bool load_data() {
    // Map
    {
        FILE* f = fopen("editor_map.bin", "rb");
        if (!f) { printf("No editor_map.bin\n"); return false; }
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        int n = sz / (2*sizeof(float));
        map_pts.resize(n);
        fread(map_pts.data(), sizeof(float), n*2, f);
        fclose(f);
        printf("Map: %d pts\n", n);
    }
    // Poses
    {
        FILE* f = fopen("editor_poses.bin", "rb");
        if (!f) { printf("No editor_poses.bin\n"); return false; }
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        N = sz / (3*sizeof(float));
        std::vector<float> buf(N*3);
        fread(buf.data(), sizeof(float), N*3, f);
        fclose(f);
        poses_xy.resize(N);
        poses_yaw.resize(N);
        for (int i = 0; i < N; i++) {
            poses_xy[i] = {buf[i*3], buf[i*3+1]};
            poses_yaw[i] = buf[i*3+2];
        }
        printf("Poses: %d\n", N);
    }
    // Scans
    {
        FILE* f = fopen("editor_scans.bin", "rb");
        if (!f) { printf("No editor_scans.bin\n"); return false; }
        int n_scans;
        fread(&n_scans, sizeof(int), 1, f);
        scans.resize(n_scans);
        int total = 0;
        for (int i = 0; i < n_scans; i++) {
            int n_pts;
            fread(&n_pts, sizeof(int), 1, f);
            scans[i].resize(n_pts);
            if (n_pts > 0)
                fread(scans[i].data(), sizeof(float), n_pts*2, f);
            total += n_pts;
        }
        fclose(f);
        printf("Scans: %d, total pts: %d\n", n_scans, total);
    }
    // Center view
    float sx=0, sy=0;
    for (int i = 0; i < N; i++) { sx += poses_xy[i].x; sy += poses_xy[i].y; }
    view_cx = sx / N; view_cy = sy / N;
    return true;
}

int main() {
    if (!load_data()) return 1;

    cv::namedWindow("Scan Editor", cv::WINDOW_NORMAL);
    cv::resizeWindow("Scan Editor", W, H);
    cv::setMouseCallback("Scan Editor", on_mouse);

    cv::Mat img(H, W, CV_8UC3);
    const float NUDGE = 0.05f;
    const float ROT = 0.5f * M_PI / 180.0f;

    while (true) {
        draw(img);
        cv::imshow("Scan Editor", img);
        int key = cv::waitKey(30);
        if (key == -1) continue;
        key &= 0xFF;

        if (key == 'q' || key == 27) break;

        if (!selected.empty()) {
            if (key == 83 || key == 'd') {  // right
                save_undo();
                for (int i : selected) poses_xy[i].x += NUDGE;
            } else if (key == 81 || key == 'a') {  // left
                save_undo();
                for (int i : selected) poses_xy[i].x -= NUDGE;
            } else if (key == 82 || key == 'w') {  // up
                save_undo();
                for (int i : selected) poses_xy[i].y += NUDGE;
            } else if (key == 84 || key == 'x') {  // down (note: 's' is save)
                save_undo();
                for (int i : selected) poses_xy[i].y -= NUDGE;
            } else if (key == 'r') {
                save_undo();
                for (int i : selected) poses_yaw[i] += ROT;
            } else if (key == 't') {
                save_undo();
                for (int i : selected) poses_yaw[i] -= ROT;
            }
        }

        if (key == 'z' && !undo_stack.empty()) {
            auto& e = undo_stack.back();
            for (int j = 0; j < (int)e.ids.size(); j++) {
                poses_xy[e.ids[j]] = {e.px[j], e.py[j]};
                poses_yaw[e.ids[j]] = e.yaw[j];
            }
            undo_stack.pop_back();
            printf("Undo\n");
        }

        if (key == 's') {
            FILE* f = fopen("manually_corrected_poses.csv", "w");
            fprintf(f, "scan,x,y,yaw_deg\n");
            for (int i = 0; i < N; i++)
                fprintf(f, "%d,%.6f,%.6f,%.6f\n", i, poses_xy[i].x, poses_xy[i].y,
                        poses_yaw[i] * 180.0f / M_PI);
            fclose(f);
            printf("Saved manually_corrected_poses.csv\n");
        }

        if (key == '+' || key == '=') sel_radius *= 1.3f;
        if (key == '-') sel_radius /= 1.3f;
    }
    return 0;
}
