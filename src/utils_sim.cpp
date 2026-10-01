#include "Unscented_Kalman_Filter/utils_sim.h"

void drawPolyline(SDL_Renderer* renderer, const std::deque<SDL_FPoint>& pts,
                  Uint8 r, Uint8 g, Uint8 b, Uint8 a){
    if(pts.size() < 2) {return;}
    std::vector<SDL_FPoint> v(pts.begin(), pts.end());
    SDL_SetRenderDrawColor(renderer, r, g, b, a); // sets color for drawing operations
    SDL_RenderLines(renderer, v.data(), static_cast<int>(v.size())); // draws a series of connected lines using the points in v
}

void drawScatter(SDL_Renderer* renderer, const std::deque<SDL_FPoint>& pts,
                 Uint8 r, Uint8 g, Uint8 b, Uint8 a, float size){
    SDL_SetRenderDrawColor(renderer, r, g, b, a);
    for(const auto& p : pts){
        SDL_FRect rect{p.x - size/2.0f, p.y - size/2.0f, size, size}; // origin of rect is its top left corner
        SDL_RenderFillRect(renderer, &rect);
    }
}

void renderTextScaled(SDL_Renderer* renderer, float x, float y, float scale,
                      const char* text){
    SDL_SetRenderScale(renderer, scale, scale); // scales the size of the text drawn in SDL_RenderDebugText() below
    SDL_RenderDebugText(renderer, x / scale, y / scale, text); // divide x and y by the same scale factor to anchor in the same point as the original scale
    SDL_SetRenderScale(renderer, 1.0f, 1.0f); // reset to the original scale to avoid affecting subsequent plots
}

void drawUncertaintyEllipse(SDL_Renderer* renderer, const Camera& cam, 
                            double mean_x, double mean_y, 
                            double Pxx, double Pxy, double Pyy){
    /* Form the position covariance matrix */
    Eigen::Matrix2d cov {{Pxx, Pxy},
                         {Pxy, Pyy}};
    
    /* Compute eigenvalues of covariance matrix */
    // An eigenvalue is the variance along the eigenvector direction -> std = sqrt(eigenvalue)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    Eigen::Vector2d eigenvalues = solver.eigenvalues(); // obtain the eigenvalues in ascending order
    double lambda1 = std::max(eigenvalues(1), 0.0);     // larger eigenvalue
    double lambda2 = std::max(eigenvalues(0), 0.0);     // clamps potential small negative values that may arise 
                                                        // from floating-point roundoff to 0
    
    /* Compute rotation of lambda1 eigenvector */
    double angle = 0.5 * std::atan2(2.0 * Pxy, Pxx - Pyy);  // radians

    /* Multiply 1-sigma std by sqrt(5.991) to obtain 95% confidence region */
    const double chi2_95_2dof = 5.991;  // 5.991 is the critical value for 95% confidence in a 2dof chi-square distribution
    double semi_major = std::sqrt(lambda1 * chi2_95_2dof);
    double semi_minor = std::sqrt(lambda2 * chi2_95_2dof);

    /* Plot points to form the 95% confidence ellipse */
    const int N = 60;
    std::vector<SDL_FPoint> pts(N + 1); // vector to store the 61 points forming the ellipse
    double cosA = std::cos(angle); 
    double sinA = std::sin(angle);
    SDL_FPoint center = cam.toScreen(mean_x, mean_y);

    for(int i=0; i<=N; i++){
        // plot the initial point when the ellipse is parallel to the x, y axes
        double theta = 2.0 * M_PI * i/N;
        double lx = semi_major * std::cos(theta);
        double ly = semi_minor * std::sin(theta);
        // rotate anticlockwise by the eigenvector angle into world orientation, 
        // then scale pixels to flip y
        double wx = lx * cosA - ly * sinA;
        double wy = lx * sinA + ly * cosA;
        pts[i].x = static_cast<float> (center.x + wx * cam.scale);
        pts[i].y = static_cast<float> (center.y + wy * cam.scale);
    }

    /* Render the ellipse */
    SDL_SetRenderDrawColor(renderer, 255, 220, 60, 200); // yellow ellipse
    SDL_RenderLines(renderer, pts.data(), static_cast<int> (pts.size())); // plot the ellipse
}

double niceNum(double range){
    if(range <= 0.0) {return 1.0;}
    double exponent = std::floor(std::log10(range));    // find the closest power of 10 exponent
    double fraction = range / std::pow(10.0, exponent); // normalise to a range [1, 10)
    double niceFraction;
    
    // Threshold values for fraction are based on Paul S. Heckbert's 
    // "Nice Numbers for Graph Labels" algorithm
    if(fraction < 1.5) {niceFraction = 1.0;}
    else if (fraction < 3.0) {niceFraction = 2.0;}
    else if (fraction < 7.0) {niceFraction = 5.0;}
    else {niceFraction = 10.0;}
    return niceFraction * std::pow(10.0, exponent);
}

void drawAxes(SDL_Renderer* renderer, const Camera& cam){
    const double target_px_spacing = 110.0; // wider spacing = more room for tick labels
    const float label_scale = 1.6f;
    double spacing = niceNum(target_px_spacing / cam.scale); // nice grid line spacing in metres

    // Determine the bounds of the screen in world coordinates (metres)
    double half_w_m = (cam.win_w / 2.0) / cam.scale;
    double half_h_m = (cam.win_h / 2.0) / cam.scale;
    double world_left = cam.center_x - half_w_m;
    double world_right = cam.center_x + half_w_m;
    double world_top = cam.center_y + half_h_m;
    double world_bot = cam.center_y - half_h_m;

    // Draw gridlines in a dark greyish colour first 
    // (everything else renders on top of them)
    SDL_SetRenderDrawColor(renderer, 50, 50, 60, 255);
    double startX = std::ceil(world_left/spacing) * spacing;
    for(double x=startX; x<=world_right; x+=spacing){
        SDL_FPoint p1 = cam.toScreen(x, world_bot);
        SDL_FPoint p2 = cam.toScreen(x, world_top);
        SDL_RenderLine(renderer, p1.x, p1.y, p2.x, p2.y); // draw vertical gridlines from left to right
    }
    double startY = std::ceil(world_bot/spacing) * spacing;
    for(double y=startY; y<=world_top; y+=spacing){
        SDL_FPoint p1 = cam.toScreen(world_left, y);
        SDL_FPoint p2 = cam.toScreen(world_right, y);
        SDL_RenderLine(renderer, p1.x, p1.y, p2.x, p2.y); // draw horizontal gridlines from bot to top
    }

    // Draw tick labels in a light greyish colour 
    // (x labels along the bottom edge, y labels along the left edge)
    SDL_SetRenderDrawColor(renderer, 190, 190, 200, 255);
    char buffer[32];
    for(double x=startX; x<=world_right; x+=spacing){
        double sx = cam.toScreen(x, 0.0).x;     // convert to px coordinates
        std::snprintf(buffer, sizeof(buffer), "%.0f", x);
        renderTextScaled(renderer, static_cast<float> (sx - 14), 
                         static_cast<float> (cam.win_h - 26), label_scale, buffer);
    }
    for(double y=startY; y<=world_top; y+=spacing){
        double sy = cam.toScreen(0.0, y).y;     // convert to px coordinates
        std::snprintf(buffer, sizeof(buffer), "%.0f", y);
        renderTextScaled(renderer, 6, static_cast<float> (sy - 7), label_scale, 
                         buffer);
    }

    // Draw axes labels
    renderTextScaled(renderer, static_cast<float> (cam.win_w - 80),
                     static_cast<float> (cam.win_h - 50), label_scale, "x (m)");
    renderTextScaled(renderer, 50, 6, label_scale, "y (m)");                 
}