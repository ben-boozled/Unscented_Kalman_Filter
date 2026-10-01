#include <iostream>
#define SDL_MAIN_HANDLED
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include "Unscented_Kalman_Filter/defines.h"
#include "Unscented_Kalman_Filter/UKF.h" // header files in folder named after the project to prevent name clash
#include "Unscented_Kalman_Filter/car_sim.h"
#include "Unscented_Kalman_Filter/sensor_sim.h"
#include "Unscented_Kalman_Filter/utils_sim.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/** ========================================================
 *                        Main Program 
 * ======================================================= **/
int main(int argc, char* argv[]){
    (void) argc; (void) argv;

    /** --------------------------------------------------------
     *              Initialise SDL video subsystem 
    -------------------------------------------------------- **/
    if(!SDL_Init(SDL_INIT_VIDEO)){
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", 
                                 "Error initialising SDL3", nullptr);
        return 1;
    }

    /** --------------------------------------------------------
     *                    Camera view setup
    -------------------------------------------------------- **/
    Camera cam;
    const double visible_width_m = 80.0;    // window will always show this many metres of width
    cam.scale = cam.win_w / visible_width_m;

    /* SDL window (GUI window) and SDL renderer (tool that draws inside the window) setup */
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    if(!SDL_CreateWindowAndRenderer("Unscented Kalman Filter - Car Tracking Demo",
                                    cam.win_w, cam.win_h, 0, &window, &renderer)){
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", 
                                 "Error creating window and renderer", nullptr);
        SDL_Quit();
        return 1;
    }

    /** --------------------------------------------------------
     *                    Simulation setup
    -------------------------------------------------------- **/
    // Render loop (how often the car motion is updated) runs at ~60Hz
    const double dt = 1.0 / 60.0;
    CarSimulator car(NOMINAL_CRUISE_SPEED, WHEELBASE, STEERING_ANGLE_CMD,
                     STRAIGHT_TIME, TURN_TIME);
    std::mt19937 rng(std::random_device{}());

    /** --------------------------------------------------------
     *             Initialise Unscented Kalman Filter
    -------------------------------------------------------- **/
    // Initial state mean vector
    VectorXd x0 {{car.get_px(),
                  car.get_py(),
                  car.get_vActual(),
                  car.get_yaw(),
                  0.0}};    // assume car starts straight (i.e. initial steering angle = 0)

    // Initial state covariance matrix
    MatrixXd P0 = MatrixXd::Identity(5, 5);

    UKF ukf(x0, P0); 
    
    /** --------------------------------------------------------
     *                   Initialise sensors
    -------------------------------------------------------- **/
    GPSSensor gps(GPS_RATE, GPS_POS_STD);   
    GyroSensor gyro(GYRO_RATE, GYRO_STD);
    AccelSensor accel(ACCEL_RATE, ACCEL_STD);

    std::vector<Sensor*> sensors = {&gps, &gyro, &accel};

    /** -----------------------------------------------------------
     * Historise trails in world-space coordinates up to a limit
    ----------------------------------------------------------- **/
    std::deque<std::pair<double, double>> trueWorld, ukfWorld;
    std::deque<std::pair<double, double>> gpsWorld;
    const size_t max_trail = 1200;      // ~20s of true/UKF history at 60Hz
    const size_t max_gps_trail = 150;   // ~30s of GPS history at 5Hz

    // Containers to store the points used for rendering
    std::deque<SDL_FPoint> truePathScreen, ukfPathScreen, gpsScreen;

    double sumSqErrUkf = 0.0, sumSqErrGps = 0.0;
    long nUkfSamples = 0, nGpsSamples = 0;

    bool running = true;
    Uint64 frame_start_time = SDL_GetTicks(); // ms since SDL library was initialised
    /** --------------------------------------------------------
     *                 Main simulation loop
    -------------------------------------------------------- **/
    while (running) {
        /* Checks for user input to quit (non-blocking, runs once per frame) */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if(ev.type == SDL_EVENT_QUIT) {running = false;} // user clicks the "x" button
            if(ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_ESCAPE) {running = false;} // user presses esc key
        }
        
        /** --------------------------------------------------------
         *                Simulates one time step
        -------------------------------------------------------- **/
        car.step(dt, rng);

        // Always run the UKF predict step, independent of sensor firing,
        // to keep the estimate up to date 
        ukf.predict(dt); 

        gps.hasReading = false;
        for(Sensor* pSensor : sensors) {pSensor->tick(dt, car, rng, ukf);} // tick all sensors
        bool gotGps = gps.hasReading;
        double gps_px = gps.last_px, gps_py = gps.last_py;

        /** --------------------------------------------------------
         *                         Camera  
         * (follow ground truth to avoid frame jitters from sensor 
         *              corrections on the UKF path)
        -------------------------------------------------------- **/
        cam.center_x = car.get_px();
        cam.center_y = car.get_py();

        /** --------------------------------------------------------
         *               Bookkeeping for plots/stats  
        -------------------------------------------------------- **/
        double error_x = ukf.getState()(0) - car.get_px();
        double error_y = ukf.getState()(1) - car.get_py();
        sumSqErrUkf += error_x * error_x + error_y * error_y;
        nUkfSamples++;
        
        if(gotGps) {
            double g_err_x = gps_px - car.get_px();
            double g_err_y = gps_py - car.get_py();
            sumSqErrGps += g_err_x * g_err_x + g_err_y * g_err_y;
            nGpsSamples++;
        }

        // Add latest position values to the end of the queue
        trueWorld.push_back({car.get_px(), car.get_py()});
        ukfWorld.push_back({ukf.getState()(0), ukf.getState()(1)});
        if(gotGps) {gpsWorld.push_back({gps_px, gps_py});}

        // Remove oldest position values from the start of the queue
        if(trueWorld.size() > max_trail) {trueWorld.pop_front();}
        if(ukfWorld.size() > max_trail) {ukfWorld.pop_front();}
        if(gpsWorld.size() > max_gps_trail) {gpsWorld.pop_front();}

        /** --------------------------------------------------------
         *                     Render visuals  
        -------------------------------------------------------- **/
        SDL_SetRenderDrawColor(renderer, 18, 18, 24, 255);
        SDL_RenderClear(renderer); // clear entire renderer and fill it with the color in SDL_SetRenderDrawColor() 

        drawAxes(renderer, cam);

        // Clear old points used to render the screen -> convert new world points to px coordinates
        // -> add new px coordinates to the deque to render updated frame
        truePathScreen.clear();
        for(auto& pair_coord : trueWorld) {truePathScreen.push_back(cam.toScreen(pair_coord.first, pair_coord.second));}
        ukfPathScreen.clear();
        for(auto& pair_coord : ukfWorld) {ukfPathScreen.push_back(cam.toScreen(pair_coord.first, pair_coord.second));}
        gpsScreen.clear();
        for(auto& pair_coord : gpsWorld) {gpsScreen.push_back(cam.toScreen(pair_coord.first, pair_coord.second));}
    
        drawPolyline(renderer, truePathScreen, 60, 220, 90, 255);    // green: ground truth
        drawScatter(renderer, gpsScreen, 235, 70, 70, 220, 5.0f);    // red: noisy gps readings
        drawPolyline(renderer, ukfPathScreen, 70, 170, 255, 255);    // blue: UKF estimate   
    
        // Draw 95% confidence ellipse around UKF estimate
        drawUncertaintyEllipse(renderer, cam, ukf.getState()(0), ukf.getState()(1), 
                               ukf.getCovariance()(0,0), ukf.getCovariance()(0,1),
                               ukf.getCovariance()(1,1));

        // Current position markers
        SDL_FPoint tp = cam.toScreen(car.get_px(), car.get_py());   // current true pos
        SDL_FRect r_tp{tp.x - 4, tp.y - 4, 8, 8};
        SDL_SetRenderDrawColor(renderer, 60, 220, 90, 255);
        SDL_RenderFillRect(renderer, &r_tp);
        
        SDL_FPoint up = cam.toScreen(ukf.getState()(0), ukf.getState()(1));     // current ukf pos
        SDL_FRect r_up{up.x - 4, up.y - 4, 8, 8};
        SDL_SetRenderDrawColor(renderer, 70, 170, 255, 255);
        SDL_RenderFillRect(renderer, &r_up);

        /* HUD using SDL3's in-built SDL_RenderDebugText() */
        double rmseUkf = (nUkfSamples > 0) ? std::sqrt(sumSqErrUkf / nUkfSamples) : 0.0 ;
        double rmseGps = (nGpsSamples > 0) ? std::sqrt(sumSqErrGps / nGpsSamples) : 0.0;
        char buffer[256];
        const float hud_scale = 2.0f;   // scale compared to the standard 8-by-8px per character bitmap
        const float line_h = 32.0f;     // px height of every line
        const float lx = 60.0f;          // top-left x-px coordinate of the HUD text
        float ly = 40.0f;               // top-left y-px coordinate of the current HUG text line
        SDL_SetRenderDrawColor(renderer, 230, 230, 230, 255);   // whitish colour for HUD

        // Legend HUD
        renderTextScaled(renderer, lx, ly, hud_scale, "Green = True  Red = GPS  Blue = UKF  Yellow = 95% ellipse");
        
        // RMSE HUD
        ly += line_h;
        std::snprintf(buffer, sizeof(buffer), "GPS RMSE: %.2fm  UKF RMSE: %.2fm", rmseGps, rmseUkf);
        renderTextScaled(renderer, lx, ly, hud_scale, buffer);
        
        // True state HUD
        ly += line_h;
        std::snprintf(buffer, sizeof(buffer), "True: v=%.2fm/s  yaw=%.2fdeg  steer=%.2fdeg  yawRate = %.2frad/s", 
                      car.get_vActual(), car.get_yaw() * 180 / M_PI, car.get_steeringAngle() * 180 / M_PI, car.get_yawRate());
        renderTextScaled(renderer, lx, ly, hud_scale, buffer);

        // UKF HUD
        ly += line_h;
        double ukf_v = ukf.getState()(2);                           // velocity
        double ukf_delta = ukf.getState()(4);                       // steering angle
        double ukf_yaw_rate = (ukf_v/ukf.L) * std::tan(ukf_delta);  // yaw rate
        std::snprintf(buffer, sizeof(buffer), "UKF: v=%.2fm/s  yaw=%.2fdeg  steer=%.2fdeg  yawRate = %.2frad/s",
                      ukf_v, ukf.getState()(3) * 180 / M_PI, ukf_delta * 180 / M_PI, ukf_yaw_rate);
        renderTextScaled(renderer, lx, ly, hud_scale, buffer);

        // Sampling Rates HUD
        ly += line_h;
        std::snprintf(buffer, sizeof(buffer), "Sampling Rates: Sim %.0fHz  GPS %.0fHz  Gyro %.0fHz  Accel %.0fHz",
                      1.0/dt, GPS_RATE, GYRO_RATE, ACCEL_RATE);
        renderTextScaled(renderer, lx, ly, hud_scale, buffer);

        // Displacement HUD
        ly += line_h;
        std::snprintf(buffer, sizeof(buffer), "Displacement from Origin: %.0fm", 
                      std::hypot(car.get_px(), car.get_py()));
        renderTextScaled(renderer, lx, ly, hud_scale, buffer);

        // ESC/Quit HUD
        renderTextScaled(renderer, lx, static_cast<float> (cam.win_h - 50), hud_scale, 
                         "Press ESC or close the window to quit :3");

        /* Update the screen with all renderings performed since the previous call */
        SDL_RenderPresent(renderer); 

        /** --------------------------------------------------------
         *              Frame pacing (target: ~60fps)  
        -------------------------------------------------------- **/
        Uint64 current_time = SDL_GetTicks();
        Uint64 time_elapsed = current_time - frame_start_time;
        const Uint64 target_ms = 16;    // from 1s / 60frames
        if(time_elapsed < target_ms) {SDL_Delay(static_cast<Uint32> (target_ms - time_elapsed));}   // wait
        frame_start_time = SDL_GetTicks(); // update start time for next frame
    }

    /* SDL clean up functions*/
    SDL_DestroyRenderer(renderer); // call before destroying the associated window
    SDL_DestroyWindow(window);  
    SDL_Quit();

    return 0;
}