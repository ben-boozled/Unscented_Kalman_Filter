#pragma once
#define _USE_MATH_DEFINES

#include <deque>
#include <vector>
#include <cmath>
#include <SDL3/SDL.h>
#include <Eigen/Dense>

/** ====================================================================
 *                           Rendering Helpers 
 * ===================================================================== **/
struct Camera{
    int win_w = 1280, win_h = 900;
    double scale = 1.0; // pixels per meter (fixed value, no zooming in/out)
    double center_x = 0.0, center_y = 0.0; // current world position (in m) mapped to the screen center

    /**
     * @brief Converts world coordinates (in m) to screen coordinates (in pixels).
     *        Used to render thecamera view of the simulation.
     * 
     * @param x x-position of camera in world coordinates, m
     * @param y y-position of camera in world coordinates, m
     * 
     * @return SDL_FPoint structure that defines the camera's position in pixel
     *         coordinates    
    **/
    SDL_FPoint toScreen(double x, double y) const {
        SDL_FPoint p;
        p.x = static_cast<float>(win_w / 2.0 + (x - center_x)*scale);
        // in world coords, y increases upwards; in pixel coords, y increases downwards
        p.y = static_cast<float>(win_h / 2.0 - (y - center_y)*scale); 
        return p;
    };

    double metersToPixels(double m) const {return m*scale;}
};

/**
 * @brief Used to draw the true path and UKF-filtered path of the car.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *        settings. Can only render within SDL_Window.
 * @param pts The deque container storing the SDL_FPoint values of the true path 
 *        or the UKF-filtered path
 * @param r Red value to draw on the rendering target, [0, 255]
 * @param g Green value to draw on the rendering target, [0, 255]
 * @param b Blue value to draw on the rendering target, [0, 255]
 * @param a Alpha value, transparency settings (0=clear, 255=opaque)
**/
void drawPolyline(SDL_Renderer* renderer, const std::deque<SDL_FPoint>& pts,
                  Uint8 r, Uint8 g, Uint8 b, Uint8 a);

/**
 * @brief Used to draw the scatter plot for GPS readings.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *        settings. Can only render within SDL_Window.
 * @param pts The deque container storing the SDL_FPoint values of the GPS readings
 * @param r Red value to draw on the rendering target, [0, 255]
 * @param g Green value to draw on the rendering target, [0, 255]
 * @param b Blue value to draw on the rendering target, [0, 255]
 * @param a Alpha value, transparency settings (0=clear, 255=opaque)
 * @param size Side length (in pixels) of the squares used in the scatter plot
**/
void drawScatter(SDL_Renderer* renderer, const std::deque<SDL_FPoint>& pts,
                 Uint8 r, Uint8 g, Uint8 b, Uint8 a, float size);

/**
 * @brief Uses SDL's built-ind text renderer (SDL_RenderDebugText) to draw the
 *        HUD without installing a font library.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *        settings. Can only render within SDL_Window.
 * @param x x-px coordinate of the top-left corner of the text
 * @param y y-px coordinate of the top-left corner of the text
 * @param scale Scale factor for the text size of SDL_RenderDebugText from its 
 *        default 8x8 pixels per character bitmap
 * @param text The text to display
**/
void renderTextScaled(SDL_Renderer* renderer, float x, float y, float scale,
                      const char* text);

/**
 * @brief Draws the 95% confidence ellipse for the UKF's position estimate.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *        settings. Can only render within SDL_Window.
 * @param cam Camera typedef to convert points on the ellipse from world coordinates
 *        to pixel coordinates
 * @param mean_x Mean UKF-filtered x-position
 * @param mean_y Mean UKF-filtered y-position
 * @param Pxx Variance of x position, m^2
 * @param Pxy Covariance of x and positions, m^2
 * @param Pyy Variance of y position, m^2
**/
void drawUncertaintyEllipse(SDL_Renderer* renderer, const Camera& cam, 
                            double mean_x, double mean_y, 
                            double Pxx, double Pxy, double Pyy);

/**
 * @brief Rounds the range to the nearest "nice number" ( 1, 2, or 5 times a 
 *        power of 10) for grid line spacing to avoid awkward fractional spacing.
 * 
 * @param range The desired number of pixels between gridlines from drawAxes()
 * 
 * @return The nearest "nice number" ( 1, 2, or 5 times a power of 10) for grid 
 *         line spacing   
**/
double niceNum(double range);

/**
 * @brief Draws a light coordinate grid and meter tick labels along the bottom
 *        and left edge of the window. Allows for a visible scale of the car's
 *        travel distance even while the camera keeps the car centered. Gridline
 *        spacing is chosen to make the lines land at ~110px intervals on the
 *        screen, then rounded to a "nice" number of meters using niceNum().
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *        settings. Can only render within SDL_Window.
 * @param cam Camera typedef to convert points on from world coordinates to 
 *        pixel coordinates
**/
void drawAxes(SDL_Renderer* renderer, const Camera& cam);