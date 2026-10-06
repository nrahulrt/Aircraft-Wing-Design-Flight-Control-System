#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#define _USE_MATH_DEFINES
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <array>

enum class Flaps { UP = 0, F1 = 1, F5 = 5, F10 = 10, F20 = 20, F25 = 25, F30 = 30 };
enum class SpeedBrake { RETRACTED, ARMED, FLIGHT_DETENT, UP };
enum class Throttle { REV_MAX, REV_IDLE, IDLE, CONT_CRUISE, CLB, MCT, TOGA };
enum class AltitudeLevel { SEA_LEVEL, FT_10000, FT_35000 };
enum class AirspeedTarget { KIAS_150, KIAS_250, MACH_085 };
enum class AircraftWeight { MTOW, TYPICAL_CRUISE, MLW };

struct Point2D { double x; double y; };

struct SimTelemetry {
    double time_sec;
    double load_factor;
    double spec_excess_power;
    double root_bending_moment;
    double drag_parasite;
    double drag_induced;
    double drag_total;
    
    double dyn_pressure;
    double lift;
    double thrust;
    double fuel_flow;
    double tsfc;
    double lift_drag_ratio;
    bool is_stalled;
};

struct AutopilotState {
    bool alt_hold = false;
    double h_target = 10000.0;
    bool mach_hold = false;
    double m_target = 0.85;
    bool hdg_sel = false;
    double rad_target = 0.0;
};

// Data Render Payload - Zero Copy architecture
struct RenderData {
    std::vector<Point2D> airfoil_top;
    std::vector<Point2D> airfoil_bottom;
    std::vector<std::vector<Point2D>> streamlines;
    std::vector<Point2D> separation_points;
    std::vector<std::vector<Point2D>> recirculation_zones;
    double pitch_angle;
    bool stalled;
};

class Boeing747Physics {
private:
    // Certified Engineering Specifications (Boeing 747-400 Airport Planning Document)
    const double wing_area_m2 = 511.0; 
    const double wingspan_m = 64.4;
    const double aspect_ratio = 7.7;
    const double oswald_efficiency = 0.80;
    const double cd0 = 0.022; // Clean configuration zero-lift drag
    const double gravity = 9.80665;
    const double tsfc_cruise = 0.000015; // kg/Ns (GE CF6 engine approximation)
    const double max_thrust_n = 1120000.0; // 4x 280kN engines

    double time_sec_ = 0.0;
    double pitch_aoa_deg_ = 0.0;
    double roll_deg_ = 0.0;
    
    AltitudeLevel current_alt_ = AltitudeLevel::SEA_LEVEL;
    AirspeedTarget current_spd_ = AirspeedTarget::KIAS_250;
    AircraftWeight current_wt_ = AircraftWeight::TYPICAL_CRUISE;
    Flaps current_flaps_ = Flaps::UP;
    SpeedBrake current_sb_ = SpeedBrake::RETRACTED;
    Throttle current_throttle_ = Throttle::IDLE;
    AutopilotState ap_state_;

    double getDensity() const noexcept {
        switch (current_alt_) {
            case AltitudeLevel::SEA_LEVEL: return 1.225;
            case AltitudeLevel::FT_10000:  return 0.904;
            case AltitudeLevel::FT_35000:  return 0.379;
            default: return 1.225;
        }
    }

    double getVelocity() const noexcept {
        switch (current_spd_) {
            case AirspeedTarget::KIAS_150: return 77.16; // m/s
            case AirspeedTarget::KIAS_250: return 128.6;
            case AirspeedTarget::MACH_085: 
                return 0.85 * (current_alt_ == AltitudeLevel::FT_35000 ? 295.0 : 340.2);
            default: return 128.6;
        }
    }

    double getMass() const noexcept {
        switch (current_wt_) {
            case AircraftWeight::MTOW: return 396890.0;
            case AircraftWeight::TYPICAL_CRUISE: return 330000.0;
            case AircraftWeight::MLW: return 295742.0;
            default: return 330000.0;
        }
    }

public:
    void setPitch(double aoa_deg) noexcept { pitch_aoa_deg_ = aoa_deg; }
    double getPitch() const noexcept { return pitch_aoa_deg_; }
    void setRoll(double roll_deg) noexcept { roll_deg_ = roll_deg; }
    void setFlaps(Flaps f) noexcept { current_flaps_ = f; }
    void setSpeedBrake(SpeedBrake sb) noexcept { current_sb_ = sb; }
    void setThrottle(Throttle t) noexcept { current_throttle_ = t; }
    
    void setAltitudeType(AltitudeLevel alt) noexcept { current_alt_ = alt; }
    void setAirspeedType(AirspeedTarget spd) noexcept { current_spd_ = spd; }
    void setWeightType(AircraftWeight wt) noexcept { current_wt_ = wt; }

    void updateAutopilot(const AutopilotState& state) noexcept { ap_state_ = state; }

    SimTelemetry tick(double delta_time) {
        if (delta_time <= 0.0) {
            SimTelemetry current_state{};
            current_state.time_sec = time_sec_;
            current_state.dyn_pressure = 0.0;
            current_state.lift = 0.0;
            current_state.thrust = 0.0;
            current_state.fuel_flow = 0.0;
            current_state.tsfc = tsfc_cruise;
            current_state.lift_drag_ratio = 0.0;
            current_state.is_stalled = false;
            return current_state;
        }

        time_sec_ += delta_time;
        double rho = getDensity();
        double V = getVelocity();
        double mass = getMass();
        double weight_N = mass * gravity;
        if (weight_N <= 0.0) {
            weight_N = 1.0;
        }

        // Autopilot overrides
        if (ap_state_.alt_hold) {
            // Basic P-controller to adjust pitch for Level flight L = W / cos(roll).
            // Prevent singularities near roll = ±90 deg.
            double roll_rad = roll_deg_ * (M_PI / 180.0);
            double roll_cos = std::cos(roll_rad);
            if (std::abs(roll_cos) < 1e-6) {
                roll_cos = (roll_cos < 0.0) ? -1e-6 : 1e-6;
            }

            double target_lift = weight_N / roll_cos;
            double q_dyn = 0.5 * rho * V * V;
            double req_cl = target_lift / (q_dyn * wing_area_m2);
            double cl_flap = static_cast<double>(current_flaps_) * 0.02;
            pitch_aoa_deg_ = (req_cl - 0.2 - cl_flap) / 0.1;
            pitch_aoa_deg_ = std::clamp(pitch_aoa_deg_, -5.0, 20.0);
        }

        double dyn_pressure = 0.5 * rho * V * V;
        double cl_flap_increment = static_cast<double>(current_flaps_) * 0.02;
        double cl = 0.2 + (0.1 * pitch_aoa_deg_) + cl_flap_increment;
        
        bool is_stalled = false;
        if (pitch_aoa_deg_ > 15.0) {
            cl *= 0.4; // Stall lift drop
            is_stalled = true;
        }

        double lift = dyn_pressure * wing_area_m2 * cl;

        double drag_parasite = dyn_pressure * wing_area_m2 * cd0;
        if (current_sb_ == SpeedBrake::UP) drag_parasite *= 1.8;
        else if (current_sb_ == SpeedBrake::FLIGHT_DETENT) drag_parasite *= 1.4;
        
        if (is_stalled) drag_parasite *= 3.0; // Stall drag spike

        double drag_induced = dyn_pressure * wing_area_m2 * ((cl * cl) / (M_PI * oswald_efficiency * aspect_ratio));
        double drag_total = drag_parasite + drag_induced;

        double thrust_percentage = 0.1;
        switch (current_throttle_) {
            case Throttle::REV_MAX: thrust_percentage = -0.5; break;
            case Throttle::REV_IDLE: thrust_percentage = -0.1; break;
            case Throttle::IDLE: thrust_percentage = 0.1; break;
            case Throttle::CONT_CRUISE: thrust_percentage = 0.6; break;
            case Throttle::CLB: thrust_percentage = 0.85; break;
            case Throttle::MCT: thrust_percentage = 0.95; break;
            case Throttle::TOGA: thrust_percentage = 1.0; break;
        }
        
        if (ap_state_.mach_hold) {
            // Match thrust to drag while keeping a realistic minimum throttle.
            thrust_percentage = drag_total / max_thrust_n;
            thrust_percentage = std::clamp(thrust_percentage, 0.1, 1.0);
        }

        double thrust = max_thrust_n * thrust_percentage;

        SimTelemetry current_state;
        current_state.time_sec = time_sec_;
        current_state.load_factor = lift / weight_N;
        current_state.spec_excess_power = ((thrust - drag_total) * V) / weight_N;
        current_state.root_bending_moment = (lift / 2.0) * (wingspan_m / 4.0);
        current_state.drag_parasite = drag_parasite;
        current_state.drag_induced = drag_induced;
        current_state.drag_total = drag_total;
        current_state.dyn_pressure = dyn_pressure;
        current_state.lift = lift;
        current_state.thrust = thrust;
        current_state.fuel_flow = (thrust > 0.0) ? (thrust * tsfc_cruise) : 0.0;
        current_state.tsfc = tsfc_cruise;
        current_state.lift_drag_ratio = (drag_total > 0.0) ? (lift / drag_total) : 0.0;
        current_state.is_stalled = is_stalled;

        return current_state;
    }

    std::shared_ptr<const RenderData> generateRenderData() const {
        auto data = std::make_shared<RenderData>();
        data->pitch_angle = pitch_aoa_deg_;
        data->stalled = (pitch_aoa_deg_ > 15.0);

        constexpr double chord_m = 10.0;
        constexpr double leading_edge_x = -5.0;
        constexpr double camber = 0.02;
        constexpr double camber_position = 0.4;
        constexpr double thickness = 0.12;
        constexpr int section_segments = 100;

        auto sectionCoordinates = [=](double x) {
            const double x2 = x * x;
            const double x3 = x2 * x;
            const double x4 = x3 * x;
            const double thickness_shape = 5.0 * thickness *
                (0.2969 * std::sqrt(x) - 0.1260 * x - 0.3516 * x2 +
                 0.2843 * x3 - 0.1015 * x4);
            const double mean_line = x < camber_position
                ? (camber / (camber_position * camber_position)) *
                    (2.0 * camber_position * x - x2)
                : (camber / ((1.0 - camber_position) * (1.0 - camber_position))) *
                    ((1.0 - 2.0 * camber_position) + 2.0 * camber_position * x - x2);
            const double mean_line_slope = x < camber_position
                ? (2.0 * camber / (camber_position * camber_position)) * (camber_position - x)
                : (2.0 * camber / ((1.0 - camber_position) * (1.0 - camber_position))) *
                    (camber_position - x);
            const double angle = std::atan(mean_line_slope);
            return std::array<double, 4>{
                chord_m * (x + leading_edge_x - thickness_shape * std::sin(angle)),
                chord_m * (mean_line + thickness_shape * std::cos(angle)),
                chord_m * (x + leading_edge_x + thickness_shape * std::sin(angle)),
                chord_m * (mean_line - thickness_shape * std::cos(angle))
            };
        };

        auto rotateForPitch = [this](const Point2D& point) {
            const double angle = -pitch_aoa_deg_ * (M_PI / 180.0);
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);
            return Point2D{
                point.x * cosine - point.y * sine,
                point.x * sine + point.y * cosine
            };
        };

        for (int i = 0; i <= section_segments; ++i) {
            const double x = static_cast<double>(i) / section_segments;
            const auto section = sectionCoordinates(x);
            data->airfoil_top.push_back(rotateForPitch({section[0], section[1]}));
            data->airfoil_bottom.push_back(rotateForPitch({section[2], section[3]}));
        }

        // Estimate laminar boundary-layer separation from the thin-airfoil surface
        // velocity and Thwaites' momentum-thickness criterion.
        std::array<double, 2> separation_chord_x = {1.01, 1.01};
        const double alpha = pitch_aoa_deg_ * (M_PI / 180.0);
        constexpr int boundary_layer_samples = 160;
        constexpr double kinematic_viscosity = 1.5e-5;
        for (int side_index = 0; side_index < 2; ++side_index) {
            const double side = side_index == 0 ? 1.0 : -1.0;
            std::array<double, 3> fourier_coefficients{};
            for (int sample = 0; sample < boundary_layer_samples; ++sample) {
                const double theta = (static_cast<double>(sample) + 0.5) *
                                     M_PI / boundary_layer_samples;
                const double x = 0.5 * (1.0 - std::cos(theta));
                const double slope = (x < camber_position)
                    ? (2.0 * camber / (camber_position * camber_position)) * (camber_position - x)
                    : (2.0 * camber / ((1.0 - camber_position) * (1.0 - camber_position))) *
                        (camber_position - x);
                const double weight = M_PI / boundary_layer_samples;
                fourier_coefficients[0] +=
                    (alpha - slope) * (1.0 - std::cos(theta)) * weight / M_PI;
                fourier_coefficients[1] +=
                    2.0 * (alpha - slope) * std::cos(theta) * weight / M_PI;
                fourier_coefficients[2] +=
                    2.0 * (alpha - slope) * std::cos(2.0 * theta) * weight / M_PI;
            }

            const double a0 = fourier_coefficients[0];
            const double a1 = fourier_coefficients[1];
            const double a2 = fourier_coefficients[2];
            std::array<double, boundary_layer_samples + 1> edge_velocity{};
            std::array<double, boundary_layer_samples + 1> surface_distance{};
            for (int sample = 1; sample <= boundary_layer_samples; ++sample) {
                const double theta = static_cast<double>(sample) *
                                     M_PI / boundary_layer_samples;
                const double sine = std::max(std::sin(theta), 0.015);
                const double circulation_shape =
                    a0 * (1.0 + std::cos(theta)) / sine +
                    a1 * std::sin(theta) + a2 * std::sin(2.0 * theta);
                const double tangential_velocity = std::max(0.05,
                    1.0 + side * 2.0 * circulation_shape);
                edge_velocity[static_cast<size_t>(sample)] = tangential_velocity;
                const double x = 0.5 * (1.0 - std::cos(theta));
                surface_distance[static_cast<size_t>(sample)] = chord_m * x;
            }

            double velocity_integral = 0.0;
            for (int sample = 1; sample <= boundary_layer_samples; ++sample) {
                const double previous_velocity = edge_velocity[static_cast<size_t>(sample - 1)];
                const double velocity = edge_velocity[static_cast<size_t>(sample)];
                const double distance = surface_distance[static_cast<size_t>(sample)] -
                                        surface_distance[static_cast<size_t>(sample - 1)];
                velocity_integral += 0.5 * (std::pow(previous_velocity, 5.0) +
                                            std::pow(velocity, 5.0)) * distance;
                if (sample < 3 || velocity_integral <= 0.0) {
                    continue;
                }

                const double momentum_thickness_squared =
                    0.45 * kinematic_viscosity * velocity_integral /
                    std::pow(velocity, 6.0);
                const double velocity_gradient =
                    (velocity - previous_velocity) / std::max(distance, 1e-9);
                const double thwaites_lambda =
                    momentum_thickness_squared * velocity_gradient / kinematic_viscosity;
                if (thwaites_lambda <= -0.09) {
                    const double theta = static_cast<double>(sample) *
                                         M_PI / boundary_layer_samples;
                    separation_chord_x[static_cast<size_t>(side_index)] =
                        0.5 * (1.0 - std::cos(theta));
                    break;
                }
            }

            if (separation_chord_x[static_cast<size_t>(side_index)] <= 1.0) {
                const auto section = sectionCoordinates(
                    separation_chord_x[static_cast<size_t>(side_index)]);
                const Point2D surface = side > 0.0
                    ? Point2D{section[0], section[1]}
                    : Point2D{section[2], section[3]};
                data->separation_points.push_back(rotateForPitch(surface));

                const double separation_x = leading_edge_x +
                    separation_chord_x[static_cast<size_t>(side_index)] * chord_m;
                const double bubble_length = std::min(2.6,
                    std::max(0.7, (1.0 - separation_chord_x[static_cast<size_t>(side_index)]) *
                                  chord_m * 0.75));
                const double bubble_height = 0.18 + 0.22 *
                    (1.0 - separation_chord_x[static_cast<size_t>(side_index)]);
                std::vector<Point2D> vortex_loop;
                vortex_loop.reserve(49);
                for (int point = 0; point <= 48; ++point) {
                    const double angle = 2.0 * M_PI * point / 48.0;
                    const double x = separation_x + bubble_length * 0.5 +
                                     bubble_length * 0.5 * std::cos(angle);
                    const double local_chord = std::clamp(
                        (x - leading_edge_x) / chord_m, 0.0, 1.0);
                    const auto local_section = sectionCoordinates(local_chord);
                    const double surface_y = side > 0.0
                        ? local_section[1] : local_section[3];
                    const double y = surface_y + side *
                        (bubble_height + 0.14 + bubble_height * std::sin(angle));
                    vortex_loop.push_back(rotateForPitch({x, y}));
                }
                data->recirculation_zones.push_back(std::move(vortex_loop));
            }
        }

        // Streamlines follow the airfoil contour and expand into a recirculation
        // bubble downstream of any predicted separation point.
        const std::array<double, 4> streamline_offsets = {0.22, 0.45, 0.70, 1.0};
        const double pitch_rad = alpha;
        const double rotation_angle = -pitch_rad;
        const double rotation_cosine = std::cos(rotation_angle);
        const double rotation_sine = std::sin(rotation_angle);

        for (const double offset : streamline_offsets) {
            for (const double side : {1.0, -1.0}) {
            std::vector<Point2D> current_line;
            current_line.reserve(81);
            for (int i = 0; i <= 80; ++i) {
                const double x = -8.0 + static_cast<double>(i) * 0.2;
                const double chord_x = (x - leading_edge_x) / chord_m;
                const bool over_section = chord_x >= 0.0 && chord_x <= 1.0;
                double upper_surface = 0.0;
                double lower_surface = 0.0;
                if (over_section) {
                    const auto section = sectionCoordinates(chord_x);
                    upper_surface = section[1];
                    lower_surface = section[3];
                }

                const double bulge = over_section
                    ? 0.35 * std::exp(-std::pow((x + 1.0) / 2.4, 2.0))
                    : 0.0;
                double y = side * (offset + bulge);
                const double separation_x = leading_edge_x +
                    separation_chord_x[side > 0.0 ? 0U : 1U] * chord_m;
                if (separation_chord_x[side > 0.0 ? 0U : 1U] <= 1.0 &&
                    x > separation_x && x < separation_x + 2.6) {
                    const double progress = (x - separation_x) / 2.6;
                    y += side * 0.45 * std::sin(M_PI * progress);
                }

                if (side > 0.0) {
                    y = std::max(y, upper_surface + 0.16);
                } else {
                    y = std::min(y, lower_surface - 0.16);
                }

                current_line.push_back({x * rotation_cosine - y * rotation_sine,
                                        x * rotation_sine + y * rotation_cosine});
            }
            data->streamlines.push_back(std::move(current_line));
            }
        }
        return data;
    }
};

#endif